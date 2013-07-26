#include "tasks.h"
#include "op_utils.h"

using namespace op_space_control;
using std::cout;
using std::endl;

OperationalSpaceTask::OperationalSpaceTask( std::vector<std::string>& linkNames ) : linkNames_(linkNames)
{
    // TODO should be virtual
    level_ = 1;
    weight_.resize(1); weight_[0] = 1; // TODO ask kris about printf in resize
    //weight_.push_back(1); // Ask Kris to implement it
    name_ = "unnamed";
    hP_ = -1;
    hD_ = 0;
    hI_ = 0;
    qLast_ = Vector(0);
    xdes_= Vector(1,0.0);
    dxdes_= Vector(1,0.0);
    eI_ = Vector(0);
}

// vcmd = hP*eP + hD*eV + hI*eI
Vector OperationalSpaceTask::GetCommandVelocity( const Config& q, const Vector&  dq, double dt )
{
    Vector eP = GetSensedError( q );
    if( HasNaN(eP) )
    {
        cout << "eP has nan" << endl;
    }

    Vector vcur = GetSensedVelocity( q, dq, dt );
    if( HasNaN(vcur) )
    {
        cout << "vcur has nan" << endl;
    }

    // P term
    Vector vP = eP * hP_;
    Vector vcmd = vP;

//    cout << "GetSensedError : " << eP << endl;

//    vcmd[0] = 1;
//    vcmd[1] = 0;
//    vcmd[2] = 0;
//    vcmd[3] = 0;
//    vcmd[4] = 0;
//    vcmd[5] = 0;

//    cout << "hP_ : " << hP_ << endl;
//    cout << "hD_ : " << hD_ << endl;
//    cout << "hI_ : " << hI_ << endl;

    // D term
    if( vcur.size() != 0 )
    {
        Vector eD = vcur - dxdes_;
        Vector vD = eD * hD_;
        vcmd = vcmd + vD;
    }

    // I term
    if( eI_.size() != 0 )
    {
        Vector vI   = eI_ * hI_;
        vcmd = vcmd + vI;
    }
    //print "task",name_,"error P=",eP,"D=",eD,"E=",eI_

    return vcmd;
}

// madd( a, b, c ) = a + c*b

void OperationalSpaceTask::Advance( const Config& q, const Vector&  dq, double dt )
{
    if( weight_.size() > 0 )
    {
        Vector eP = GetSensedError( q );

        // update iterm
        if( eI_.empty() )
        {
            eI_ = eP * dt;
        }
        else
        {
            eI_.madd( eP, dt );
        }
    }

    qLast_ = q; // update qLast
}

//! Returns x(q)-xdes where - is the task-space differencing operator
Vector OperationalSpaceTask::GetSensedError( const Vector& q)
{
    return TaskDifference( GetSensedValue(q), xdes_ );
}

Vector OperationalSpaceTask::TaskDifference( const Vector& a, const Vector& b )
{
    // Default: assumes a Cartesian space
    return a - b;
}

Vector OperationalSpaceTask::GetSensedVelocity( const Config& q, const Vector&  dq, double dt )
{
    // Gets task velocity from sensed configuration q
    // Default implementation uses finite differencing
    // Other implementations may use jacobian

    // uncomment this to get a jacobian based technique
    // return np.dot(_getJacobian(q),dq)

    if( qLast_.empty() )
    {
        return Vector();
    }
    else
    {
        Vector xlast = GetSensedValue( qLast_ );
        Vector xcur = GetSensedValue( q );
        return TaskDifference( xcur, xlast ) / dt;
    }
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

COMTask::COMTask( RobotDynamics3D& robot, std::vector<std::string>& linkNames, int baseLinkNo ) :robot_(robot), OperationalSpaceTask(linkNames)
{
    baseLinkNo_ = baseLinkNo;
    mass_ = GetMass();
    cout << "Robot total mass is : " << mass_ << endl;
    name_ = "CoM";
}

double COMTask::GetMass()
{
    double mass = 0.0; // Compute total mass !! TODO check against robot class function

    for(int i=0;i<int(robot_.links.size());i++)
    {
        mass += robot_.links[i].mass;
    }

    return mass;
}

Vector COMTask::GetSensedValue( const Config& q )
{
    robot_.UpdateConfig(q);

    // Compute the robot center of mass !! TODO check against robot class function
    Vector3 com(0.0);
    for(int i=0;i<int(robot_.links.size());i++)
    {
        const RobotLink3D& link = robot_.links[i];
        com.madd( link.T_World * link.com, link.mass );
    }
    com /= mass_;

    if( baseLinkNo_ >= 0 )
    {
        Frame3D Tbinv;
        Tbinv.setInverse( robot_.links[baseLinkNo_].T_World );
        com = Tbinv * com;
    }

    cout << "com : " << com << endl;

    Vector x;
    PushPosToVector( com, x );
    return x;
}

Matrix COMTask::GetJacobian( const Config& q )
{
    robot_.UpdateConfig(q);

    int numLinks = robot_.links.size();
    Matrix Jcom( 3, numLinks, 0.0 );
    for ( int i=0; i<numLinks; i++ )
    {
        Matrix Ji; // Position jacobian of ith link com, (3,q.n)(rows,cols)
        const RobotLink3D& link = robot_.links[i];
        robot_.GetPositionJacobian( link.com, i, Ji );
        Ji *= link.mass;

        Vector row;
        for ( int j=0; j<3; j++ ) // on x,y,z
        {
            Jcom.getRowRef( j, row );
            row = Jcom.row(j) + Ji.row(j);
        }
    }

    Jcom /= mass_; // Divide by robot total mass

    // If relative positioning task, subtract out COM jacobian w.r.t. base
    if( baseLinkNo_ >= 0 )
    {
        Matrix Jb; // Full jacobian with respect to base, (6,q.n)(rows,cols)
        Frame3D Tbinv;
        Tbinv.setInverse( robot_.links[baseLinkNo_].T_World ); // TODO Ask Kris for getInverse in Frame3D
        Vector3 pb = Tbinv * GetVector3( GetSensedValue( q ) );
        robot_.GetFullJacobian( pb, baseLinkNo_, Jb );

        Vector row;
        for ( int j=0; j<3; j++ ) // on x,y,z
        {
            Jcom.getRowRef( j, row );
            row = Jcom.row(j) - Jb.row(j);
        }
    }

    cout << "Jcom : " << Jcom << endl;
    return Jcom;
}

void COMTask::DrawGL( const Config& q)
{
    Vector3 x = GetVector3( GetSensedValue(q) );
    Vector3 xdes = GetVector3( xdes_ );

    if(baseLinkNo_ >= 0)
    {
        Frame3D Tb = robot_.links[baseLinkNo_].T_World;
        xdes = Tb * xdes;
        x = Tb * x;
    }

// TODO link to OpenGL
//    glPointSize(10);
//    glEnable(GL_POINT_SMOOTH);
//    glBegin(GL_POINTS);
//    glColor3f(0,1,0);	//green
//    glVertex3fv( GetGlVector3(xdes) );
//    glColor3f(0,1,1);	//cyan
//    glVertex3fv( GetGlVector3(x) );
//    glEnd();
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

LinkTask::LinkTask( RobotDynamics3D& robot, std::vector<std::string>& linkNames, int linkNo, std::string taskType, int baseLinkNo ) : OperationalSpaceTask(linkNames), robot_( robot )
{
    linkNo_ = linkNo;
    baseLinkNo_ = baseLinkNo;
    hP_ = -1;
    hD_ = 0;
    hI_ = 0;
    localPosition_[0] = 0;
    localPosition_[1] = 0;
    localPosition_[2] = 0;
    name_ = "Link";

    SetTaskType( taskType );
}

void LinkTask::SetTaskType(const std::string& taskType)
{
    if( taskType == "po" )
    {
        taskType_ = po;
    }
    else if( taskType == "position" )
    {
        taskType_ = position;
    }
    else if( taskType == "orientation" )
    {
        taskType_ = orientation;
    }
    else
    {
        //raise ValueError("Invalid taskType "+taskType_) TODO exeption
    }
}

Vector LinkTask::GetSensedValue( const Config& q )
{
    robot_.UpdateConfig(q);

    Frame3D T = robot_.links[linkNo_].T_World;

    // Check if relative transform task, modify T to local transform
    if( baseLinkNo_ >= 0 )
    {
        Frame3D Tbinv;
        Tbinv.setInverse( robot_.links[baseLinkNo_].T_World );
        T = Tbinv * T;
    }

    Vector x(0);

    if( taskType_ == po )
    {
        cout << "T : " << T * localPosition_ << endl;
        T.t = Vector3( T * localPosition_ );
        PushFrameToVector( T, x );
    }
    else if( taskType_ == position )
    {
        PushPosToVector( T*localPosition_, x );
    }
    else if( taskType_ == orientation )
    {
        PushRotationToVector( T.R, x );
    }
    else
    {
        // raise ValueError("Invalid taskType "+taskType_); TODO exeption
    }
    return x;
}

Vector LinkTask::TaskDifference( const Vector& a, const Vector& b )
{
    if( taskType_ == po )
    {
        Frame3D Ta;
        Frame3D Tb;
        Vector atmp = a;
        Vector btmp = b;
        PopFrameFromVector( atmp, Ta ); // TODO change pile
        PopFrameFromVector( btmp, Tb );

        Vector3 p_e = Ta.t - Tb.t;
        Vector3 o_e = Error( Ta.R, Tb.R );

        Vector x_e(6);
        x_e[0] = p_e[0];
        x_e[1] = p_e[1];
        x_e[2] = p_e[2];
        x_e[3] = o_e[0];
        x_e[4] = o_e[1];
        x_e[5] = o_e[2];
        return x_e;
    }
    else if( taskType_ == position )
    {
        return (a - b);
    }
    else if( taskType_ == orientation )
    {
        Matrix3 Ra;
        Matrix3 Rb;
        Vector atmp = a;
        Vector btmp = b;
        PopRotationFromVector( atmp, Ra );
        PopRotationFromVector( btmp, Rb );
        return GetVector( Error( Ra, Rb ) );
    }
    else
    {
        // raise ValueError("Invalid taskType "+taskType_) TODO exeption
        Vector x_e;
        return x_e;
    }
}

Matrix LinkTask::GetJacobian( const Config& q )
{
    robot_.UpdateConfig(q);

    Matrix J; // Jacobian of the Link

    if( taskType_ == po )
    {
        robot_.GetFullJacobian( localPosition_, linkNo_, J );
    }
    else if( taskType_ == position )
    {
        robot_.GetPositionJacobian( localPosition_, linkNo_, J );
    }
    else if( taskType_ == orientation )
    {
        //robot_.GetOrientationJacobian( i, J ); TODO
    }
    else
    {
        // TODO throw exeption
        // raise ValueError("Invalid taskType "+taskType_)
    }
    // check if relative transform task, modify Jacobian accordingly
    if( baseLinkNo_ >= 0 )
    {
        Matrix Jb; // Jacobian

        Frame3D Tbinv;
        Tbinv.setInverse( robot_.links[baseLinkNo_].T_World );
        Vector3 pb = Tbinv * ( robot_.links[linkNo_].T_World * localPosition_ );

        if( taskType_ == po )
        {
            robot_.GetFullJacobian( pb, baseLinkNo_, Jb );
        }
        else if( taskType_ == position )
        {
            robot_.GetPositionJacobian( pb, baseLinkNo_, Jb );
        }
        else if( taskType_ == orientation )
        {
            //robot_.links(baseLinkNo_).GetOrientationJacobian(Jb); TODO
        }

        Vector row;
        for ( int i=0; i<J.numRows(); i++ )
        {
            J.getRowRef( i, row );
            row = J.row(i) - Jb.row(i);
        }
    }

    if( name_ == "Left Foot" ) {
        cout << "J : " << J << endl;
    }
    if( name_ == "Left Hand"){
        cout << "J : " << J << endl;
    }

    return J;
}

void LinkTask::DrawGL( const Vector& q )
{
    Vector x_buff = GetSensedValue(q);

    // TODO link to openGl
    // glPointSize(6);
    // glEnable(GL_POINT_SMOOTH);
    // glBegin(GL_POINTS);

    const Frame3D& Tb = robot_.links[baseLinkNo_].T_World;

    if( taskType_ == position )
    {
        Vector3 x;
        Vector3 xdes;
        PopPosFromVector(  x_buff, x );
        PopPosFromVector(  xdes_, xdes );
        if ( baseLinkNo_ >= 0 )
        {
            x = Tb * x;
            xdes = Tb * xdes;
        }

        // glColor3f(1,0,0);	//red
        // glVertex3fv(xdes);
        // glColor3f(1,0.5,0);	//orange
        // glVertex3fv(x);
    }
    else if( taskType_ == po )
    {
        Frame3D x;
        Frame3D xdes;
        PopFrameFromVector( x_buff, x );
        PopFrameFromVector( xdes_, xdes );

        if ( baseLinkNo_ >= 0 )
        {
            x = Tb * x;
            xdes = Tb * xdes;
        }

        //        glColor3f(1,0,0);	//red
        //        glVertex3fv(xdes[1]);
        //        glColor3f(1,0.5,0);	//orange
        //        glVertex3fv(x[1]);
        //        glEnd();
    }
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

JointTask::JointTask( RobotDynamics3D& robot, std::vector<std::string>& linkNames, const std::vector<int>& jointIndices ) : OperationalSpaceTask(linkNames) , robot_(robot)
{
    jointIndices_ = jointIndices;
    name_ = "Joint";
    // pass
}

Vector JointTask::GetSensedValue( const Config& q )
{
    Vector x( jointIndices_.size() );

    for(int i=0;i<int(jointIndices_.size());i++)
        x[i] = q[jointIndices_[i]];

    return x;
}

Matrix JointTask::GetJacobian( const Config& q )
{
    Matrix J( jointIndices_.size(), q.size()  );

    Vector row;
    for(int i=0;i<int(jointIndices_.size());i++)
    {
        Vector Ji( robot_.links.size(), 0.0 );
        Ji( jointIndices_[i] ) = 1;
        J.getRowRef( i, row );
        row = Ji;
    }
    return J;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

JointLimitTask::JointLimitTask( RobotDynamics3D& robot, std::vector<std::string>& linkNames ) : OperationalSpaceTask(linkNames) , robot_(robot)
{
    buffersize_ = 2.0;
    qMin_ =  robot.qMin;
    qMax_ = robot.qMax;
    aMax_ = Vector( robot.q.n, 0.0 ); // TODO get accelerations, see if urdf adds Acc
    wscale_ = 0.1;
    maxw_ = 10;
    active_.clear();
    weight_.clear();
    name_ = "Joint limits";
    SetGains(-0.1,-2.0,0);
}

Vector JointLimitTask::GetSensedValue( const Config& q )
{
    Vector x( active_.size() );

    for(int i=0;i<(active_.size());i++)
        x[i] = q[active_[i]];

    return x;
}

Matrix JointLimitTask::GetJacobian( const Config& q )
{
    Matrix J( active_.size(), q.size() );

    Vector row;
    for(int i=0;i<int(active_.size());i++)
    {
        Vector Ji( robot_.q.size(), 0.0 );
        Ji( active_[i] ) = 1;
        J.getRowRef( i, row );
        row = Ji; // TODO check that it's rows or coll ??
    }
    return J;
}

// TODO test this task
void JointLimitTask::UpdateState(  const Config& q, const Vector&  dq, double dt )
{
    active_.clear();
    weight_.clear();
    xdes_.clear();
    dxdes_.clear();
    double buffersize = buffersize_;
    double wscale = wscale_;
    double maxw = maxw_;

    int s = q.size(); // Check sizes of array
    bool size_ok = ( s == dq.size() ) &&  ( s == qMin_.size() ) && ( s == aMax_.size() );
    if( !size_ok )
    {
        cout << "Error in the joint array size of joint limit taks, line " << __LINE__ << endl;
        return;
    }

    for( int i=0; i<s; i++ )
    {
        double j = q[i];
        double dj= dq[i];
        double jmin = qMin_[i];
        double jmax = qMax_[i];
        double amax = aMax_[i]; // TODO get accel

        if( jmax <= jmin )
            continue;

        int jstop = j;
        double a = amax / buffersize;
        double w = 0;
        double ades = 0;

        if( dj > 0.0 )
        {
            double t = dj / a;
            //j + t*dj - t^2*a/2
            jstop = j + t*dj - t*t*a*0.5;
            if( jstop > jmax )
            {
                //add task to slow down
                //perfect accel solves for:
                //j+ dj^2 / 2a  = jmax
                //dj^2 / 2(jmax-j)   = a
                if( j >= jmax )
                {
                    //cout<<  "Joint" self.robot.getLink(i).getName(),"exceeded max",j,">=",jmax
                    ades = -amax;
                    w = maxw;
                }
                else
                {
                    double alim = dj*dj/(jmax-j)*0.5;

                    if( alim > amax)
                    {
                        ades = -amax;
                        w = maxw;
                    }
                    else{
                        ades = -alim;
                        w = wscale*(alim-a)/(amax-alim);
                    }
                }
            }
            //print "Joint",self.robot.getLink(i).getName(),j,dj,"near upper limit",jmax,", desired accel:",ades," weight",w
        }
        else
        {
            double t = -dj / a;
            // + t*dj + t^2*a/2
            jstop = j + t*dj + t*t*a*0.5;
            if( jstop < jmin )
            {
                //add task to slow down
                //perfect accel solves for:
                //j - dj^2 / 2a  = jmin
                //dj^2 / 2(j-jmin)   = a
                if( j <= jmin )
                {
                    //print "Joint",self.robot.getLink(i).getName(),"exceeded min",j,"<=",jmin
                    ades = amax;
                    w = maxw;
                }
                else
                {
                    double alim = dj*dj/(j-jmin)*0.5;
                    if( alim > amax )
                    {
                        ades = amax;
                        w = maxw;
                    }
                    else
                    {
                        ades = alim;
                        w = wscale*(alim-a)/(amax-alim);
                        // print "Joint",self.robot.getLink(i).getName(),j,dj,"near lower limit",jmin,", desired accel:",ades," weight",w
                    }
                }
            }
        }

        if( w > maxw )
            w = maxw;

        active_.push_back(i);
        weight_.push_back(w);

        xdes_.resizePersist(xdes_.size()+1); // TODO ask kris for push_back function
        xdes_[xdes_.size()-1] = std::max(jmin,std::min(jmax,j));

        dxdes_.resizePersist(dxdes_.size()+1);
        dxdes_[dxdes_.size()-1] = dj+dt*ades;
    }

    if( weight_.empty() )
    {
        weight_.push_back( 0 );
    }
    return;
}
