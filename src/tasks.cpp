#include "tasks.h"

using namespace op_space_control;
using std::cout;
using std::endl;

OperationalSpaceTask::OperationalSpaceTask()
{
    // TODO should be virtual
    _level = 1;
    _weight.resize(1); _weight[0] = 1;
    //_weight.push_back(1); // Ask Kris to implement it
    _name = "unnamed";
    _hP;
    _hD;
    _hI;
    _qLast;
    _xdes;
    _dxdes;
    _eI;
}

// vcmd = hP*eP + hD*eV + hI*eI
Vector OperationalSpaceTask::GetCommandVelocity( Vector q, Vector dq, double dt)
{
    Vector eP = GetSensedError( q );

    // P term
    Vector vP = eP * _hP;
    Vector vcmd = vP;
    Vector vcur = GetSensedVelocity( q, dq, dt );

    // D term
    if( vcur.size() != 0 )
    {
        Vector eD = vcur - _dxdes;
        Vector vD = eD * _hD;
        vcmd = vcmd + vD;
    }

    // I term
    if( _eI.size() != 0 )
    {
        Vector vI   = _eI * _hI;
        vcmd = vcmd + vI;
    }
    //print "task",_name,"error P=",eP,"D=",eD,"E=",_eI
    return vcmd;
}

// madd( a, b, c ) = a + c*b

void OperationalSpaceTask::Advance( Vector q, Vector dq, double dt )
{
    if( _weight.size() > 0 )
    {
        Vector eP = GetSensedError( q );

        // update iterm
        if( _eI.size() != 0 )
        {
            _eI = eP * dt;
        }
        else
        {
            _eI.madd( eP, dt );
        }
    }

    _qLast = q; // update qLast
}

Vector OperationalSpaceTask::GetSensedError(Vector q)
{
    // Returns x(q)-xdes where - is the task-space differencing operator
    return TaskDifference( GetSensedValue(q), _xdes );
}

Vector OperationalSpaceTask::TaskDifference( Vector a, Vector b )
{
    // Default: assumes a Cartesian space
    return a - b;
}

Vector OperationalSpaceTask::GetSensedVelocity(Vector q, Vector dq, double dt)
{
    // Gets task velocity from sensed configuration q.
    // Default implementation uses finite differencing.
    // Other implementations may use jacobian.

    // uncomment this to get a jacobian based technique
    // return np.dot(_getJacobian(q),dq)

    if( _qLast.size() > 0 )
    {
        return Vector();
    }
    else
    {
        Vector xlast = GetSensedValue( _qLast );
        Vector xcur = GetSensedValue( q );
        return TaskDifference( xcur, xlast ) / dt;
    }
}

void OperationalSpaceTask::DrawGL( Vector q )
{
    // Optionally can be overridden to visualize the task in OpenGL
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

std::vector<double> GetStdVector(const Vector3& pos)
{
    std::vector<double> vect(3);
    vect[0] = pos[0];
    vect[1] = pos[1];
    vect[2] = pos[2];
    return vect;
}

Vector3 GetVector3(const Vector& vect)
{
    Vector3 pos;
    pos[0] = vect[0];
    pos[1] = vect[1];
    pos[2] = vect[2];
    return pos;
}

void PushRotationToVector( const Matrix3& R, Vector& x )
{
    int size = x.size();
    Vector x_temp = x;
    x.resize(size+9);

    for(int i=0;i<size;i++) // Copy the old values to resized vector
        x[i] = x_temp[i];

    x[size+0] = R(0,0); x[size+1] = R(0,1); x[size+2] = R(0,2);
    x[size+3] = R(1,0); x[size+4] = R(1,1); x[size+5] = R(1,2);
    x[size+6] = R(2,0); x[size+7] = R(2,1); x[size+8] = R(2,2);
}

void PopRotationFromVector( Vector& x, Matrix3& R )
{
    int size = x.size();
    R(0,0) = x[size-9]; R(0,1) = x[size-8]; R(0,2) = x[size-7];
    R(1,0) = x[size-6]; R(1,1) = x[size-5]; R(1,2) = x[size-4];
    R(2,0) = x[size-3]; R(2,1) = x[size-2]; R(2,2) = x[size-1];

    Vector x_temp = x;
    x.resize(size-9);

    for(int i=0;i<x.size();i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];
}

void PushPosToVector( const Vector3& p, Vector& x )
{
    int size = x.size();
    Vector x_temp = x;
    x.resize(size+3);

    for(int i=0;i<size;i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];

    x[size+0] = p[0];
    x[size+1] = p[1];
    x[size+2] = p[2];
}

void PopPosFromVector( Vector& x, Vector3& p )
{
    int size = x.size();
    p[0]   = x[size-3];
    p[1]   = x[size-2];
    p[2]   = x[size-1];

    Vector x_temp = x;
    x.resize(size-3);

    for(int i=0;i<x.size();i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];
}

void PushFrameToVector( const Frame3D& T, Vector& x )
{
    PushRotationToVector( T.R, x );
    PushPosToVector( T.t, x );
}

void PopFrameFromVector( Vector& x, Frame3D& T )
{
    PopPosFromVector( x, T.t );
    PopRotationFromVector( x, T.R );
}

// used in python interface
Matrix GetJacobian( RobotDynamics3D& robot, int index, Vector3 p )
{
    Matrix Jmat;
    robot.GetFullJacobian( p, index, Jmat );
    return Jmat;
}

// used in python interface
Matrix GetPositionJacobian( RobotDynamics3D& robot, int index,  Vector3 p )
{
    Matrix Jmat;
    robot.GetPositionJacobian( p, index, Jmat );
    return Jmat;
}

// used in python interface
Matrix GetOrientationJacobian( RobotDynamics3D& robot, int index )
{
    Matrix Jmat;
    // TODO exeption
    //throw PyException("Orientation Jacobian not supported yet");
    //robot.GetOrientationJacobian(index,Jmat);
    return Jmat;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

COMTask::COMTask( RobotDynamics3D& robot, int baseLinkNo ) : OperationalSpaceTask(), _robot(robot)
{
    _baseLinkNo = baseLinkNo;
    _mass = GetMass();
    _name = "CoM";
}

double COMTask::GetMass()
{
    double mass = 0.0;
    // TODO implement mass
    //        for i in xrange(_robot.numLinks()):
    //            mi = _robot.getLink(i).getMass().getMass();
    //            mass += mi;
    return mass;
}

Vector COMTask::GetSensedValue(Vector q)
{
    _robot.UpdateConfig(q);

    Vector3 com_n(0.0);
    for(int i=0;i<int(_robot.links.size());i++)
    {
        RobotLink3D& link = _robot.links[i];
        Vector3 comi = link.com; // Check the reference frame
        com_n.madd( comi, link.mass );
    }

    Vector3 com = com_n / _mass;
    if(_baseLinkNo >= 0)
    {
        const Frame3D& Tb = _robot.links[_baseLinkNo].T_World;
        Frame3D Tbinv; Tbinv.setInverse(Tb);
        com = Tbinv * com;
    }

    Vector x;
    PushPosToVector( com, x );
    return x;
}

Matrix COMTask::GetJacobian(Vector q)
{
    _robot.UpdateConfig(q);
    int numLinks = _robot.links.size();
    Matrix Jcom( numLinks, 3, 0.0 );

    for ( int i=0; i<numLinks; i++ )
    {
        RobotLink3D& link = _robot.links[i];
        Matrix Ji; _robot.GetPositionJacobian( link.com, i, Ji );
        Ji *= link.mass;

        // Matrix Jcom_tmp( Jcom ); // TODO check that it is necessary

        Vector row;
        for ( int j=0; j<3; j++ )
        {
            Jcom.getRowRef( j, row );
            row = Jcom.row(j) + Ji.row(j); // TODO check that it's rows or coll ??
        }
    }

    Jcom /= _mass;
    // if relative positioning task, subtract out COM jacobian w.r.t. base
    if( _baseLinkNo >= 0 )
    {
        RobotLink3D& link =_robot.links[_baseLinkNo];
        Frame3D Tbinv; Tbinv.setInverse( link.T_World ); // TODO check if that should be world ??
        Vector3 pb = Tbinv * GetVector3( GetSensedValue( q ) );
        Matrix Jb; _robot.GetFullJacobian( pb, _baseLinkNo, Jb );

        Vector row;
        for ( int j=0; j<3; j++ )
        {
            Jcom.getRowRef( j, row );
            row = Jcom.row(j) - Jb.row(j); // TODO check that it's rows or coll ??
        }
    }
    return Jcom;
}

void COMTask::DrawGL(Config q)
{
    Vector3 x = GetVector3( GetSensedValue(q) );
    Vector3 xdes = GetVector3( _xdes );
    if(_baseLinkNo >= 0)
    {
        Frame3D Tb = _robot.links[_baseLinkNo].T_World;
        xdes = Tb * xdes;
        x = Tb * x;
    }
    // Link to OpenGL
    //    glPointSize(10);
    //    glEnable(GL_POINT_SMOOTH);
    //    glBegin(GL_POINTS);
    //    glColor3f(0,1,0);	//green
    //    glVertex3fv(xdes);
    //    glColor3f(0,1,1);	//cyan
    //    glVertex3fv(x);
    //    glEnd();
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

LinkTask::LinkTask( RobotDynamics3D& robot, int linkNo, std::string taskType, int baseLinkNo ) : OperationalSpaceTask(), _robot( robot )
{
    _linkNo = linkNo;
    _taskType = taskType;
    _baseLinkNo = baseLinkNo;
    _hP = -1;
    _hD = 0;
    _hI = 0;
    _localPosition[0] = 0;
    _localPosition[1] = 0;
    _localPosition[2] = 0;
    _name = "Link";

    if( _taskType == "po" || _taskType == "position" || _taskType == "orientation" )
    {
        //pass
    }
    else
    {
        //raise ValueError("Invalid taskType "+_taskType)
    }
}

Vector LinkTask::GetSensedValue( Vector q )
{
    _robot.UpdateConfig(q);
    Frame3D T = _robot.links[_linkNo].T_World;

    // check if relative transform task, modify T to local transform
    if( _baseLinkNo >= 0 )
    {
        const Frame3D& Tb = _robot.links[_baseLinkNo].T_World;
        Frame3D Tbinv; Tbinv.setInverse(Tb);
        T = Tbinv * T;
    }

    Vector x;
    x.clear();

    if( _taskType == "po" )
    {
        T.inplaceShift( _localPosition ); // TODO check python
        PushFrameToVector( T, x );

    }
    else if( _taskType == "position" )
    {
        PushPosToVector( T*_localPosition, x );
    }
    else if( _taskType == "orientation" )
    {
        PushRotationToVector( T.R, x );
    }
    else
    {
        // raise ValueError("Invalid taskType "+_taskType);
    }
    return x;
}

Vector LinkTask::TaskDifference( Vector a, Vector b )
{
    if( _taskType == "po" )
    {
        Frame3D Ta;
        Frame3D Tb;
        PopFrameFromVector( a, Ta );
        PopFrameFromVector( a, Tb );

        Matrix3 Rbinv; Rbinv.setInverse( Tb.R );
        //Ta.R * Rbinv  + (a - b); // TODO add the moment comutation see python
        Vector x_buff;
        return x_buff;
    }
    else if( _taskType == "position" )
    {
        return (a - b);
    }
    else if( _taskType == "orientation" )
    {
        Matrix3 Ra;
        Matrix3 Rb;
        PopRotationFromVector( a, Ra );
        PopRotationFromVector( b, Rb );
        Vector x_buff; // TODO rotation see python
        return x_buff;
    }
    else
    {
        // raise ValueError("Invalid taskType "+_taskType)
        Vector x_buff;
        return x_buff;
    }
}

Matrix LinkTask::GetJacobian( Config q )
{
    _robot.UpdateConfig(q);

    Matrix J; // Jacobian of the Link

    if( _taskType == "po" )
    {
        _robot.GetFullJacobian( _localPosition, _linkNo, J );
    }
    else if( _taskType == "position" )
    {
        _robot.GetPositionJacobian( _localPosition, _linkNo, J );
    }
    else if( _taskType == "orientation" )
    {
        //_robot.GetOrientationJacobian( i, J ); TODO
    }
    else
    {
        // TODO throw exeption
        // raise ValueError("Invalid taskType "+_taskType)
    }
    // check if relative transform task, modify Jacobian accordingly
    if( _baseLinkNo >= 0 )
    {
        const Frame3D& T = _robot.links[_linkNo].T_World;
        const Frame3D& Tb = _robot.links[_baseLinkNo].T_World;
        Frame3D Tbinv; Tbinv.setInverse( Tb );
        Vector3 pb = Tbinv*(T*_localPosition);
        Matrix Jb;

        if( _taskType == "po" )
        {
            _robot.GetFullJacobian( pb, _baseLinkNo, Jb );
        }
        else if( _taskType == "position" )
        {
            _robot.GetPositionJacobian( pb, _baseLinkNo, Jb );
        }
        else if( _taskType == "orientation" )
        {
            //_robot.links(_baseLinkNo).GetOrientationJacobian(Jb); TODO
        }

        Vector row;
        for ( int j=0; j<3; j++ )
        {
            J.getRowRef( j, row );
            row = J.row(j) - Jb.row(j); // TODO check that it's rows or coll ??
        }
    }
    return J;
}

Vector LinkTask::DrawGL(Vector q)
{
    Vector x_buff = GetSensedValue(q);

    //    glPointSize(6);
    //    glEnable(GL_POINT_SMOOTH);
    //    glBegin(GL_POINTS);

    const Frame3D& Tb = _robot.links[_baseLinkNo].T_World;

    if( _taskType == "position" )
    {
        Vector3 x;
        Vector3 xdes;
        PopPosFromVector(  x_buff, x );
        PopPosFromVector(  _xdes, xdes );
        if ( _baseLinkNo >= 0 )
        {
            x = Tb * x;
            xdes = Tb * xdes;
        }

        //        glColor3f(1,0,0);	//red
        //        glVertex3fv(xdes);
        //        glColor3f(1,0.5,0);	//orange
        //        glVertex3fv(x);
    }
    else if( _taskType == "po" )
    {
        Frame3D x;
        Frame3D xdes;
        PopFrameFromVector( x_buff, x );
        PopFrameFromVector( _xdes, xdes );

        if ( _baseLinkNo >= 0 )
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

JointTask::JointTask( RobotDynamics3D& robot, const std::vector<int>& jointIndices ) : OperationalSpaceTask() , _robot(robot)
{
    _jointIndices = jointIndices;
    _name = "Joint";
    // pass
}

Vector JointTask::GetSensedValue( Config q )
{
    Vector x( _jointIndices.size() );

    for(int i=0;i<(_jointIndices.size());i++)
        x[i] = q[_jointIndices[i]];

    return x;
}

Matrix JointTask::GetJacobian( Config q )
{
    Matrix J(  _jointIndices.size(), q.size()  );

    Vector row;
    for(int i=0;i<int(_jointIndices.size());i++)
    {
        Vector Ji( _robot.links.size(), 0.0 );
        Ji( _jointIndices[i] ) = 1;
        J.getRowRef( i, row );
        row = Ji; // TODO check that it's rows or coll ??
    }
    return J;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

JointLimitTask::JointLimitTask( RobotDynamics3D& robot ) : OperationalSpaceTask() , _robot(robot)
{
    _buffersize = 2.0;
    _qMin =  robot.qMin;
    _qMax = robot.qMax;
    _aMax = Vector( robot.q.n, 0.0 ); // TODO get accelerations, see if urdf adds Acc
    _wscale = 0.1;
    _maxw = 10;
    _active.clear();
    _weight.clear();
    _name = "Joint limits";
    SetGains(-0.1,-2.0,0);
}

Vector JointLimitTask::GetSensedValue( Config q )
{
    Vector x( _active.size() );

    for(int i=0;i<(_active.size());i++)
        x[i] = q[_active[i]];

    return x;
}

Matrix JointLimitTask::GetJacobian( Config q )
{
    Matrix J( _active.size(), q.size() );

    Vector row;
    for(int i=0;i<int(_active.size());i++)
    {
        Vector Ji( _robot.q.size(), 0.0 );
        Ji( _active[i] ) = 1;
        J.getRowRef( i, row );
        row = Ji; // TODO check that it's rows or coll ??
    }

    return J;
}

void JointLimitTask::UpdateState( Config q, Vector dq, double dt )
{
    _active.clear();
    _weight.clear();
    _xdes.clear();
    _dxdes.clear();
    double buffersize = _buffersize;
    double wscale = _wscale;
    double maxw = _maxw;

    int s = q.size(); // Check sizes of array
    bool size_ok = ( s == dq.size() ) &&  ( s == _qMin.size() ) && ( s == _aMax.size() );
    if( !size_ok )
    {
        cout << "Error in the joint array size of joint limit taks, line " << __LINE__ << endl;
        return;
    }

    for( int i=0; i<s; i++ )
    {
        double j = q[i];
        double dj= dq[i];
        double jmin = _qMin[i];
        double jmax = _qMax[i];
        double amax = _aMax[i]; // TODO get accel

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

        _active.push_back(i);
        _weight.push_back(w);

        _xdes.resizePersist(_xdes.size()+1); // TODO ask kris for push_back function
        _xdes[_xdes.size()-1] = std::max(jmin,std::min(jmax,j));

        _dxdes.resizePersist(_dxdes.size()+1);
        _dxdes[_dxdes.size()-1] = dj+dt*ades;
    }

    if( _weight.empty() )
    {
        _weight.push_back( 0 );
    }
    return;
}
