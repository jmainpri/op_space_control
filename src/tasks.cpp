#include "tasks.h"

OperationalSpaceTask::OperationalSpaceTask()
{
    // TODO should be virtual
    _level = 1;
    _weight = 1;
    _name = 'unnamed';
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
        Vector eD = vcur.sub( _dxdes );
        Vector vD = eD.mul( _hD );
        vcmd = vcmd.add( vD );
    }

    // I term
    if( _eI.size() != 0 )
    {
        Vector vI   = _eI.mul( _hI );
        vcmd = vcmd.add( vI );
    }
    //print "task",_name,"error P=",eP,"D=",eD,"E=",_eI
    return vcmd;
}

// madd( a, b, c ) = a + c*b

void OperationalSpaceTask::Advance( Vector q, Vector dq, double dt )
{
    if ( _weight > 0 )
    {
        Vector eP = GetSensedError( q );

        // update iterm
        if( _eI.size() != 0 )
        {
            _eI = eP.mul( dt );
        }
        else
        {
            _eI = _eI.madd( eP, dt );
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
    return a.sub(b);
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

COMTask::COMTask( RobotDynamics3D& robot, int baseLinkNo = -1 ) : Task()
{
    _robot = robot;
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

    Vector3 com_n;
    for(int i=0;i<int(_robot.links.size());i++)
    {
        RobotLink3D& link = _robot.links(i);
        Vector3 comi = link.GetWorldCOM();
        mi = link.mass;
        com_n = com_n.madd( comi, mi );
    }

    Vector com = com_n / _mass;
    if(_baseLinkNo >= 0)
    {
        Frame3D Tb = _robot.links[_baseLinkNo].T_World;
        Frame3D Tbinv; Tb.inverse();
        com = Tbinv * com;
    }
    return com;
}

Vector COMTask::GetJacobian(Vector q)
{
    _robot.UpdateConfig(q);
    int numLinks = _robot.links.size();
    Matrix Jcom( numLinks, 3, 0.0 );
    for ( int i=0; i<numLinks; i++ )
    {
        RobotLink3D& link = _robot.links[i];
        mi = link.mass;
        comi = link.com;

        link.GetPositionJacobian( q[i], comi, Ji );
        Ji *= mi;

        for ( int j=0; j<numLinks; j++ )
        {
            Jcom[j] = Jcom[j] + Ji[j];
        }
    }
    Jcom /= _mass;
    // if relative positioning task, subtract out COM jacobian w.r.t. base
    if( _baseLinkNo >= 0 )
    {
        RobotLink3D& link =_robot.links[_baseLinkNo];
        Tb = link.T_World;
        Tbinv = T_World.inv(Tb);
        pb = se3.apply( Tbinv, GetSensedValue(q) );
        Jb = _link.GetJacobian(pb);
        for( i in xrange(len(Jcom) )
        {
            Jcom[i] = vectorops.sub(Jcom[i],Jb[i])
        }
    }
    return Jcom;
}

void COMTask::DrawGL(Config q)
{
    x = GetSensedValue(q);
    xdes = _xdes;
    if(_baseLinkNo >= 0)
    {
        Tb = _robot.links[_baseLinkNo].GetTransform();
        xdes = se3.apply(Tb,_xdes);
        x = se3.apply(Tb,x);
    }
    glPointSize(10);
    glEnable(GL_POINT_SMOOTH);
    glBegin(GL_POINTS);
    glColor3f(0,1,0);	//green
    glVertex3fv(xdes);
    glColor3f(0,1,1);	//cyan
    glVertex3fv(x);
    glEnd();
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

LinkTask::LinkTask( RobotDynamics3D& robot, int linkNo, std::string taskType, int baseLinkNo = -1 ) : Task()
{
    _robot = robot;
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

    if( _taskType == 'po' || _taskType == 'position' || _taskType == 'orientation' )
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
    T = _robot.links[_linkNo].getTransform();
    //check if relative transform task, modify T to local transform
    if( _baseLinkNo >= 0 )
    {
        Tb = _robot.links[_baseLinkNo].T_World;
        Tbinv = se3.inv(Tb);
        T = se3.mul(Tbinv,T);
    }
    if( _taskType == 'po' )
    {
        x = (T[0],se3.apply(T,_localPosition));
    }
    else if( _taskType == 'position' )
    {
        x = se3.apply(T,_localPosition);
    }
    else if( _taskType == 'orientation' )
    {
        x = T[0]
    }
    else
    {
        // raise ValueError("Invalid taskType "+_taskType);
    }
    return x;
}

Vector LinkTask::TaskDifference( Vector a, Vector b )
{
    if( _taskType == 'po' )
    {
        return se3.error(a,b);
    }
    else if( _taskType == 'position' )
    {
        return vectorops.sub(a,b);
    }
    else if( _taskType == 'orientation' )
    {
        return so3.error(a,b);
    }
    else
    {
        // raise ValueError("Invalid taskType "+_taskType)
    }
}

Vector LinkTask::GetJacobian( Vector q )
{
    _robot.UpdateConfig(q);
    J = None;
    if( _taskType == 'po' )
    {
        J = _robot.links[_linkNo].getJacobian(_localPosition);
    }
    else if _taskType == 'position' )
    {
        J = _robot.links[_linkNo].getPositionJacobian(_localPosition);
    }
    else if( _taskType == 'orientation' )
    {
        J = _robot.getLink(_linkNo).getOrientationJacobian();
    }
    else
    {
        // raise ValueError("Invalid taskType "+_taskType)
    }
    // check if relative transform task, modify Jacobian accordingly
    if( _baseLinkNo >= 0 )
    {
        T = _robot.links(_linkNo).getTransform();
        Tb = _robot.links(_baseLinkNo).getTransform();
        Tbinv = se3.inv(Tb);
        pb = se3.apply(Tbinv,se3.apply(T,_localPosition));
        if( _taskType == 'po' )
        {
            Jb = _robot.links(_baseLinkNo).getJacobian(pb);
        }
        else if( _taskType == 'position' )
        {
            Jb = _robot.links(_baseLinkNo).getPositionJacobian(pb);
        }
        else if( _taskType == 'orientation' )
        {
            Jb = _robot.links(_baseLinkNo).getOrientationJacobian();
        }
        // subtract out jacobian w.r.t. baseLink
        for(int i=0; in; xrange(len(J)))
        {
            J[i] = vectorops.sub(J[i],Jb[i]);
        }
    }
    return J;
}


Vector LinkTask::DrawGL(Vector q)
{
    x = GetSensedValue(q);
    xdes = _xdes;
    if( _baseLinkNo >= 0 )
    {
        Tb = _robot.links(_baseLinkNo).getTransform();
        if( _taskType == "position" )
        {
            x = se3.apply(Tb,x);
            xdes = se3.apply(Tb,xdes);
        }
        else if( _taskType == "po" )
        {
            x = se3.mul(Tb,x);
            xdes = se3.mul(Tb,xdes);
        }
    }
    glPointSize(6);
    glEnable(GL_POINT_SMOOTH);
    glBegin(GL_POINTS);
    if( _taskType == "position" )
    {
        glColor3f(1,0,0);	//red
        glVertex3fv(xdes);
        glColor3f(1,0.5,0);	//orange
        glVertex3fv(x);
    }
    else if( _taskType == "po" )
    {
        glColor3f(1,0,0);	//red
        glVertex3fv(xdes[1]);
        glColor3f(1,0.5,0);	//orange
        glVertex3fv(x[1]);
        glEnd();
    }
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

JointTask::JointTask( RobotDynamics3D& robot, const std::vector<int>& jointIndices ) : Task()
{
    _robot = robot;
    _jointIndices = jointIndices;
    _name = "Joint";
    // pass
}

Vector JointTask::GetSensedValue( Vector q )
{
    return [q[jointi] for jointi in _jointIndices];
}

Vector JointTask::GetJacobian( Vector q )
{
    J = [];
    for( jointi in _jointIndices )
    {
        Ji = [0] * _robot.links.size();
    }
    Ji[jointi] = 1;
    J.append(Ji);
    return J;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

JointLimitTask::JointLimitTask() : Task()
{
    //    def __init__(self,robot):
    //        Task.__init__(self)
    _robot = robot;
    _buffersize = 2.0;
    _qmin,_qmax = robot.getJointLimits();
    _accelMax = robot.getAccelerationLimits();
    _wscale = 0.1;
    _maxw = 10;
    _active = [];
    _weight = 0;
    _xdes = [];
    _dxdes = [];
    _name = "Joint limits";
    SetGains(-0.1,-2.0,0);
}


Vector JointLimitTask::GetSensedValue(Vector q)
{
    return [q[jointi] for jointi in _active]
}

Vector  JointLimitTask::GetJacobian(Vector q)
{
    J = [];
    for( jointi in _active)
    {
        Ji = [0] * _robot.links.size();
        Ji[jointi] = 1;
        J.append(Ji);
        return J;
    }
}
