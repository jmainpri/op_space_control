#include "OperationalSpaceTasks.h"

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
Eigen::VectorXd OperationalSpaceTask::GetCommandVelocity( Eigen::VectorXd q, Eigen::VectorXd dq, double dt)
{
    Eigen::VectorXd eP = GetSensedError(q);

    // P term
    Eigen::VectorXd vP = eP * _hP;
    Eigen::VectorXd vcmd = vP;
    Eigen::VectorXd vcur = GetSensedVelocity( q, dq, dt );

    // D term
    if( vcur.size() != 0 )
    {
        Eigen::VectorXd eD( vcur - _dxdes );
        Eigen::VectorXd vD = eD * _hD;
        vcmd = vcmd + vD;
    }

    // I term
    if( _eI.size() != 0 )
    {
        Eigen::VectorXd vI   = _eI * _hI;
        vcmd = vcmd + vI;
    }
    //print "task",_name,"error P=",eP,"D=",eD,"E=",_eI
    return vcmd;
}

// madd( a, b, c ) = a + c*b

void OperationalSpaceTask::Advance(Eigen::VectorXd q, Eigen::VectorXd dq, double dt)
{
    if ( _weight > 0 )
    {
        Eigen::VectorXd eP = GetSensedError(q);

        // update iterm
        if( _eI.size() != 0 )
        {
            _eI = eP * dt;
        }
        else
        {
            _eI = _eI + eP * dt;
        }
    }

    _qLast = q; // update qLast
}

Eigen::VectorXd OperationalSpaceTask::GetSensedError(Eigen::VectorXd q)
{
    // Returns x(q)-xdes where - is the task-space differencing operator
    return TaskDifference( GetSensedValue(q), _xdes );
}

Eigen::VectorXd  OperationalSpaceTask::TaskDifference( Eigen::VectorXd a, Eigen::VectorXd b )
{
    // Default: assumes a Cartesian space
    return a - b;
}

Eigen::VectorXd  OperationalSpaceTask::GetSensedVelocity(Eigen::VectorXd q, Eigen::VectorXd dq, double dt)
{
    // Gets task velocity from sensed configuration q.
    // Default implementation uses finite differencing.
    // Other implementations may use jacobian.

    // uncomment this to get a jacobian based technique
    // return np.dot(_getJacobian(q),dq)

    if( _qLast.size() > 0 )
    {
        return Eigen::VectorXd();
    }
    else
    {
        Eigen::VectorXd xlast = GetSensedValue(_qLast);
        Eigen::VectorXd xcur = GetSensedValue(q);
        return TaskDifference(xcur,xlast) / dt;
    }
}

void OperationalSpaceTask::DrawGL( Eigen::VectorXd q )
{
    // Optionally can be overridden to visualize the task in OpenGL
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

COMTask::COMTask( RobotKin& robot, int baseLinkNo = -1 ) : Task()
{
    _robot = robot;
    q = _robot.getConfig();
    _mass = _getMass();
    _name = "CoM";
    _baseLinkNo = baseLinkNo;
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

Eigen::VectorXd COMTask::GetSensedValue(Eigen::VectorXd q)
{
    _robot.values(q);
    Eigen::VectorXd com_n(Eigen::VectorXd::Zeros(3));
    for(int i=0;i<robot.nJoints();i++)
    {
        link = _robot.const_joint(i);
        comi = link.getWorldPosition(link.getMass().getCom());
        mi = link.getMass().getMass();
        com_n = vectorops.madd(com_n, comi, mi);
    }
    com = vectorops.div(com_n, _mass);
    if(_baseLinkNo >= 0)
    {
        Tb = _robot.getLink(_baseLinkNo).getTransform();
        Tbinv = se3.inv(Tb);
        com = se3.apply(Tbinv,com);
    }
    return com
}

Eigen::Vector COMTask::GetJacobian(Eigen::VectorXd q)
{
        _robot.setConfig(q)
        numLinks = _robot.numLinks()
        Jcom = [[0.0]*numLinks] * 3
        for i in xrange(numLinks):
            link = _robot.getLink(i)
            mi = link.getMass().getMass()
            comi = link.getMass().getCom()

            Ji = link.getPositionJacobian(comi)
            Ji = [vectorops.mul(Jii, mi) for Jii in Ji]
            for j in xrange(3):
                Jcom[j] = vectorops.add(Jcom[j], Ji[j])
        Jcom = [vectorops.div(Jcomi, _mass) for Jcomi in Jcom]
        #if relative positioning task, subtract out COM jacobian w.r.t. base
        if _baseLinkNo >= 0:
            Tb = _robot.getLink(_baseLinkNo).getTransform()
            Tbinv = se3.inv(Tb)
            pb = se3.apply(Tbinv,_getSensedValue(q))
            Jb = _robot.getLink(_baseLinkNo).getJacobian(pb)
            for i in xrange(len(Jcom)):
                Jcom[i] = vectorops.sub(Jcom[i],Jb[i])
        return Jcom
}


void COMTask::DrawGL(Eigen::VectorXd q)
{
        x = _getSensedValue(q)
        xdes = _xdes
        if _baseLinkNo >= 0:
            Tb = _robot.getLink(_baseLinkNo).getTransform()
            xdes = se3.apply(Tb,_xdes)
            x = se3.apply(Tb,x)
        glPointSize(10)
        glEnable(GL_POINT_SMOOTH)
        glBegin(GL_POINTS)
        glColor3f(0,1,0)	#green
        glVertex3fv(xdes)
        glColor3f(0,1,1)	#cyan
        glVertex3fv(x)
        glEnd()
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

LinkTask::LinkTask()
{
    def __init__(self, robot, linkNo, taskType, baseLinkNo=-1):
        """Supply a robot and link number to control.  taskType can be:
        -'po' for position and orientation
        -'position' for position only
        -'orientation' for orientation only.

        For po and position tasks the localPosition member can be set to
        control a specified point on the link.  The origin is assumed by
        default.

        If baseLinkNo is supplied, the values are treated as relative
        values as calculated with respect to a given base link.
        """
        Task.__init__(self)
        _linkNo = linkNo
        _baseLinkNo = baseLinkNo
        _robot = robot
        _hP, _hD, _hI = -1, 0, 0
        _localPosition=[0.,0., 0.]
        _taskType = taskType
        _name = "Link"

        if _taskType == 'po' or _taskType == 'position' or _taskType == 'orientation':
            pass
        else:
            raise ValueError("Invalid taskType "+_taskType)
}


Eigen::VectorXd LinkTask::GetSensedValue( Eigen::VectorXd q )
    {
        _robot.setConfig(q)
        T = _robot.getLink(_linkNo).getTransform()
        #check if relative transform task, modify T to local transform
        if _baseLinkNo >= 0:
            Tb = _robot.getLink(_baseLinkNo).getTransform()
            Tbinv = se3.inv(Tb)
            T = se3.mul(Tbinv,T)
        if _taskType == 'po':
            x = (T[0],se3.apply(T,_localPosition))
        elif _taskType == 'position':
            x = se3.apply(T,_localPosition)
        elif _taskType == 'orientation':
            x = T[0]
        else:
            raise ValueError("Invalid taskType "+_taskType)
        return x
    }

Eigen::VectorXd LinkTask::TaskDifference( Eigen::VectorXd a, Eigen::VectorXd b)
{
        if _taskType == 'po':
            return se3.error(a,b)
        elif _taskType == 'position':
            return vectorops.sub(a,b)
        elif _taskType == 'orientation':
            return so3.error(a,b)
        else:
            raise ValueError("Invalid taskType "+_taskType)
}

Eigen::VectorXd LinkTask::GetJacobian( Eigen::VectorXd q )
        {
        _robot.setConfig(q)
        J = None
        if _taskType == 'po':
            J = _robot.getLink(_linkNo).getJacobian(_localPosition)
        elif _taskType == 'position':
            J = _robot.getLink(_linkNo).getPositionJacobian(_localPosition)
        elif _taskType == 'orientation':
            J = _robot.getLink(_linkNo).getOrientationJacobian()
        else:
            raise ValueError("Invalid taskType "+_taskType)
        #check if relative transform task, modify Jacobian accordingly
        if _baseLinkNo >= 0:
            T = _robot.getLink(_linkNo).getTransform()
            Tb = _robot.getLink(_baseLinkNo).getTransform()
            Tbinv = se3.inv(Tb)
            pb = se3.apply(Tbinv,se3.apply(T,_localPosition))
            if _taskType == 'po':
                Jb = _robot.getLink(_baseLinkNo).getJacobian(pb)
            elif _taskType == 'position':
                Jb = _robot.getLink(_baseLinkNo).getPositionJacobian(pb)
            elif _taskType == 'orientation':
                Jb = _robot.getLink(_baseLinkNo).getOrientationJacobian()
            #subtract out jacobian w.r.t. baseLink
            for i in xrange(len(J)):
                J[i] = vectorops.sub(J[i],Jb[i])
        return J
}


Eigen::VectorXd LinkTask::DrawGL(Eigen::VectorXd q)
{
        x = _getSensedValue(q)
        xdes = _xdes
        if _baseLinkNo >= 0:
            Tb = _robot.getLink(_baseLinkNo).getTransform()
            if _taskType == "position":
                x = se3.apply(Tb,x)
                xdes = se3.apply(Tb,xdes)
            elif _taskType == "po":
                x = se3.mul(Tb,x)
                xdes = se3.mul(Tb,xdes)
        glPointSize(6)
        glEnable(GL_POINT_SMOOTH)
        glBegin(GL_POINTS)
        if _taskType == "position":
            glColor3f(1,0,0)	#red
            glVertex3fv(xdes)
            glColor3f(1,0.5,0)	#orange
            glVertex3fv(x)
        elif _taskType == "po":
            glColor3f(1,0,0)	#red
            glVertex3fv(xdes[1])
            glColor3f(1,0.5,0)	#orange
            glVertex3fv(x[1])
        glEnd()
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

JointTask::JointTask()
{
    def __init__(self, robot, jointIndices):
        Task.__init__(self)
        _robot = robot
        _jointIndices = jointIndices
        _name = "Joint"
        pass

Eigen::VectorXd JointTask::GetSensedValue(Eigen::VectorXd q)
    {
        return [q[jointi] for jointi in _jointIndices]
    }

Eigen::VectorXd JointTask::GetJacobian(Eigen::VectorXd q)
{
        J = []
        for jointi in _jointIndices:
            Ji = [0] * _robot.numLinks()
            Ji[jointi] = 1
            J.append(Ji)
        return J
}


//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

JointLimitTask::JointLimitTask()
    {
    def __init__(self,robot):
        Task.__init__(self)
        _robot = robot
        _buffersize = 2.0
        _qmin,_qmax = robot.getJointLimits()
        _accelMax = robot.getAccelerationLimits()
        _wscale = 0.1
        _maxw = 10
        _active = []
        _weight = 0
        _xdes = []
        _dxdes = []
        _name = "Joint limits"
        _setGains(-0.1,-2.0,0)
}

void JointLimitTask::UpdateState(Eigen::VectorXd q, Eigen::VectorXd dq, double dt)
    {
        """ check (q, dq) against joint limits
        Activates joint limit constraint, i.e., add a joint task
        to avoid reaching limit, when surpassing a threshold.

        Or, increase weight on this joint task as joint gets closer to its limit.

        """
        _active = []
        _weight = []
        _xdes = []
        _dxdes = []
        buffersize = _buffersize
        wscale = _wscale
        maxw = _maxw
        for i,(j,dj,jmin,jmax,amax) in enumerate(zip(q,dq,_qmin,_qmax,_accelMax)):
            if jmax <= jmin: continue
            jstop = j
            a = amax / buffersize
            w = 0
            ades = 0
            if dj > 0.0:
                t = dj / a
                    #j + t*dj - t^2*a/2
                jstop = j + t*dj - t*t*a*0.5
                if jstop > jmax:
                    #add task to slow down
                        #perfect accel solves for:
                        #j+ dj^2 / 2a  = jmax
                        #dj^2 / 2(jmax-j)   = a
                    if j >= jmax:
                        print "Joint",_robot.getLink(i).getName(),"exceeded max",j,">=",jmax
                        ades = -amax
                        w = maxw
                    else:
                        alim = dj*dj/(jmax-j)*0.5
                        if alim > amax:
                            ades = -amax
                            w = maxw
                        else:
                            ades = -alim
                            w = wscale*(alim-a)/(amax-alim)
                        #print "Joint",_robot.getLink(i).getName(),j,dj,"near upper limit",jmax,", desired accel:",ades," weight",w
            else:
                t = -dj / a
                    #j + t*dj + t^2*a/2
                jstop = j + t*dj + t*t*a*0.5
                if jstop < jmin:
                    #add task to slow down
                        #perfect accel solves for:
                        #j - dj^2 / 2a  = jmin
                        #dj^2 / 2(j-jmin)   = a
                    if j <= jmin:
                        print "Joint",_robot.getLink(i).getName(),"exceeded min",j,"<=",jmin
                        ades = amax
                        w = maxw
                    else:
                        alim = dj*dj/(j-jmin)*0.5
                        if alim > amax:
                            ades = amax
                            w = maxw
                        else:
                            ades = alim
                            w = wscale*(alim-a)/(amax-alim)
                        #print "Joint",_robot.getLink(i).getName(),j,dj,"near lower limit",jmin,", desired accel:",ades," weight",w
            if w > maxw:
                w = maxw
            _active.append(i)
            _xdes.append(max(jmin,min(jmax,j)))
            _dxdes.append(dj+dt*ades)
            _weight.append(w)
        if len(_weight)==0:
            _weight = 0
        return
}


Eigen::VectorXd JointLimitTask::GetSensedValue(Eigen::VectorXd q)
        {
        return [q[jointi] for jointi in _active]
        }

Eigen::VectorXd  JointLimitTask::GetJacobian(Eigen::VectorXd q)
{
        J = []
        for jointi in _active:
            Ji = [0] * _robot.numLinks()
            Ji[jointi] = 1
            J.append(Ji)
        return J
}

/*
class OperationalSpaceController:
    """A two-level velocity-based operational space controller class, mapping from joint space into operational space.
    """

    def __init__(self, robot, dt):
        """robot is a robot model
        dt is simulator time interval
        """
        _taskList = []
        _dqdes = None
        _qdes = None
        _robot = robot
        _dt = dt

    def addTask(self, task):
        """Adds a task into operational space
        """
        _taskList.append(task)

    def getTaskByName(self, taskName):
        """Finds a named task.
        Users need to assure no duplicated task names in the task list manually.
        """
        for taski in _taskList:
            if taski.name == taskName:
                return taski
        return None

    def setDesiredValuesFromConfig(self,qdes,tasks=None):
        """Sets all the tasks' desired values from a given desired
        configuration (e.g., to follow a reference trajectory).

        If the 'tasks' variable is provided, it should be a list of
        tasks for which the desired values should be set.
        """
        if tasks == None:
            tasks = _taskList
        for t in tasks:
            t.setDesiredValue(t.getSensedValue(qdes))

    def setDesiredVelocityFromDifference(self,qdes0,qdes1,dt,tasks=None):
        """Sets all the tasks' desired velocities from a given pair
        of configurations separated by dt (e.g., to follow a reference
        trajectory).

        If the 'tasks' variable is provided, it should be a list of
        tasks for which the desired values should be set.
        """
        if tasks == None:
            tasks = _taskList
        for t in tasks:
            xdes0 = t.getSensedValue(qdes0)
            xdes1 = t.getSensedValue(qdes1)
            dx = vectorops.div(t.taskDifference(xdes1,xdes0),dt)
            t.setDesiredVelocity(dx)

    def printStatus(self,q):
        """Prints a status printout summarizing all tasks' errors."""
        priorities = set()
        names = dict()
        errors = dict()
        totalerrors = dict()
        for t in _taskList:
            if t.weight==0: continue
            priorities.add(t.level)
            s = t.name
            if len(s) > 8:
                s = s[0:8]
            err = t.getSensedError(q)
            names.setdefault(t.level,[]).append(s)
            errors.setdefault(t.level,[]).append("%.3f"%(vectorops.norm(err)),)
            werrsq = vectorops.normSquared(vectorops.mul(err,t.weight))
            totalerrors[t.level] = totalerrors.get(t.level,0.0) + werrsq
        cols = 5
        colwidth = 10
        for p in priorities:
            print "Priority",p,"weighted error^2",totalerrors[p]
            pnames = names[p]
            perrs = errors[p]
            start = 0
            while start < len(pnames):
                last = min(start+cols,len(pnames))
                print "  Name:  ",
                for i in range(start,last):
                    print pnames[i],' '*(colwidth-len(pnames[i])),
                print
                print "  Error: ",
                for i in range(start,last):
                    print perrs[i],' '*(colwidth-len(perrs[i])),
                print
                start=last


    def getStackedJacobian(self, q,dq,priority):
        """Formulates J to calculate dqdes
        """
        J = None
        for taski in _taskList:
            if taski.weight == 0:
                continue
            if taski.level == priority:
                Jtemp = taski.getJacobian(q)
                #scale by weight
                if hasattr(taski.weight,'__iter__'):
                    assert(len(taski.weight)==len(Jtemp))
                    #treat as an elementwise weight
                    for i in xrange(len(Jtemp)):
                        Jtemp[i] = vectorops.mul(Jtemp[i],taski.weight[i])
                else:
                    for i in xrange(len(Jtemp)):
                        Jtemp[i] = vectorops.mul(Jtemp[i],taski.weight)
                if J is None:
                    J = Jtemp
                else:
                    J = np.vstack((J, Jtemp))
        return J

    def getStackedVelocity(self, q, dq, priority):
        """Formulates dx to calculate dqdes
        """
        V = None
        for taski in _taskList:
            if taski.weight == 0:
                continue
            if taski.level == priority:
                #scale by weight
                Vtemp = vectorops.mul(taski.getCommandVelocity(q, dq, _dt),taski.weight)
                if V is None:
                    V = Vtemp
                else:
                    V = np.hstack((V, Vtemp))
        return V

    def checkMax(self, limit):
        """Check dqdes against joint velocity limits.
        """
        limits = _robot.getVelocityLimits()
        m = max(vectorops.div(_dqdes, limits))
        if m > limit:
            for i in xrange(len(_dqdes)):
                _dqdes[i] /= m

    def solve(self, q,dq,dt):
        """Takes sensed q,dq, timestep dt and returns dqdes and qdes
        in joint space.
        """
        for task in _taskList:
            task.updateState(q,dq,dt)
        # priority 1
        J1 = _getStackedJacobian(q,dq,1)
        v1 = _getStackedVelocity(q,dq,1)
        J1inv = np.linalg.pinv(np.array(J1), rcond=1e-3)
        dq1 = np.dot(J1inv, np.array(v1))

        # priority 2
        N = np.eye(len(dq1)) - np.dot(J1inv, np.array(J1))
        Jtask = _getStackedJacobian(q,dq,2)
        if Jtask is not None:
            Vtask = _getStackedVelocity(q,dq,2)
            JtaskN = np.dot(Jtask, N)
            assert np.isfinite(Jtask).all()
            Vtask_m_resid = Vtask - np.dot(Jtask, dq1)
            try:
                JtaskNinv = np.linalg.pinv(JtaskN, rcond=1e-3)
                z = np.dot(JtaskNinv, Vtask_m_resid)
            except np.linalg.LinAlgError:
                #print "SVD failed, trying lstsq"
                z = np.linalg.lstsq(Jtask,Vtask_m_resid,rcond=1e-3)[0]
            dqtask = np.dot(N, z)
        else:
            dqtask = [0.0]*len(dq1)

        #compose the velocities together
        _dqdes = dq1 + dqtask
        _checkMax(1)

        _qdes = vectorops.madd(q, _dqdes, _dt)

        return (_dqdes, _qdes)

    def advance(self,q,dq,dt):
        """Updates all tasks states"""
        for task in _taskList:
            task.advance(q,dq,dt)
*/

