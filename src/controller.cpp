#include "controller.h"

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
