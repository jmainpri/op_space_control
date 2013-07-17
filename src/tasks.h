#ifndef OPERATIONALSPACETASKS_H
#define OPERATIONALSPACETASKS_H

#include <string>
#include <eigen3/Eigen/Core> // TODO include eigen
#include <eigen3/Eigen/Geometry>
#include <RobotKin/Robot.h>

/*!
A base class for an operational space task. x=f(q)
Subclasses should override getters for sensed x, sensed error
(optional), sensed dx (optional), and appropriate calculation
of Jacobian.
Subclasses inherits setters for xdes, dxdes, gains, priority level,
weight, and task name.
If the task space is non-cartesian, the taskDifference method should
be overridden.
*/
class OperationalSpaceTask
{
public:
    OperationalSpaceTask();

    //! Returns task xdes that has been set
    Eigen::VectorXd GetDesiredValue()
    {
        return _xdes;
    }

    //! User calls this to set task state xdes
    Eigen::VectorXd SetDesiredValue(Eigen::VectorXd xdes)
    {
        _xdes = xdes;
    }

    //! User calls this to set task state xdes
    Eigen::VectorXd SetDesiredVelocity(Eigen::VectorXd dxdes)
    {
        _dxdes = dxdes;
    }

    //! User calls this to set PID gains for feedback control in operational space
    void SetGains(double hP=-1,double hD=-0.1,double hI=-0.1)
    {
        _hP = hP;
        _hD = hD;
        _hI = hI;
    }

    //! User calls this to set priority level. A smaller value means more important
    void SetPriority(int level=1)
    {
        _level = level;
    }

    //! User calls this to set task weight to differentiate from others on the same
    //! priority level. A larger weight means more important.
    void SetWeight(int weight)
    {
        _weight = weight;
    }

    //! Task name can be used to retrieve a task in an OperationalSpaceController instance
    void SetName(std::string name)
    {
        _name = name;
    }

    void ResetITerm(Eigen::VectorXd eI)
    {
        _eI = eI; // Change
    }

    //! Called at beginning of new timestep.
    //! Optionally does something before computing stuff in getCommandVelocity/advance. e.g., compute cached values
    void UpdateState( Eigen::VectorXd q, Eigen::VectorXd dq, double dt ) { }

    //! Get Command Velocity
    Eigen::VectorXd GetCommandVelocity( Eigen::VectorXd q, Eigen::VectorXd dq, double dt);

    //! Updates internal state: accumulates iterm and updates x_last
    void Advance(Eigen::VectorXd q, Eigen::VectorXd dq, double dt);

    //! Gets task x from sensed configuration q
    virtual Eigen::VectorXd GetSensedValue(Eigen::VectorXd dq) = 0;

    //! Gets Jacobian dx/dq(q)
    //! Subclasses MUST override this.
    virtual Eigen::VectorXd GetJacobian(Eigen::VectorXd q) = 0;

    //! Returns x(q)-xdes where - is the task-space differencing operator
    Eigen::VectorXd GetSensedError(Eigen::VectorXd q);

    //! Default: assumes a Cartesian space
    Eigen::VectorXd TaskDifference(Eigen::VectorXd a, Eigen::VectorXd b);

    //! GetSensedVelocity
    //! Gets task velocity from sensed configuration q.
    Eigen::VectorXd GetSensedVelocity(Eigen::VectorXd q, Eigen::VectorXd dq, double dt);

    //! Optionally can be overridden to visualize the task in OpenGL.
    void DrawGL(Eigen::VectorXd q);

protected:
    int _level;
    int _weight;
    std::string _name;
    double _hP;
    double _hD;
    double _hI;
    Eigen::VectorXd _qLast;
    Eigen::VectorXd _xdes;
    Eigen::VectorXd _dxdes;
    Eigen::VectorXd _eI;
};

//! Center of Mass position task subclass
class COMTask : public Task
{
    COMTask( RobotKin& robot, int baseLinkNo = -1 );

    double GetMass();

    //! Returns CoM position
    Eigen::VectorXd GetSensedValue(Eigen::VectorXd q);

    //! Returns axis-weighted CoM Jacobian by averaging
    Eigen::VectorXd GetJacobian(Eigen::VectorXd q);

    void DrawGL(Eigen::VectorXd q);

private:
    RobotKin::Robot& _robot;
    double _mass;
    std::string _name;
    int _baseLinkNo;
};

//! Link position/orientation task subclass.
//! Supports both absolute and relative positioning.
class LinkTask : public Task
{
    LinkTask();
    Eigen::VectorXd GetSensedValue( Eigen::VectorXd q );
    Eigen::VectorXd TaskDifference( Eigen::VectorXd a, Eigen::VectorXd b);
    Eigen::VectorXd GetJacobian( Eigen::VectorXd q );
    Eigen::VectorXd DrawGL(Eigen::VectorXd q);

private:

    int _linkNo;
    int _baseLinkNo;
    RobotKin::Robot _robot;
//    self.hP = -1
//    self.hD = 0
//    self.hI = 0
    Eigen::VectorXd _localPosition;
    std::string _taskType;
    std::string _name = "Link";
};

//! A joint angle task class
class JointTask : public Task
{
    JointTask();

    Eigen::VectorXd GetSensedValue(Eigen::VectorXd q);
    Eigen::VectorXd GetJacobian(Eigen::VectorXd q);

private:
    RobotKin::Robot _robot;
    std::vector<int> _jointIndices;
    std::string _name;
};

class JointLimitTask : public Task
{
    JointLimitTask();

    //! check (q, dq) against joint limits
    void UpdateState(Eigen::VectorXd q, Eigen::VectorXd dq, double dt);

    Eigen::VectorXd GetSensedValue(Eigen::VectorXd q);
    Eigen::VectorXd GetJacobian(Eigen::VectorXd q);

private:
    RobotKin::Robot _robot;
    double _buffersize;
    Eigen::VectorXd _qmin;
    Eigen::VectorXd _qmax;
    Eigen::VectorXd _accelMax;
    double _wscale;
    double _maxw;
    std::vector<int> _active;
    double _weight;
    Eigen::VectorXd _xdes;
    Eigen::VectorXd _dxdes;
    std::string _name;
};

#endif // OPERATIONALSPACETASKS_H
