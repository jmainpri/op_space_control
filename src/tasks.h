#ifndef OPERATIONALSPACETASKS_H
#define OPERATIONALSPACETASKS_H

#include "robotics/RobotDynamics3D.h"

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
namespace op_space_control
{
class OperationalSpaceTask
{
public:
    OperationalSpaceTask();

    //! Returns task xdes that has been set
    Vector GetDesiredValue()
    {
        return _xdes;
    }

    //! User calls this to set task state xdes
    void SetDesiredValue( Vector xdes )
    {
        _xdes = xdes;
    }

    //! User calls this to set task state xdes
    void SetDesiredVelocity( Vector dxdes )
    {
        _dxdes = dxdes;
    }

    //! User calls this to set PID gains for feedback control in operational space
    void SetGains( double hP=-1, double hD=-0.1, double hI=-0.1 )
    {
        _hP = hP;
        _hD = hD;
        _hI = hI;
    }

    //! User calls this to get priority level. A smaller value means more important
    double GetPriority() { return _level; }

    //! User calls this to set priority level. A smaller value means more important
    void SetPriority( int level=1 )
    {
        _level = level;
    }

    //! User calls this to get task weight to differentiate from others on the same
    //! priority level. A larger weight means more important.
    Vector GetWeight() { return _weight; }

    //! User calls this to set task weight to differentiate from others on the same
    //! priority level. A larger weight means more important.
    void SetWeight( Vector weight )
    {
        _weight = weight;
    }

    //! Task name can be used to retrieve a task in an OperationalSpaceController instance
    std::string GetName() { return _name; }

    //! Task name can be used to retrieve a task in an OperationalSpaceController instance
    void SetName( std::string name )
    {
        _name = name;
    }

    void ResetITerm( Vector eI )
    {
        _eI = eI; // Change
    }

    //! Called at beginning of new timestep.
    //! Optionally does something before computing stuff in getCommandVelocity/advance. e.g., compute cached values
    void UpdateState( Config q, Vector dq, double dt ) { }

    //! Get Command Velocity
    Vector GetCommandVelocity( Config q, Vector dq, double dt );

    //! Updates internal state: accumulates iterm and updates x_last
    void Advance( Config q, Vector dq, double dt );

    //! Gets task x from sensed configuration q
    virtual Vector GetSensedValue( Vector dq ) = 0;

    //! Gets Jacobian dx/dq(q)
    //! Subclasses MUST override this.
    virtual Matrix GetJacobian( Config q ) = 0;

    //! Returns x(q)-xdes where - is the task-space differencing operator
    Vector GetSensedError( Config q );

    //! Default: assumes a Cartesian space
    Vector TaskDifference( Vector a, Vector b );

    //! GetSensedVelocity
    //! Gets task velocity from sensed configuration q.
    Vector GetSensedVelocity( Vector q, Vector dq, double dt );

    //! Optionally can be overridden to visualize the task in OpenGL.
    void DrawGL(Vector q);

protected:
    int _level;
    Vector _weight;
    std::string _name;
    double _hP;
    double _hD;
    double _hI;
    Config _qLast;
    Vector _xdes;
    Vector _dxdes;
    Vector _eI;
};

//! Center of Mass position task subclass
class COMTask : public OperationalSpaceTask
{
public:
    COMTask( RobotDynamics3D& robot, int baseLinkNo = -1 );

    //! Set base link id
    void SetBaseLinkNo(int baseLinkNo) { _baseLinkNo = baseLinkNo; }

    double GetMass();

    //! Returns CoM position
    Vector GetSensedValue( Config q );

    //! Returns axis-weighted CoM Jacobian by averaging
    Matrix GetJacobian( Config q );

    //! Draws useful sanity check of the task
    void DrawGL( Config q );

private:
    RobotDynamics3D& _robot;
    double _mass;
    std::string _name;
    int _baseLinkNo;
};

//! Link position/orientation task subclass.
//! Supports both absolute and relative positioning.
class LinkTask : public OperationalSpaceTask
{
public:
    LinkTask( RobotDynamics3D& robot, int linkNo, std::string taskType, int baseLinkNo = -1  );
    void SetBaseLinkNo(int baseLinkNo) { _baseLinkNo = baseLinkNo; }
    void SetLocalPosition(Vector3 p) { _localPosition = p; }
    Vector GetSensedValue( Config q );
    Vector TaskDifference( Vector a, Vector b);
    Matrix GetJacobian( Config q );
    Vector DrawGL( Vector q );

private:

    int _linkNo;
    int _baseLinkNo;
    RobotDynamics3D& _robot;
    //    self.hP = -1
    //    self.hD = 0
    //    self.hI = 0
    Vector3 _localPosition;
    std::string _taskType;
    std::string _name;
};

//! A joint angle task class
class JointTask : public OperationalSpaceTask
{
public:
    JointTask( RobotDynamics3D& robot, const std::vector<int>& jointIndices );

    Vector GetSensedValue( Config q );
    Matrix GetJacobian( Config q );

private:
    RobotDynamics3D& _robot;
    std::vector<int> _jointIndices;
    std::string _name;
};

//! Check (q, dq) against joint limits
//! Activates joint limit constraint, i.e., add a joint task
//! to avoid reaching limit, when surpassing a threshold.
//! Or, increase weight on this joint task as joint gets closer to its limit
class JointLimitTask : public OperationalSpaceTask
{
public:
    JointLimitTask(  RobotDynamics3D& robot );

    void UpdateState( Config q, Vector dq, double dt );

    Vector GetSensedValue( Config q );
    Matrix GetJacobian( Config q );

private:
    RobotDynamics3D& _robot;
    double _buffersize;
    Vector _qMin;
    Vector _qMax;
    Vector _aMax;
    double _wscale;
    double _maxw;
    std::vector<int> _active;
    std::vector<double> _weight;
    Vector _xdes;
    Vector _dxdes;
    std::string _name;
};
}

#endif // OPERATIONALSPACETASKS_H
