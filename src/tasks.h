#ifndef OPERATIONALSPACETASKS_H
#define OPERATIONALSPACETASKS_H

#include "robotics/RobotDynamics3D.h"

/*!
A base class for an operational space task. x=f(q)
Subclasses should override getters for sensed x, sensed error
(optional), sensed dx (optional), and appropriate calculation of Jacobian.

Subclasses inherits setters for xdes, dxdes, gains, priority level,
weight, and task name.

If the task space is non-cartesian, the taskDifference method should be overridden.
*/
namespace op_space_control
{
class OperationalSpaceTask
{
public:
    OperationalSpaceTask(std::vector<std::string>& linkNames);

    //! Returns task xdes that has been set
    Vector GetDesiredValue()
    {
        return xdes_;
    }

    //! User calls this to set task state xdes
    void SetDesiredValue( const Vector& xdes )
    {
        xdes_ = xdes;
    }

    //! User calls this to set task state xdes
    void SetDesiredVelocity( const Vector& dxdes )
    {
        dxdes_ = dxdes;
    }

    //! User calls this to set PID gains for feedback control in operational space
    void SetGains( double hP=-1, double hD=-0.1, double hI=-0.1 )
    {
        hP_ = hP;
        hD_ = hD;
        hI_ = hI;
    }

    //! User calls this to get priority level, a smaller value means more important
    double GetPriority()
    {
        return level_;
    }

    //! User calls this to set priority level, a smaller value means more important
    void SetPriority( int level=1 )
    {
        level_ = level;
    }

    //! User calls this to get task weight to differentiate from others on the same
    //! priority level. A larger weight means more important.
    Vector GetWeight()
    {
        return weight_;
    }

    //! User calls this to set task weight to differentiate from others on the same
    //! priority level. A larger weight means more important.
    void SetWeight( Vector weight )
    {
        weight_ = weight;
    }

    //! Task name can be used to retrieve a task in an OperationalSpaceController instance
    std::string GetName()
    {
        return name_;
    }

    //! Task name can be used to retrieve a task in an OperationalSpaceController instance
    void SetName( std::string name )
    {
        name_ = name;
    }

    void ResetITerm( Vector eI )
    {
        eI_ = eI; // Change
    }

    //! Called at beginning of new timestep.
    //! Optionally does something before computing stuff in getCommandVelocity/advance. e.g., compute cached values
    virtual void UpdateState( const Config& q, const Vector&  dq, double dt ) { }

    //! Get Command Velocity
    virtual Vector GetCommandVelocity( const Config& q, const Vector&  dq, double dt );

    //! Updates internal state: accumulates iterm and updates x_last
    virtual void Advance( const Config& q, const Vector&  dq, double dt );

    //! Gets task x from sensed configuration q
    virtual Vector GetSensedValue( const Vector& dq ) = 0;

    //! Gets Jacobian dx/dq(q)
    virtual Matrix GetJacobian( const Config& q ) = 0;

    //! Returns x(q)-xdes where - is the task-space differencing operator
    virtual Vector GetSensedError( const Config& q );

    //! Default: assumes a Cartesian space
    virtual Vector TaskDifference( const Vector& a, const Vector& b );

    //! Gets task velocity from sensed configuration q.
    virtual Vector GetSensedVelocity( const Config& q, const Vector&  dq, double dt );

    //! Optionally can be overridden to visualize the task in OpenGL.
    virtual void DrawGL(const Vector& q) { }

protected:

    int level_;
    Vector weight_;
    std::string name_;
    std::vector<std::string>& linkNames_;
    double hP_;
    double hD_;
    double hI_;
    Config qLast_;
    Vector xdes_;
    Vector dxdes_;
    Vector eI_;
};

//! Center of Mass position task subclass
class COMTask : public OperationalSpaceTask
{
public:
    COMTask( RobotDynamics3D& robot, std::vector<std::string>& linkNames, int baseLinkNo = -1 );

    //! Set base link id
    void SetBaseLinkNo(int baseLinkNo) { baseLinkNo_ = baseLinkNo; }

    double GetMass();

    //! Returns CoM position
    Vector GetSensedValue( const Config& q );

    //! Returns axis-weighted CoM Jacobian by averaging
    Matrix GetJacobian( const Config& q );

    //! Draws useful sanity check of the task
    void DrawGL( const Config& q );

private:
    RobotDynamics3D& robot_;
    double mass_;
    int baseLinkNo_;
};

//! Link position/orientation task subclass.
//! Supports both absolute and relative positioning.
//! taskType should be :
class LinkTask : public OperationalSpaceTask
{
public:
    LinkTask( RobotDynamics3D& robot, std::vector<std::string>& linkNames, int linkNo, std::string taskType, int baseLinkNo = -1  );
    void SetBaseLinkNo(int baseLinkNo) { baseLinkNo_ = baseLinkNo; }
    void SetLocalPosition(Vector3 p) { localPosition_ = p; }
    Vector GetSensedValue( const Config& q );
    Vector TaskDifference( const Vector& a, const Vector& b);
    Matrix GetJacobian( const Config& q );
    void DrawGL( const Vector& q );

private:
    void SetTaskType( const std::string &taskType );
    enum TaskType { position, orientation, po } taskType_;
    int linkNo_;
    int baseLinkNo_;
    RobotDynamics3D& robot_;
    Vector3 localPosition_;
};

//! A joint angle task class
class JointTask : public OperationalSpaceTask
{
public:
    JointTask( RobotDynamics3D& robot, std::vector<std::string>& linkNames, const std::vector<int>& jointIndices );

    Vector GetSensedValue( const Config& q );
    Matrix GetJacobian( const Config& q );

private:
    RobotDynamics3D& robot_;
    std::vector<int> jointIndices_;
};

//! Check (q, dq) against joint limits
//! Activates joint limit constraint, i.e., add a joint task
//! to avoid reaching limit, when surpassing a threshold.
//! Or, increase weight on this joint task as joint gets closer to its limit
class JointLimitTask : public OperationalSpaceTask
{
public:
    JointLimitTask(  RobotDynamics3D& robot, std::vector<std::string>& linkNames );

    void UpdateState( const Config& q, const Vector& dq, double dt );

    Vector GetSensedValue( const Config& q );
    Matrix GetJacobian( const Config& q );

private:
    RobotDynamics3D& robot_;
    double buffersize_;
    Vector qMin_;
    Vector qMax_;
    Vector aMax_;
    double wscale_;
    double maxw_;
    std::vector<int> active_;
    std::vector<double> weight_;
    Vector xdes_;
    Vector dxdes_;
};
}

#endif // OPERATIONALSPACETASKS_H
