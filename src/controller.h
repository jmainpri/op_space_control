#ifndef CONTROLLER_H
#define CONTROLLER_H

//! A two-level velocity-based operational space controller class, mapping from joint space into operational space.

#include "robotics/RobotDynamics3D.h"

// TODO can not currently link directly with RobotSim
// It could work if the library was installed on the system

namespace op_space_control
{
class OperationalSpaceTask;
class OperationalSpaceController
{
public:
    //! robot is a robot model
    //! dt is simulator time interval
    OperationalSpaceController( RobotDynamics3D& robot, double dt );

    //! delete all tasks
    ~OperationalSpaceController();

    //! Adds a task into operational space
    void AddTask(  OperationalSpaceTask* task );

    //! Finds a named task.
    //! Users need to assure no duplicated task names
    //! in the task list manually.
    OperationalSpaceTask* GetTaskByName( const std::string &taskName ) const;

    //! Sets all the tasks' desired values from a given desired
    //! configuration (e.g., to follow a reference trajectory)
    //! If the 'tasks' variable is provided, it should be a list of
    //! tasks for which the desired values should be set.
    void SetDesiredValuesFromConfig( const Config& qdes );
    void SetDesiredValuesFromConfig( const Config& qdes, const std::vector<OperationalSpaceTask*>& tasks );

    //! Sets all the tasks' desired velocities from a given pair
    //! of configurations separated by dt (e.g., to follow a reference trajectory)
    void SetDesiredVelocityFromDifference( const Config& qdes0, const Config& qdes1, double dt );
    void SetDesiredVelocityFromDifference( const Config& qdes0, const Config& qdes1, double dt, const std::vector<OperationalSpaceTask*>& tasks);

    void PrintStatus( const Config& q);

    //! Formulates J to calculate dqdes
    Matrix GetStackedJacobian( const Config& q, const Vector& dq, int priority );

    //! Formulates dx to calculate dqdes
    Vector GetStackedVelocity( const Config& q, const Vector& dq, int priority );

    //! Check dqdes against joint velocity limits.
    void CheckMax(double limit);

    //! Takes sensed q,dq, timestep dt and returns dqdes and qdes
    //! in joint space
    std::pair<Vector,Vector> Solve( const Config& q, const Vector& dq, double dt );

    //! Updates all tasks states
    void Advance( const Config& q, const Vector& dq, double dt );

private:
    std::vector<OperationalSpaceTask*> taskList_;
    Vector dqdes_;
    Vector qdes_;
    RobotDynamics3D& robot_;
    double dt_;
};

}

#endif // CONTROLLER_H
