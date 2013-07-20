#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "robotics/RobotDynamics3D.h"

//! A two-level velocity-based operational space controller class, mapping from joint space into operational space.

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
    OperationalSpaceTask* GetTaskByName( std::string taskName );

    //! Sets all the tasks' desired values from a given desired
    //! configuration (e.g., to follow a reference trajectory)
    //! If the 'tasks' variable is provided, it should be a list of
    //! tasks for which the desired values should be set.
    void SetDesiredValuesFromConfig( Config qdes );
    void SetDesiredValuesFromConfig( Config qdes, const std::vector<OperationalSpaceTask*>& tasks );

    //! Sets all the tasks' desired velocities from a given pair
    //! of configurations separated by dt (e.g., to follow a reference trajectory)
    void SetDesiredVelocityFromDifference( Config qdes0, Config qdes1, double dt );
    void SetDesiredVelocityFromDifference( Config qdes0, Config qdes1, double dt, const std::vector<OperationalSpaceTask*>& tasks);

    void PrintStatus(Config q);

    //! Formulates J to calculate dqdes
    Matrix GetStackedJacobian( Config q, Vector dq, int priority );

    //! Formulates dx to calculate dqdes
    Vector GetStackedVelocity( Config q, Vector dq, int priority );

    //! Check dqdes against joint velocity limits.
    void CheckMax(double limit);

    //! Takes sensed q,dq, timestep dt and returns dqdes and qdes
    //! in joint space
    Vector Solve( Config q, Vector dq, double dt );

    //! Updates all tasks states
    void Advance( Config q, Vector dq, double dt );

private:
    std::vector<OperationalSpaceTask*> _taskList;
    Vector _dqdes;
    Vector _qdes;
    RobotDynamics3D& _robot;
    double _dt;
};

}

#endif // CONTROLLER_H
