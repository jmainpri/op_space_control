/*
 * (C) Copyright 2013 WPI-ARC (http://arc.wpi.edu) and others.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser General Public License
 * (LGPL) version 2.1 which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/lgpl-2.1.html
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * Contributors:
 *      Jim Mainprice
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

//! A two-level velocity-based operational space controller class, mapping from joint space into operational space.

// TODO can not currently link directly with RobotSim
// It could work if the library was installed on the system

#include <vector>

class RobotLink3D;
class RobotDynamics3D;

namespace OpSpaceControl
{
typedef std::vector<double> OpVect;
typedef std::vector< std::vector<double> > OpMatrix;
class OperationalSpaceTask;
class OperationalSpaceController
{
public:
    //! robot is a robot model
    //! dt is simulator time interval
    OperationalSpaceController( RobotDynamics3D* robot, std::vector<std::string>& linkNames, double dt );

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
    void SetDesiredValuesFromConfig( const OpVect& qdes );
    void SetDesiredValuesFromConfig( const OpVect& qdes, const std::vector<OperationalSpaceTask*>& tasks );

    //! Sets all the tasks' desired velocities from a given pair
    //! of configurations separated by dt (e.g., to follow a reference trajectory)
    void SetDesiredVelocityFromDifference( const OpVect& qdes0, const OpVect& qdes1, double dt );
    void SetDesiredVelocityFromDifference( const OpVect& qdes0, const OpVect& qdes1, double dt, const std::vector<OperationalSpaceTask*>& tasks);

    void PrintStatus( const OpVect& q);

    //! Formulates J to calculate dqdes
    OpMatrix GetStackedJacobian( const OpVect& q, const OpVect& dq, int priority );

    //! Formulates dx to calculate dqdes
    OpVect GetStackedVelocity( const OpVect& q, const OpVect& dq, int priority );

    //! Check dqdes against joint velocity limits.
    void CheckMax(double limit);

    //! Takes sensed q,dq, timestep dt and returns dqdes and qdes
    //! in joint space
    std::pair<OpVect,OpVect> Solve( const OpVect& q, const OpVect& dq, double dt );

    //! Updates all tasks states
    void Advance( const OpVect& q, const OpVect& dq, double dt );

    //! Mapping to active joints
    virtual OpMatrix JointMappingToActive( const OpMatrix& m ) { return m; }

    //! Mapping to active joints
    virtual OpVect JointMappingToActive( const OpVect& v ) { return v; }

    //! Mapping to full configuration
    virtual OpVect JointMappingToFull( const OpVect& v ) { return v; }

    //! Zeros part of the configuration
    void SetZeros( OpVect& q );

    //! Returns pointer to tasks
    const std::vector<OperationalSpaceTask*>& GetTasks() { return taskList_; }

protected:

    int GetLinkIdByName(const std::string& name);
    const RobotLink3D* GetLinkByName(const std::string& name);

    std::vector<OperationalSpaceTask*> taskList_;
    OpVect dqdes_;
    OpVect qdes_;
    RobotDynamics3D* robot_;
    std::vector<std::string>& linkNames_;
    double dt_;
};

}

#endif // CONTROLLER_H
