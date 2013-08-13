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

#ifndef DRC_HUBO_TRAJFOLLOING_H
#define DRC_HUBO_TRAJFOLLOING_H

#include <vector>
#include <map>
#include <string>

#include "controller.h"

namespace OpSpaceControl
{
class Robot;

//! Operational space for DRCHubo
//! This class is supposdly instanciated in the robot controler or simulator
//! it holds the task definitions and priorities
class DRCHuboOpSpace
{
    //! Controller instance for DRCHubo
    //! it implements the mapping from active joints to fully specified and vicesa
    class DRCHuboController : public OperationalSpaceController
    {
    public:
        DRCHuboController(  RobotDynamics3D* robot, std::vector<std::string>& linkNames, double dt );
        OpMatrix JointMappingToActive( const OpMatrix& m );
        OpVect JointMappingToActive( const OpVect& v );
        OpVect JointMappingToFull( const OpVect& v );
    private:
        std::vector<int> active_dofs_;
        int nb_links_;
    };

public:
    DRCHuboOpSpace();
    ~DRCHuboOpSpace();

    //! maps a configuration from the internal representation to the robot's
    OpVect JointMappingToRobot( const OpVect& q, bool map_out = false);

    //! specifies active tasks, pid gains, priorities and weights
    void CreateTasks( const OpVect& q_init, double dt );

    //! main function to be called by the robot controller
    //! q_cur is the current configuration
    //! q_des is the desired configuration
    std::pair<OpVect,OpVect> Trigger( const OpVect& q_cur_in, const OpVect& dq_cur_in,
                                      const OpVect& q_des_in, const OpVect& dq_des_in, double dt );

    void SetRobotNbDofs( int nb_dofs ) { nb_dofs_ = nb_dofs; }
    void SetRobot( RobotDynamics3D* robot ) { robot_ = robot; }
    void SetLinkNames( const std::vector<std::string>& linknames ) { linkNames_ = linknames; }

    //! draws task space
    void Draw();

    //! returns machine real time
    double GetRealTime();

private:
    //! initializes the joint mapping between the robot and the kinematic libray
    void InitMaps();
    RobotDynamics3D* robot_;
    OperationalSpaceController* opController_;
    std::map<std::string,int> rs_map;
    std::map<std::string,int> urdf_map;
    std::vector<std::string> linkNames_;
    int nb_dofs_;
    bool use_mapping_;
    double dt_;
    OpVect q_last_;
    OpVect q_des_;
};
}

#endif // TRAJFOWLLOING_H
