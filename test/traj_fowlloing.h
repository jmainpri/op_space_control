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

#ifndef TRAJFOLLOING_H
#define TRAJFOLLOING_H

#include "trajectory.h"
#include "robotics/RobotDynamics3D.h"
#include "drchubo_controller.h"

namespace OpSpaceControl
{
typedef std::vector<double> OpVect;
class TrajFollowing
{
public:
    TrajFollowing(RobotDynamics3D& robot, double dt);
    ~TrajFollowing();

    void LoadTrajectory();
    void CreateTasks( Config q_init );
    void Run();
    Config GetInitConfig() { return traj_.eval(0); }
    void SetLinkNames( const std::vector<std::string>& linknames ) { linkNames_ = linknames; }

private:
    double GetRealTime();
    std::pair<Vector,Vector> Trigger(Config q, Vector dq, double dt, double time_cur);
    Config GetSensedConfig(double time);

    std::vector<std::string> linkNames_;
    RobotDynamics3D& robot_;
    Trajectory traj_;
    DRCHuboOpSpace* opController_;
    double dt_;
    Config q_last_;
    Config dq_last_;
};
}

#endif // TRAJFOWLLOING_H
