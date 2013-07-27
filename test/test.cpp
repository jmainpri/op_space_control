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

#include "urdf_parser.h"
#include "robotsim_parser.h"
#include "traj_fowlloing.h"

using std::cout;
using std::endl;

void trajectory_following( RobotDynamics3D& robot, Config q, const std::vector<std::string>& linknames )
{
    OpSpaceControl::TrajFollowing test( robot, 0.005 );
    test.LoadTrajectory();
    test.SetLinkNames( linknames );
    test.CreateTasks( test.GetInitConfig() );
    test.Run();
}

void print_link_names(const std::vector<std::string>& linknames)
{
    for(int i=0;i<int(linknames.size());i++)
    {
        cout << i  << " : " << linknames[i] << endl;
    }
}

void test_urdf()
{
    cout << "parse urdf format" << endl;

    OpSpaceControl::UrdfRobotParser robot;
    robot.LoadUrdf("/home/jmainpri/workspace/ros_workspace/src/drchubo/drchubo-v2/robots/drchubo-v2.urdf");

    cout << "Number of links : " << robot.links.size() << endl;
    cout << "robot.q.n : " << robot.q.n << endl;

    std::vector<std::string> linknames = robot.GetLinkNames();
    // Uncomment to print joint mapping
    // print_link_names( linknames );
    trajectory_following( robot, robot.q, linknames );
}

void test_robotsim()
{
    cout << "parse robotsim format" << endl;

    OpSpaceControl::RobotsimParser robot;
    robot.Load("/home/jmainpri/workspace/RobotSim/data/drchubo/DRC/drchubo-v2/drchubo_v2.rob");

    cout << "Number of links : " << robot.links.size() << endl;
    cout << "robot.q.n : " << robot.q.n << endl;

    std::vector<std::string> linknames = robot.GetLinkNames();
    // Uncomment to print joint mapping
    // print_link_names( linknames );
    trajectory_following( robot, robot.q, linknames );
}

int main(int argc, char** argv)
{
    //test_urdf();
    test_robotsim();
}
