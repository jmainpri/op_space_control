#include "urdf-parser.h"
#include "robotsim-parser.h"
#include "trajfowlloing.h"

using std::cout;
using std::endl;

void trajectory_following( RobotDynamics3D& robot, Config q )
{
    op_space_control::TrajFollowing test( robot );
    test.LoadTrajectory();
    test.CreateTasks( test.GetInitConfig() );
    test.Run();
}

void print_link_names(const op_space_control::UrdfRobotParser& robot)
{
    for(int i=0;i<int(robot.links.size());i++)
    {
        cout << robot.LinkName(i) << " : " << robot.linkNames[i] << endl;
    }
}

void test_urdf()
{
    cout << "parse urdf format" << endl;

    op_space_control::UrdfRobotParser robot;
    robot.LoadUrdf("/home/jmainpri/workspace/ros_workspace/src/drchubo/drchubo-v2/robots/drchubo-v2.urdf");

    cout << "Number of links : " << robot.links.size() << endl;
    cout << "robot.q.n : " << robot.q.n << endl;

    // Uncomment to print joint mapping
    print_link_names( robot );
    trajectory_following( robot, robot.q );
}

void test_robotsim()
{
    cout << "parse robotsim format" << endl;

    op_space_control::RobotsimParser robot;
    robot.Load("/home/jmainpri/workspace/RobotSim/data/drchubo/DRC/drchubo-v2/drchubo_col.rob");

    cout << "Number of links : " << robot.links.size() << endl;
    cout << "robot.q.n : " << robot.q.n << endl;

    trajectory_following( robot, robot.q );
}

int main(int argc, char** argv)
{
    //test_urdf();
    test_robotsim();
}
