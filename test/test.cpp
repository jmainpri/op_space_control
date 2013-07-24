#include "urdf-parser.h"
#include "robotsim-parser.h"
#include "trajfowlloing.h"

using std::cout;
using std::endl;

void trajectory_following( RobotDynamics3D& robot, Config q, const std::vector<std::string>& linknames )
{
    op_space_control::TrajFollowing test( robot, 0.005 );
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

    op_space_control::UrdfRobotParser robot;
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

    op_space_control::RobotsimParser robot;
    robot.Load("/home/jmainpri/workspace/RobotSim/data/drchubo/DRC/drchubo-v2/drchubo_col.rob");

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
