#include "urdf-parser.h"
#include "trajfowlloing.h"

using namespace op_space_control;
using std::cout;
using std::endl;

void trajectory_following( Robot& robot, Config q )
{
    TrajFollowing test(robot);
    test.CreateTasks(q);
    test.LoadTrajectory();
    test.Run();
}

void print_link_names(const Robot& robot)
{
    for(int i=0;i<int(robot.links.size());i++)
    {
        cout << robot.LinkName(i) << " : " << robot.linkNames[i] << endl;
    }
}

int main(int argc, char** argv)
{
    Robot robot;
    robot.load_urdf("/home/jmainpri/workspace/ros_workspace/src/drchubo/drchubo-v2/robots/drchubo-v2.urdf");

    cout << "Number of links : " << robot.links.size() << endl;
    cout << "robot.q.n : " << robot.q.n << endl;

    // Uncomment to print joint mapping
    // print_link_names(robot);
    trajectory_following( robot, robot.q );
}
