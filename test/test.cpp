#include "urdf-parser.h"

int main(int argc, char** argv)
{
    op_space_control::Robot robot;
    robot.load_urdf("/home/jmainpri/workspace/ros_workspace/src/drchubo/drchubo-v2/robots/drchubo-v2.urdf");
    //robot.load_urdf("/home/jmainpri/workspace/ros_workspace/src/drchubo/drchubo-v1/robots/drchubo-v1.urdf");
}
