#include "urdf-parser.h"

#include "tasks.h"
#include "controller.h"
#include "op_utils.h"

using namespace op_space_control;
using std::cout;
using std::endl;

void create_tasks( Robot& robot, Config q_init )
{
//    self.qtraj = trajectory.RobotTrajectory(robot)
//    self.qtraj.read(trajfn)
//    assert (self.qtraj.times[0] == 0.0),"Trajectory start time is not 0"
//    qinit = self.qtraj.milestones[0]

    // priority 1
    // right foot task
    cout << "Create right foot task" << endl;
    LinkTask* RFTask = new LinkTask( robot, 56, "po" );
    RFTask->SetPriority(1);
    RFTask->SetName("RF");
    RFTask->SetDesiredValue( GetPushedFrame( robot.links[56].T_World ) );
    RFTask->SetDesiredVelocity( Vector(6,0.0) );
    RFTask->SetGains(0,0,0);

    // the same with left foot
    cout << "Create left foot task" << endl;
    LinkTask* LFTask = new LinkTask( robot, 62, "po");
    LFTask->SetPriority(1);
    LFTask->SetName("LF");
    LFTask->SetDesiredValue( GetPushedFrame( robot.links[62].T_World ) );
    LFTask->SetDesiredVelocity( Vector(6,0.0) );
    LFTask->SetGains(0,0,0);

    // priority 2
    // CoM task
    cout << "Create CoM foot task" << endl;
    COMTask* comTask = new COMTask( robot );
    // maintain position relative to foot
    comTask->SetBaseLinkNo( 56 );
    comTask->SetPriority(2);
    comTask->SetDesiredValue( comTask->GetSensedValue(q_init) );
    comTask->SetDesiredVelocity( Vector(3,0.0) );
    comTask->SetGains(-10.0,-0.5,-0.5);
    Vector weights(3);
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 0.1;
    comTask->SetWeight(weights);

    // link-13 hand task
    cout << "Create hand task" << endl;
    LinkTask* handTask = new LinkTask( robot, 13, "position" );
    // maintain position relative to foot
    handTask->SetBaseLinkNo( 56 );
    // point around middle of palm
    handTask->SetLocalPosition( Vector3(0.0,0.0,-0.07) );
    handTask->SetName("handTask");
    handTask->SetDesiredValue( handTask->GetSensedValue(q_init) );
    handTask->SetDesiredVelocity( Vector(3,0.0) );
    handTask->SetGains(-10,-0.1,-1);
    handTask->SetPriority(2);
    handTask->SetWeight(1);

    // joint task
    cout << "Create joint task" << endl;
    std::vector<JointTask*> jointTasks;

    for( int i=0;i<q_init.n;i++)
        jointTasks.push_back(new JointTask(robot,GetStdVector(i)));

    for( int i=0;i<q_init.size();i++)
    {
        jointTasks[i]->SetName( robot.linkNames[i] );
        jointTasks[i]->SetDesiredValue(Vector(1, q_init[i]));
        jointTasks[i]->SetDesiredVelocity(Vector(1,0.0));
        jointTasks[i]->SetGains(-1, -0.0, -0.1);
        jointTasks[i]->SetWeight(Vector(1,0.001));
        //turn off tasks for the base translation and rotation
        if( i < 6 )
            jointTasks[i]->SetWeight(Vector(1,0.0));

        jointTasks[i]->SetPriority(2);
    }

    // Setup operational space controller
    cout << "Create operational space controller" << endl;
    double dt = 0.02;
    OperationalSpaceController* opController = new OperationalSpaceController(robot, dt);
    opController->AddTask(RFTask);
    opController->AddTask(LFTask);
    opController->AddTask(comTask);
    opController->AddTask(handTask);

    for( int i=0;i<q_init.size();i++)
        opController->AddTask( jointTasks[i] );

    cout << "delete all " << endl;
    delete opController;
}

int main(int argc, char** argv)
{
    Robot robot;
    robot.load_urdf("/home/jmainpri/workspace/ros_workspace/src/drchubo/drchubo-v2/robots/drchubo-v2.urdf");
    //robot.load_urdf("/home/jmainpri/workspace/ros_workspace/src/drchubo/drchubo-v1/robots/drchubo-v1.urdf");

    cout << "Number of links : " << robot.links.size() << endl;
    cout << "robot.q.n : " << robot.q.n << endl;

    create_tasks( robot, robot.q );
}
