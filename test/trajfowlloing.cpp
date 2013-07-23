#include "trajfowlloing.h"
#include "tasks.h"
#include "controller.h"
#include "op_utils.h"
#include "statistics/UniformDistribution.h"
#include <sys/time.h>

using namespace op_space_control;

using std::cout;
using std::endl;

TrajFollowing::TrajFollowing(Robot& robot) : robot_(robot)
{
    dt_ = 0.005;
}

TrajFollowing::~TrajFollowing()
{
    cout << "delete all " << endl;
    delete opController_;
}

void TrajFollowing::CreateTasks(Config q_init)
{
    int right_foot_id = 59; // RAR, robotsim -> 62
    int left_foot_id = 52; // LAR, robotsim -> 56
    int hand_id = 11; // LWP, robotsim -> 13

    // priority 1
    // right foot task
    cout << "Create left foot task" << endl;
    LinkTask* RFTask = new LinkTask( robot_, left_foot_id, "po" ); // LAR
    RFTask->SetPriority(1);
    RFTask->SetName("LF");
    RFTask->SetDesiredValue( GetPushedFrame( robot_.links[left_foot_id].T_World ) );
    RFTask->SetDesiredVelocity( Vector(6,0.0) );
    RFTask->SetGains(0,0,0);

    // the same with left foot
    cout << "Create right foot task" << endl;
    LinkTask* LFTask = new LinkTask( robot_, right_foot_id, "po"); // RAR
    LFTask->SetPriority(1);
    LFTask->SetName("RF");
    LFTask->SetDesiredValue( GetPushedFrame( robot_.links[right_foot_id].T_World ) );
    LFTask->SetDesiredVelocity( Vector(6,0.0) );
    LFTask->SetGains(0,0,0);

    // priority 2
    // CoM task
    cout << "Create CoM foot task" << endl;
    COMTask* comTask = new COMTask( robot_ );
    // maintain position relative to foot
    comTask->SetBaseLinkNo( left_foot_id ); // LAR
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
    LinkTask* handTask = new LinkTask( robot_, hand_id, "position" ); // LWP
    // maintain position relative to foot
    handTask->SetBaseLinkNo( left_foot_id ); // LAR
    // point around middle of palm
    handTask->SetLocalPosition( Vector3(0.0,0.0,-0.07) );
    handTask->SetName("handTask");
    handTask->SetDesiredValue( handTask->GetSensedValue(q_init) );
    handTask->SetDesiredVelocity( Vector(3,0.0) );
    handTask->SetGains(-10,-0.1,-1);
    handTask->SetPriority(2);
    handTask->SetWeight(Vector(1,1.0));

    // joint task
    cout << "Create joint task" << endl;
    std::vector<JointTask*> jointTasks;

    for( int i=0;i<q_init.n;i++)
        jointTasks.push_back(new JointTask(robot_,GetStdVector(i)));

    for( int i=0;i<q_init.size();i++)
    {
        jointTasks[i]->SetName( robot_.linkNames[i] );
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
    opController_ = new OperationalSpaceController(robot_, dt);
    opController_->AddTask(RFTask);
    opController_->AddTask(LFTask);
    opController_->AddTask(comTask);
    opController_->AddTask(handTask);

    for( int i=0;i<q_init.size();i++)
        opController_->AddTask( jointTasks[i] );
}

void TrajFollowing::LoadTrajectory()
{
    if( traj_.parse_robotsim("../test/robot_commands.log") )
    {
        cout << "trajectory loaded" << endl;
        cout << " num config : " << traj_.milestones_.size() << endl;
        cout << " dofs : " << traj_.milestones_[0].second.size() << endl;
        cout << " length : " << traj_.length_ << endl;
    }
    else{
        cout << "error loading trajectory" << endl;
    }
}

// Triggers Operational Space Controller to compute (qdes, dqdes),
// and update tasks states """
std::pair<Vector,Vector> TrajFollowing::Trigger(Config q, Vector dq, double dt, double time_cur)
{
    // Updates tasks desired value following trajectory
    Config qt_tmp = traj_.eval(time_cur);
    Vector qp_tmp = traj_.eval(time_cur-dt_);
    opController_->SetDesiredValuesFromConfig( qt_tmp);
    opController_->SetDesiredVelocityFromDifference( qp_tmp, qt_tmp, dt_ );

    // Solves the stack of task
    std::pair<Vector,Vector> q_out = opController_->Solve( q, dq, dt );
    opController_->Advance(q, dq, dt);
    return q_out;
}

// Simulate error on the trajectory
Config TrajFollowing::GetSensedConfig(double time)
{
    Config q = traj_.eval( time );
    Vector q_max( q.n, 0.01 );
    Vector q_min( q.n, -0.01 );
    Statistics::BoxProbabilityDistribution dist(q_max,q_min);
    Config q_noise;
    dist.Sample(q_noise);

    // Generate noise only one upper body joints
    Config q_tmp = q + q_noise;
    for(int i=0;i<6;i++)
        q_tmp[i] = q[i];
    return q_tmp;
}

double TrajFollowing::GetRealTime()
{
    timeval tim;
    gettimeofday(&tim, NULL);
    return tim.tv_sec+(tim.tv_usec/1000000.0);
}

void TrajFollowing::Run()
{
    cout << "Start trajectory following" << endl;
    double time = 0.0;

    while( traj_.length_ > time )
    {
//        cout<< "Time t=" << time << endl;
//        double chrono_start = GetRealTime();

        Config q = GetSensedConfig( time );

        Vector dq;
        if(q_last_.empty())
            dq = Vector(q.n,0.0);
        else
            dq = ( q - q_last_) / dt_;

        q_last_ = q;

        // Gets solution in operational space
        std::pair<Vector,Vector> q_out = Trigger( q, dq, dt_, time );
//        cout << "q_out.first : " << q_out.first << endl;
//        cout << "q_out.second : " << q_out.second << endl;
//        cout << GetRealTime() - chrono_start << " sec" << endl;

        // Print status
        //opController_->PrintStatus(q);
        time += dt_;
    }
}
