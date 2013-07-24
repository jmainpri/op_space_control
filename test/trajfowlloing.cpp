#include "trajfowlloing.h"
#include "tasks.h"
#include "controller.h"
#include "op_utils.h"
#include "statistics/UniformDistribution.h"
#include <sys/time.h>

using namespace op_space_control;

using std::cout;
using std::endl;

TrajFollowing::TrajFollowing( RobotDynamics3D& robot, double dt ) : robot_( robot ), dt_(dt)
{

}

TrajFollowing::~TrajFollowing()
{
    cout << "delete all " << endl;
    delete opController_;
}

void TrajFollowing::CreateTasks( Config q_init )
{
    opController_ = new DRCHuboOpSpace();
    opController_->SetRobot(&robot_);
    opController_->SetLinkNames(linkNames_);
    opController_->CreateTasks( q_init, dt_ );
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
std::pair<Vector,Vector> TrajFollowing::Trigger( Config q, Vector dq, double dt, double time_cur )
{
    std::pair<OpVect,OpVect> qdes = opController_->Trigger( q_last_, dq_last_, q, dq, dt );
    std::pair<Vector,Vector> out;
    out.first =  qdes.first;
    out.second = qdes.second;
    return out;
}

// Simulate error on the trajectory
Config TrajFollowing::GetSensedConfig(double time)
{
    Config q = traj_.eval( time );
    double noise_std_dev = 0.01;
    Vector q_max( q.n, noise_std_dev );
    Vector q_min( q.n, -noise_std_dev );
    Statistics::BoxProbabilityDistribution dist( q_max, q_min );
    Config q_noise(q.n);
    dist.Sample(q_noise);

    // Generate noise only one upper body joints
    Config q_tmp = q + q_noise;
    //cout << q_noise << endl;
    for(int i=6;i<q_tmp.size();i++)
        q[i] = q_tmp[i];
    return q;
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

    bool start = true;
    Config q;
    Config q_init(robot_.links.size(),0.0);

    while( traj_.length_ > time )
    {
//        cout<< "Time t=" << time << endl;
//        double chrono_start = GetRealTime();

//        q = GetSensedConfig( time );

        if( start )
        {
            q_last_ = q = q_init;
            start =false;
        }

        Vector dq;
        if(q_last_.empty())
            dq = Vector( q.n, 0.0 );
        else
            dq = ( q - q_last_) / dt_;

        // Gets solution in operational space
        std::pair<Vector,Vector> q_out = Trigger( q, dq, dt_, time );

        q_last_ = q;
        dq_last_ = dq;

        q = q_out.second;
        cout << q_out.second << endl;
        // Print status
        //opController_->PrintStatus( q_out.second );
        time += dt_;
    }
}
