#ifndef TRAJFOLLOING_H
#define TRAJFOLLOING_H

#include "trajectory.h"
#include "robotics/RobotDynamics3D.h"
#include "drchubo_controller.h"

namespace op_space_control
{
typedef std::vector<double> OpVect;
class TrajFollowing
{
public:
    TrajFollowing(RobotDynamics3D& robot);
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
};
}

#endif // TRAJFOWLLOING_H
