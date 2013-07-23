#ifndef TRAJFOLLOING_H
#define TRAJFOLLOING_H

#include "trajectory.h"
#include "urdf-parser.h"
#include "controller.h"

namespace op_space_control
{
typedef std::vector<double> OpVect;
class TrajFollowing
{
public:
    TrajFollowing(Robot& robot);
    ~TrajFollowing();

    void CreateTasks( Config q_init );
    void LoadTrajectory();
    void Run();

private:
    double GetRealTime();
    std::pair<Vector,Vector> Trigger(Config q, Vector dq, double dt, double time_cur);
    Config GetSensedConfig(double time);

    Robot& robot_;
    Trajectory traj_;
    OperationalSpaceController* opController_;
    double dt_;
    Config q_last_;
};
}

#endif // TRAJFOWLLOING_H
