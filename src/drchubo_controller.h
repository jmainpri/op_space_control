#ifndef DRC_HUBO_TRAJFOLLOING_H
#define DRC_HUBO_TRAJFOLLOING_H

#include <vector>
#include <map>
#include <string>

class RobotDynamics3D;

namespace op_space_control
{
typedef std::vector<double> OpVect;
class Robot;
class OperationalSpaceController;
class DRCHuboOpSpace
{
public:
    DRCHuboOpSpace();
    ~DRCHuboOpSpace();

    OpVect MapConfig( const OpVect& q, bool map_out = false);
    void CreateTasks( const OpVect& q_init, double dt );
    std::pair<OpVect,OpVect> Trigger( const OpVect& q_cur_in, const OpVect& dq_cur_in,
                                      const OpVect& q_des_in, const OpVect& dq_des_in, double dt );
    void SetRobotNbDofs( int nb_dofs ) { nb_dofs_ = nb_dofs; }
    void SetRobot( RobotDynamics3D* robot ) { robot_ = robot; }
    void SetLinkNames( const std::vector<std::string>& linknames ) { linkNames_ = linknames; }

private:
    void InitMaps();
    //Robot* robot_;
    RobotDynamics3D* robot_;
    OperationalSpaceController* opController_;
    std::map<std::string,int> rs_map;
    std::map<std::string,int> urdf_map;
    std::vector<std::string> linkNames_;
    int nb_dofs_;
    bool use_mapping_;
    double dt_;
};
}

#endif // TRAJFOWLLOING_H
