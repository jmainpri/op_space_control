#include "drchubo_controller.h"
#include "tasks.h"
#include "controller.h"
#include "op_utils.h"
#include "statistics/UniformDistribution.h"
#include <sys/time.h>
#include "urdf-parser.h"

using namespace op_space_control;

using std::cout;
using std::endl;

DRCHuboOpSpace::DRCHuboOpSpace()
{
//    cout << "Parse URDF Model" << endl;
//    robot_ = new Robot();
//    robot_->load_urdf("/home/jmainpri/workspace/ros_workspace/src/drchubo/drchubo-v2/robots/drchubo-v2.urdf");
//    InitMaps();
    nb_dofs_ = 0;
    use_mapping_ = false;
    dt_ = 0.005;
}

DRCHuboOpSpace::~DRCHuboOpSpace()
{
    cout << "delete all " << endl;
    delete opController_;
}

OpVect DRCHuboOpSpace::MapConfig( const OpVect& q, bool map_out ) // TODO use hardoded to be faster
{
    std::map<std::string,int>& m_in = rs_map; // Changes on the real robot
    std::map<std::string,int>& m_out = urdf_map;

    OpVect q_out(robot_->q.n);

    if( map_out ) {
        std::map<std::string,int>& m_tmp = m_in;
        m_in = m_out;
        m_out = m_tmp;
        if( nb_dofs_ == 0 ) {
            cout << "Error, the number of robot dofs has not been set" << endl;
            return OpVect(0);
        }
        q_out.resize( nb_dofs_ );
    }

    size_t size_i = m_in.size();
    size_t size_o = m_out.size();

    for( std::map<std::string,int>::iterator it_map=m_in.begin(); it_map!=m_in.end(); it_map++ )
    {
        q_out[ m_out[it_map->first] ] = q[ it_map->second ];
    }

    if( size_i !=  m_in.size() || size_o != m_out.size() ){
        cout << "Error in mapping!!!!" << endl;
    }
    return q_out;
}

void DRCHuboOpSpace::CreateTasks(const OpVect& q_init)
{
    int right_foot_id = 59; //      RAR, hubo+ robotsim -> 62,   drc_hubo urdf -> 59,   drc_hubo robotsim -> 59
    int left_foot_id = 52; //       LAR, hubo+ robotsim -> 56,   drc_hubo urdf -> 52,   drc_hubo robotsim -> 52
    int hand_id = 11; //            LWP, hubo+ robotsim -> 13,   drc_hubo urdf -> 11,   drc_hubo robotsim -> 11

    // priority 1
    // right foot task
    cout << "Create right foot task" << endl;
    LinkTask* RFTask = new LinkTask( (*robot_), right_foot_id, "po" ); // RAR
    RFTask->SetPriority(1);
    RFTask->SetName("LF");
    RFTask->SetDesiredValue( GetPushedFrame( (*robot_).links[right_foot_id].T_World ) );
    RFTask->SetDesiredVelocity( Vector(6,0.0) );
    RFTask->SetGains(0,0,0);

    // the same with left foot
    cout << "Create left foot task" << endl;
    LinkTask* LFTask = new LinkTask( (*robot_), left_foot_id, "po"); // LAR
    LFTask->SetPriority(1);
    LFTask->SetName("RF");
    LFTask->SetDesiredValue( GetPushedFrame( (*robot_).links[left_foot_id].T_World ) );
    LFTask->SetDesiredVelocity( Vector(6,0.0) );
    LFTask->SetGains(0,0,0);

    // priority 2
    // CoM task
    cout << "Create CoM foot task" << endl;
    COMTask* comTask = new COMTask( (*robot_) );
    // maintain position relative to foot
    comTask->SetBaseLinkNo( left_foot_id ); // LAR
    comTask->SetPriority(2);
    comTask->SetDesiredValue( comTask->GetSensedValue( GetKrisVector(q_init) ) );
    comTask->SetDesiredVelocity( Vector(3,0.0) );
    comTask->SetGains(-10.0,-0.5,-0.5);
    Vector weights(3);
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 0.1;
    comTask->SetWeight(weights);

    // link-13 hand task
    cout << "Create hand task" << endl;
    LinkTask* handTask = new LinkTask( (*robot_), hand_id, "position" ); // LWP
    // maintain position relative to foot
    handTask->SetBaseLinkNo( left_foot_id ); // LAR
    // point around middle of palm
    handTask->SetLocalPosition( Vector3(0.0,0.0,-0.07) );
    handTask->SetName("handTask");
    handTask->SetDesiredValue( handTask->GetSensedValue( GetKrisVector(q_init) ) );
    handTask->SetDesiredVelocity( Vector(3,0.0) );
    handTask->SetGains(-10,-0.1,-1);
    handTask->SetPriority(2);
    handTask->SetWeight(Vector(1,1.0));

    // joint task
    cout << "Create joint task" << endl;
    std::vector<JointTask*> jointTasks;

    for( int i=0;i<int(q_init.size());i++)
        jointTasks.push_back(new JointTask((*robot_),GetStdVector(i)));

    for( int i=0;i<int(q_init.size());i++)
    {
        jointTasks[i]->SetName( linkNames_[i] );
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

    opController_ = new OperationalSpaceController((*robot_), dt_);
    opController_->AddTask(RFTask);
    opController_->AddTask(LFTask);
    opController_->AddTask(comTask);
    opController_->AddTask(handTask);

    for( int i=0;i<q_init.size();i++)
        opController_->AddTask( jointTasks[i] );
}

// Triggers Operational Space Controller to compute (qdes, dqdes),
// and update tasks states """
std::pair<OpVect,OpVect> DRCHuboOpSpace::Trigger( const OpVect& q, const OpVect& dq )
{
    opController_->SetDesiredValuesFromConfig( GetKrisVector(q) );
    opController_->SetDesiredVelocityFromDifference( GetKrisVector(dq)+GetKrisVector(q), GetKrisVector(q), dt_ );

    std::pair<OpVect,OpVect> out;
    if( use_mapping_ )
    {
        out.first = MapConfig( GetStdVector( out_tmp.first ), true );
        out.second = MapConfig( GetStdVector( out_tmp.second ), true );
    }
    else {
        out.first = GetStdVector( out_tmp.first ); //
        out.second = GetStdVector( out_tmp.second );
    }

    opController_->Advance( GetKrisVector(out.second), GetKrisVector(out.first), dt_ );
    //opController_->PrintStatus( GetKrisVector(out.second) );
    return out;
}

void DRCHuboOpSpace::InitMaps()
{
    // Version 2
    //rs_map["Hip"]= 5;
    //rs_map["leftFoot"]= 12;
    //rs_map["leftPalm"]= 31;
    //rs_map["rightFoot"]= 19;
    //rs_map["Torso, 5
    rs_map["LSP"]= 6;
    rs_map["LSR"]= 7;
    rs_map["LSY"]= 8;
    rs_map["LEP"]= 9;
    rs_map["LWY"]= 10;
    rs_map["LWP"]= 11;
    rs_map["LWR"]= 12;
    rs_map["LF11"]= 13;
    rs_map["LF12"]= 14;
    rs_map["LF13"]= 15;
    rs_map["LF21"]= 16;
    rs_map["LF22"]= 17;
    rs_map["LF23"]= 18;
    rs_map["LF31"]= 19;
    rs_map["LF32"]= 20;
    rs_map["LF33"]= 21;
    //leftPalm"]= 22
    rs_map["NKY"]= 23;
    rs_map["NK1"]= 24;
    rs_map["NK2"]= 25;
    rs_map["RSP"]= 26;
    rs_map["RSR"]= 27;
    rs_map["RSY"]= 28;
    rs_map["REP"]= 29;
    rs_map["RWY"]= 30;
    rs_map["RWP"]= 31;
    rs_map["RWR"]= 32;
    rs_map["RF11"]= 33;
    rs_map["RF12"]= 34;
    rs_map["RF13"]= 35;
    rs_map["RF21"]= 36;
    rs_map["RF22"]= 37;
    rs_map["RF23"]= 38;
    rs_map["RF31"]= 39;
    rs_map["RF32"]= 40;
    rs_map["RF33"]= 41;
    rs_map["RF41"]= 42;
    rs_map["RF42"]= 43;
    rs_map["RF43"]= 44;
    //rightPalm"]= 45
    rs_map["TSY"]= 46;
    rs_map["LHY"]= 47;
    rs_map["LHR"]= 48;
    rs_map["LHP"]= 49;
    rs_map["LKP"]= 50;
    rs_map["LAP"]= 51;
    rs_map["LAR"]= 52;
    //leftFoot"]= 53
    rs_map["RHY"]= 54;
    rs_map["RHR"]= 55;
    rs_map["RHP"]= 56;
    rs_map["RKP"]= 57;
    rs_map["RAP"]= 58;
    rs_map["RAR"]= 59;
    //rightFoot, 60

    // urdf_map["base0"] = 0;
    // urdf_map["base1"] = 1;
    // urdf_map["base2"] = 2;
    // urdf_map["base3"] = 3;
    // urdf_map["base4"] = 4;
    // urdf_map["Torso"]= 5;
    urdf_map["LSP"]= 6;
    urdf_map["LSR"]= 7;
    urdf_map["LSY"]= 8;
    urdf_map["LEP"]= 9;
    urdf_map["LWY"]= 10;
    urdf_map["LWP"]= 11;
    urdf_map["LWR"]= 12;
    urdf_map["LF11"]= 13;
    urdf_map["LF12"]= 14;
    urdf_map["LF13"]= 15;
    urdf_map["LF21"]= 16;
    urdf_map["LF22"]= 17;
    urdf_map["LF23"]= 18;
    urdf_map["LF31"]= 19;
    urdf_map["LF32"]= 20;
    urdf_map["LF33"]= 21;
    //urdf_map["leftPalm"]= 22;
    urdf_map["NKY"]= 23;
    urdf_map["NK1"]= 24;
    urdf_map["NK2"]= 25;
    urdf_map["RSP"]= 26;
    urdf_map["RSR"]= 27;
    urdf_map["RSY"]= 28;
    urdf_map["REP"]= 29;
    urdf_map["RWY"]= 30;
    urdf_map["RWP"]= 31;
    urdf_map["RWR"]= 32;
    urdf_map["RF11"]= 33;
    urdf_map["RF12"]= 34;
    urdf_map["RF13"]= 35;
    urdf_map["RF21"]= 36;
    urdf_map["RF22"]= 37;
    urdf_map["RF23"]= 38;
    urdf_map["RF31"]= 39;
    urdf_map["RF32"]= 40;
    urdf_map["RF33"]= 41;
    urdf_map["RF41"]= 42;
    urdf_map["RF42"]= 43;
    urdf_map["RF43"]= 44;
    // urdf_map["rightPalm"]= 45;
    urdf_map["TSY"]= 46;
    urdf_map["LHY"]= 47;
    urdf_map["LHR"]= 48;
    urdf_map["LHP"]= 49;
    urdf_map["LKP"]= 50;
    urdf_map["LAP"]= 51;
    urdf_map["LAR"]= 52;
    // urdf_map["leftFoot"]= 53;
    urdf_map["RHY"]= 54;
    urdf_map["RHR"]= 55;
    urdf_map["RHP"]= 56;
    urdf_map["RKP"]= 57;
    urdf_map["RAP"]= 58;
    urdf_map["RAR"]= 59;
    // urdf_map["rightFoot"]= 60;
}
