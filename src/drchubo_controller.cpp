/*
 * (C) Copyright 2013 WPI-ARC (http://arc.wpi.edu) and others.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Lesser General Public License
 * (LGPL) version 2.1 which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/lgpl-2.1.html
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * Contributors:
 *      Jim Mainprice
 */

#include "drchubo_controller.h"
#include "tasks.h"
#include "controller.h"
#include "op_utils.h"
#include "urdf_parser.h"

#include "statistics/UniformDistribution.h"
#include "robotics/RobotDynamics3D.h"

#include <sys/time.h>
#include <GL/gl.h>

using namespace OpSpaceControl;

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
    opController_ = NULL;
    q_last_ = OpVect(0);
}

DRCHuboOpSpace::~DRCHuboOpSpace()
{
    cout << "delete all " << endl;
    delete opController_;
}

OpVect DRCHuboOpSpace::JointMappingToRobot( const OpVect& q, bool map_out ) // TODO use hardoded to be faster
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

void DRCHuboOpSpace::CreateTasks( const OpVect& q_init, double dt )
{
    dt_ = dt; // Set Delta Time

    int waist_id = 46; // waist
    int right_foot_id = 59; //       RAR, hubo+ robotsim -> 62,   drc_hubo urdf -> 59,   drc_hubo robotsim -> 59
    int left_foot_id = 52; //        LAR, hubo+ robotsim -> 56,   drc_hubo urdf -> 52,   drc_hubo robotsim -> 52
    int hand_id = 11; //             LWP, hubo+ robotsim -> 13,   drc_hubo urdf -> 11,   drc_hubo robotsim -> 11
    int right_elbow_id_2 = 30; //       RWR, drc_hubo robotsim -> 32
    int left_elbow_id_2 = 10;  //       LWR, drc_hubo robotsim -> 12
    int right_elbow_id = 9; //       REP, drc_hubo robotsim -> 12
    int left_elbow_id = 29;  //      LEP, drc_hubo robotsim -> 32
    int right_hand_id = 32; //       REP, drc_hubo robotsim -> 12
    int left_hand_id = 12;  //      LEP, drc_hubo robotsim -> 32

    // priority 1
    // right foot task
    cout << "Create right foot task" << endl;
    LinkTask* RFTask = new LinkTask( (*robot_), linkNames_, right_foot_id, "po" ); // RAR
    RFTask->SetPriority(1);
    RFTask->SetName("Left Foot");
    RFTask->SetDesiredValue( GetPushedFrame( (*robot_).links[right_foot_id].T_World ) );
    RFTask->SetDesiredVelocity( Vector(6,0.0) );
    RFTask->SetGains(0,0,0);

    // the same with left foot
    cout << "Create left foot task" << endl;
    LinkTask* LFTask = new LinkTask( (*robot_), linkNames_, left_foot_id, "po"); // LAR
    LFTask->SetPriority(1);
    LFTask->SetName("Right Foot");
    LFTask->SetDesiredValue( GetPushedFrame( (*robot_).links[left_foot_id].T_World ) );
    LFTask->SetDesiredVelocity( Vector(6,0.0) );
    LFTask->SetGains(0,0,0);

    // priority 2
    // CoM task
    cout << "Create CoM foot task" << endl;
    COMTask* comTask = new COMTask( (*robot_), linkNames_ );
    // maintain position relative to foot
    comTask->SetBaseLinkNo( left_foot_id ); // LAR
    comTask->SetPriority(1); // Change to 2
    comTask->SetDesiredValue( comTask->GetSensedValue( q_init ) );
    comTask->SetDesiredVelocity( Vector(0,0.0) );
    comTask->SetGains(-100.0,0,0);
    Vector weights(3);
    weights[0] = 1.0;
    weights[1] = 1.0;
    weights[2] = 1;
    comTask->SetWeight(weights);

//    // link-13 hand task
    cout << "Create hand task" << endl;
    LinkTask* handTask = new LinkTask( (*robot_), linkNames_, hand_id, "position" ); // LWP
    // maintain position relative to foot
    // handTask->SetBaseLinkNo( left_foot_id ); // LAR
    // point around middle of palm
    //handTask->SetLocalPosition( Vector3(0.0,0.0,-0.07) );
    handTask->SetName("handTask");
    handTask->SetDesiredValue( handTask->GetSensedValue( (q_init) ) );
    handTask->SetDesiredVelocity( Vector(3,0.0) );
    handTask->SetGains(-10,-0.1,-1);
    handTask->SetPriority(1);
    handTask->SetWeight(Vector(1,1.0));

    cout << "Create right elbow task" << endl;
    LinkTask* RETask = new LinkTask( *robot_, linkNames_, right_elbow_id, "po", left_foot_id ); // RWR
    RETask->SetLocalPosition( Vector3(0.0,0.0,0.0) ); // TODO see offset
    RETask->SetDesiredValue( RETask->GetSensedValue( (q_init) ) );
    RETask->SetDesiredVelocity( Vector(6,0.0) );
    RETask->SetName("Right Elbow");
    RETask->SetGains(-10, -0, -0);
    RETask->SetPriority(1);
    Vector wre(6,1);
    RETask->SetWeight(wre);

    cout << "Create left elbow task" << endl;
    LinkTask* LETask = new LinkTask( *robot_, linkNames_, left_elbow_id, "po", left_foot_id ); // LWR
    LETask->SetLocalPosition( Vector3(0.0,0.0,0.0) ); // TODO see offset
    LETask->SetDesiredValue( LETask->GetSensedValue( (q_init) ) );
    LETask->SetDesiredVelocity( Vector(6,0.0) );
    LETask->SetName("Left Elbow");
    LETask->SetGains(-10, -0, -0);
    LETask->SetPriority(1);
    Vector wle(6,1);
    LETask->SetWeight(wle);

    cout << "Create right elbow task" << endl;
    LinkTask* RETask_2 = new LinkTask( *robot_, linkNames_, right_elbow_id_2, "po", left_foot_id ); // RWR
    RETask_2->SetLocalPosition( Vector3(0.0,0.0,0.0) ); // TODO see offset
    RETask_2->SetDesiredValue( RETask_2->GetSensedValue( (q_init) ) );
    RETask_2->SetDesiredVelocity( Vector(6,0.0) );
    RETask_2->SetName("Right Elbow 2");
    RETask_2->SetGains(-10, -0, -0);
    RETask_2->SetPriority(1);
    Vector wre_2(6,1);
    RETask_2->SetWeight(wre_2);

    cout << "Create left elbow task" << endl;
    LinkTask* LETask_2 = new LinkTask( *robot_, linkNames_, left_elbow_id_2, "po", left_foot_id ); // LWR
    LETask_2->SetLocalPosition( Vector3(0.0,0.0,0.0) ); // TODO see offset
    LETask_2->SetDesiredValue( LETask_2->GetSensedValue( (q_init) ) );
    LETask_2->SetDesiredVelocity( Vector(6,0.0) );
    LETask_2->SetName("Left Elbow 2");
    LETask_2->SetGains(-10, -0, -0);
    LETask_2->SetPriority(1);
    Vector wle_2(6,1);
    LETask_2->SetWeight(wle_2);

    cout << "Create right hand task" << endl;
    LinkTask* RHTask = new LinkTask( *robot_, linkNames_, right_hand_id, "po", left_foot_id ); // RWR
    RHTask->SetLocalPosition( Vector3(0.0,0.0,0.0) ); // TODO see offset
    RHTask->SetDesiredValue( RHTask->GetSensedValue( (q_init) ) );
    RHTask->SetDesiredVelocity( Vector(6,0.0) );
    RHTask->SetName("Right Hand");
    RHTask->SetGains(-10, -0, -0);
    RHTask->SetPriority(1);
    Vector wrh(6,1);
    RHTask->SetWeight(wrh);

    cout << "Create left hand task" << endl;
    LinkTask* LHTask = new LinkTask( *robot_, linkNames_, left_hand_id, "po", left_foot_id ); // LWR
    LHTask->SetLocalPosition( Vector3(0.0,0.0,0.0) ); // TODO see offset
    LHTask->SetDesiredValue( LHTask->GetSensedValue( (q_init) ) );
    LHTask->SetDesiredVelocity( Vector(6,0.0) );
    LHTask->SetName("Left Hand");
    LHTask->SetGains(-10, -0, -0);
    LHTask->SetPriority(1);
    Vector wlh(6,1);
    LHTask->SetWeight(wlh);

    // joint task
    cout << "Create joint task" << endl;
    std::vector<JointTask*> jointTasks;

    for( int i=0;i<int(q_init.size());i++)
        jointTasks.push_back(new JointTask(*robot_,linkNames_, GetStdVector(i) ));

    for( int i=0;i<int(q_init.size());i++)
    {
        jointTasks[i]->SetName( linkNames_[i] );
        jointTasks[i]->SetGains( -0.1, -0, 0 );
        jointTasks[i]->SetWeight( Vector(1,1.0) );
        jointTasks[i]->SetDesiredValue( Vector(1,q_init[i]) );
        jointTasks[i]->SetDesiredVelocity( Vector(1,0.0) );
        //turn off tasks for the base translation and rotation
        if( i < 6 )
            jointTasks[i]->SetWeight(Vector(1,0.0));

        jointTasks[i]->SetPriority(1);
    }

    // Setup operational space controller
    cout << "Create operational space controller" << endl;

    opController_ = new DRCHuboController( robot_, linkNames_, dt_ );
//    opController_->AddTask(RFTask); // Feet
//    opController_->AddTask(LFTask);
//    opController_->AddTask(comTask); // Center of mass
//    opController_->AddTask(handTask); // hand id
    opController_->AddTask(RHTask); // Hands
    opController_->AddTask(LHTask);
    opController_->AddTask(RETask_2); // Hands
    opController_->AddTask(LETask_2);
    opController_->AddTask(RETask); // Hands
    opController_->AddTask(LETask);


//    for( int i=0;i<q_init.size();i++)
//        opController_->AddTask( jointTasks[i] );
}

void DRCHuboOpSpace::Draw()
{
    if( opController_ == NULL )
        return;
    if( q_last_.size() < 1 )
        return;

    const std::vector<OperationalSpaceTask*> tasks = opController_->GetTasks();

//    cout << tasks.size() << endl;
//    cout << tasks[0]->GetName() << endl;

    for( int i=0;i<int(tasks.size()); i++)
    {
        tasks[i]->DrawGL( q_last_ );
    }
}

double DRCHuboOpSpace::GetRealTime()
{
    timeval tim;
    gettimeofday(&tim, NULL);
    return tim.tv_sec+(tim.tv_usec/1000000.0);
}

// Triggers Operational Space Controller to compute (qdes, dqdes),
// and update tasks states """
std::pair<OpVect,OpVect> DRCHuboOpSpace::Trigger( const OpVect& q_cur, const OpVect& dq_cur,
                                                  const OpVect& q_des, const OpVect& dq_des, double dt )
{
    double start = GetRealTime();

    // Store current and desired
    q_last_ = q_cur;
    q_des_ = q_des;

    // Set desired config and vel
    opController_->SetDesiredValuesFromConfig( q_des );
    opController_->SetDesiredVelocityFromDifference( q_cur, q_des, dt );

    // Solve the operational space control problem
    std::pair<OpVect,OpVect> out = opController_->Solve( q_cur, dq_cur, dt );

    // map output
    if( use_mapping_ )
    {
        out.first  = JointMappingToRobot( out.first, true );
        out.second = JointMappingToRobot( out.second , true );
    }

    // Advance and print status
    opController_->Advance( out.first, out.second, dt );
    //opController_->PrintStatus( q_cur );

    cout << "dt in trigger : " << GetRealTime() - start << endl;
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

//-----------------------------------------------------------
//-----------------------------------------------------------
//-----------------------------------------------------------
//-----------------------------------------------------------

DRCHuboOpSpace::DRCHuboController::DRCHuboController(  RobotDynamics3D* robot, std::vector<std::string>& linkNames, double dt ) :
    OperationalSpaceController( robot, linkNames, dt )
{
    nb_links_ = robot_->links.size();

    active_dofs_.resize( 30 );
    active_dofs_[0]= 6;   // "LSP"
    active_dofs_[1]= 7;   // "LSR"
    active_dofs_[2]= 8;   // "LSY"
    active_dofs_[3]= 9;   // "LEP"
    active_dofs_[4]= 10;  // "LWY"
    active_dofs_[5]= 11;  // "LWP"
    active_dofs_[6]= 12;  // "LWR"
    active_dofs_[7]= 23;  // "NKY"
    active_dofs_[8]= 24;  // "NK1"
    active_dofs_[9]= 25;  // "NK2"
    active_dofs_[10]= 26; // "RSP"
    active_dofs_[11]= 27; // "RSR"
    active_dofs_[12]= 28; // "RSY"
    active_dofs_[13]= 29; // "REP"
    active_dofs_[14]= 30; // "RWY"
    active_dofs_[15]= 31; // "RWP"
    active_dofs_[16]= 32; // "RWR"
    active_dofs_[17]= 46; // "TSY"
    active_dofs_[18]= 47; // "LHY"
    active_dofs_[19]= 48; // "LHR"
    active_dofs_[20]= 49; // "LHP"
    active_dofs_[21]= 50; // "LKP"
    active_dofs_[22]= 51; // "LAP"
    active_dofs_[23]= 52; // "LAR"
    active_dofs_[24]= 54; // "RHY"
    active_dofs_[25]= 55; // "RHR"
    active_dofs_[26]= 56; // "RHP"
    active_dofs_[27]= 57; // "RKP"
    active_dofs_[28]= 58; // "RAP"
    active_dofs_[29]= 59; // "RAR"
}

OpMatrix DRCHuboOpSpace::DRCHuboController::JointMappingToActive( const OpMatrix& m )
{
    OpMatrix out( m.size() ); // int to number of rows
    std::vector<int>::iterator it;
    for( int i=0; i<int(out.size()); i++ )
    {
        OpVect row;
        for( it = active_dofs_.begin(); it!=active_dofs_.end(); it++ )
        {
            row.push_back( m[i][(*it)] );
        }
        out[i] = row;
    }

    return out;
}

OpVect DRCHuboOpSpace::DRCHuboController::JointMappingToActive( const OpVect& v )
{
    OpVect out(v.size());
    std::vector<int>::iterator it;
    for( it = active_dofs_.begin(); it!=active_dofs_.end(); it++ )
    {
        out[(*it)] = v[(*it)];
    }

    return out;
}

OpVect DRCHuboOpSpace::DRCHuboController::JointMappingToFull( const OpVect& v )
{
    OpVect out( nb_links_, 0.0 );

    for( int i=0; i<int(active_dofs_.size()); i++ )
    {
        out[active_dofs_[i]] = v[i];
    }
    return out;
}
