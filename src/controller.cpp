#include "tasks.h"

#include "math/SVDecomposition.h"
#include "math/DiagonalMatrix.h"
#include "robotics/RobotDynamics3D.h"

#include "controller.h"
#include "op_utils.h"

#include <map>
#include <sstream>
#include <iomanip>

using namespace op_space_control;
using std::cout;
using std::endl;

OperationalSpaceController::OperationalSpaceController( RobotDynamics3D* robot, std::vector<std::string>& linkNames, double dt) :
    robot_(robot), linkNames_(linkNames)
{
    taskList_.clear();
    dqdes_.clear();
    qdes_.clear();
    dt_ = dt;
}

OperationalSpaceController::~OperationalSpaceController()
{
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        delete taskList_[i];
    }
}

int OperationalSpaceController::GetLinkIdByName(const std::string& name)
{
    for(int i=0; i<int(robot_->links.size());i++)
    {
        if( linkNames_[i] == name )
            return i;
    }

    std::cout << "Error getting link by name in " << __func__ << std::endl;
    return 0;
}

const RobotLink3D* OperationalSpaceController::OperationalSpaceController::GetLinkByName(const std::string& name)
{
    return (&robot_->links[GetLinkIdByName(name)]);
}

void OperationalSpaceController::AddTask( OperationalSpaceTask* task )
{
    taskList_.push_back(task);
}

OperationalSpaceTask* OperationalSpaceController::GetTaskByName(const std::string& taskName) const
{
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetName() == taskName )
            return taskList_[i];

        return NULL;
    }
}

void OperationalSpaceController::SetDesiredValuesFromConfig( const OpVect& qdes )
{
    SetDesiredValuesFromConfig( qdes, taskList_ );
}

void OperationalSpaceController::SetDesiredValuesFromConfig( const OpVect& qdes, const std::vector<OperationalSpaceTask*>& tasks )
{
    for( int i=0;i< int(tasks.size()); i++ )
    {
        taskList_[i]->SetDesiredValue( taskList_[i]->GetSensedValue(qdes) );
    }
}

void OperationalSpaceController::SetDesiredVelocityFromDifference( const OpVect& qdes0, const OpVect& qdes1, double dt )
{
    SetDesiredVelocityFromDifference( qdes0, qdes1, dt, taskList_ );
}

void OperationalSpaceController::SetDesiredVelocityFromDifference( const OpVect& qdes0, const OpVect& qdes1, double dt,
                                                                   const std::vector<OperationalSpaceTask*>& tasks )
{
    for( int i=0;i< int(tasks.size()); i++ )
    {
        Vector xdes0 = taskList_[i]->GetSensedValue( qdes0 );
        Vector xdes1 = taskList_[i]->GetSensedValue( qdes1 );
        Vector dx = taskList_[i]->TaskDifference( xdes1, xdes0 ) / dt;
        taskList_[i]->SetDesiredVelocity( dx );
    }
}

void OperationalSpaceController::PrintStatus(const OpVect& q)
{
    std::vector<int> priorities;
    std::map<int, std::vector<std::string> > names;
    std::map<int, std::vector<std::string> > errors;
    std::map<int,double> totalerrors;

    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetWeight().empty() )
            continue;

        int p = taskList_[i]->GetPriority();

        priorities.push_back( p );

        std::ostringstream num_str;
        num_str << std::setprecision(2)
                << std::scientific
                << std::fixed << taskList_[i]->GetSensedError(q).norm();

        errors[p].push_back( num_str.str() );
        names[p].push_back( taskList_[i]->GetName() );
        // totalerrors TODO
    }

    int cols = 5;
    int colwidth = 10;
    for( int p=0;p< int(priorities.size()); p++ )
    {
        //cout << "Priority" << p << "weighted error^2" << totalerrors[p] << endl;
        std::vector<std::string> pnames = names[p];
        std::vector<std::string> perrs = errors[p];
        int start = 0;
        while( start < int(pnames.size()) )
        {
            int last = std::min( start+cols, int(pnames.size()) );
            cout << "  Name:  ";
            for( int i=start; i<last; i++ )
            {
                std::string spaces( colwidth-pnames[i].size(),' ');
                cout << pnames[i] << spaces ;
            }
            cout << "  Error: ";
            for( int i=start; i<last; i++ )
            {
                std::string spaces( colwidth-perrs[i].size(),' ');
                cout << perrs[i] << spaces ;
            }
            cout << endl;
            start=last;
        }
    }
}

OpMatrix OperationalSpaceController::GetStackedJacobian( const OpVect& q, const OpVect& dq, int priority )
{
    Matrix J(0,0);

    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetWeight().empty() )
            continue;
        if( taskList_[i]->GetWeight()[0] == 0.0 )
            continue;

        if( taskList_[i]->GetPriority() == priority )
        {
            Matrix Jtemp = taskList_[i]->GetJacobian(q);
            const Vector& weight = taskList_[i]->GetWeight();

            // scale by weight
            if( weight.size() > 1 )
            {
                assert(weight.size()==Jtemp.numRows()); // TODO ask Kris to be sure

                Vector row;
                // treat as an elementwise weight
                for( int j=0;j< int(Jtemp.numRows()); j++ )
                {
                    Jtemp.getRowRef( j, row );
                    row = Jtemp.row(j) * weight[j];
                }
            }
            else
            {
                Vector row;
                // treat as an el``ementwise weight
                for( int j=0;j< int(Jtemp.numRows()); j++ )
                {
                    Jtemp.getRowRef( j, row );
                    row = Jtemp.row(j) * weight[0];
                }
            }

            if( J.numRows() == 0 && J.numCols() == 0 )
                J = Jtemp;
            else
                J = VStack( J, Jtemp );
        }
    }

    return GetStdMatrix(J);
}

OpVect OperationalSpaceController::GetStackedVelocity( const OpVect& q, const OpVect& dq, int priority)
{
    Vector V(0);
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetWeight().empty() )
            continue;
        if( taskList_[i]->GetWeight()[0] == 0.0 )
            continue;

        if( taskList_[i]->GetPriority() == priority )
        {
            // scale by weight
            Vector Vtemp = taskList_[i]->GetCommandVelocity( q, dq, dt_);
            Vector Weight = taskList_[i]->GetWeight();

            if( Weight.size() == 1  )
            {
                for( int j=0;j<Vtemp.size();j++)
                {
                    Vtemp[j] *= Weight[0];
                }
            }
            else if (Weight.size() == Vtemp.size())
            {
                for( int j=0;j<Weight.size();j++)
                {
                    Vtemp[j] *= Weight[j]; // TODO why size 0???
                }
            }
            else {
                cout << "Error in weights" << endl;
            }

            if( HasNaN( Vtemp) ) {
                cout << "Vtemp has nan in task : " << taskList_[i]->GetName() << endl;
            }
            if( V.empty() )
            {
                V = Vtemp;
            }
            else{
                V = HStack( V, Vtemp );
            }
        }
    }
    return V;
}

void OperationalSpaceController::CheckMax( double limit )
{
    Vector limits( robot_->velMax );

    for(int i=0;i<dqdes_.size();i++)
        limits[i] = dqdes_[i] / limits[i];

    double m = limits.maxElement();

    if( m > limit )
    {
        //cout << "Exceedes limits" << endl;
        for( int i=0; i<dqdes_.size();i++) // Scale down velocity
            dqdes_[i] /= m;
    }
}

std::pair<OpVect,OpVect> OperationalSpaceController::Solve( const OpVect& q_tmp, const OpVect& dq_tmp, double dt )
{
    Vector q( q_tmp );
    Vector dq( dq_tmp );

    for( int i=0;i< int(taskList_.size()); i++ )
    {
        taskList_[i]->UpdateState( q, dq, dt );
    }

    // priority 1
    Matrix J1 = GetKrisMatrix( JointMappingToActive( GetStackedJacobian( q, dq, 1 ) ));
    Vector v1 = GetStackedVelocity( q, dq, 1 );
    Matrix J1inv;
    Math::SVDecomposition<double> svd(J1);
    svd.getInverse( J1inv );

    cout << "J1 : " << endl << J1 << endl;
    cout << "J1inv : " <<  endl << J1inv << endl;
    cout << "v1 : " <<  endl << v1 << endl;
    Vector dq1; J1inv.mul( v1, dq1 );

    // priority 2
    Matrix eye(dq1.size(),dq1.size()); eye.setIdentity();
    Matrix Np(dq1.size(),dq1.size()); Np.mul( J1inv, J1 );
    Matrix N; N.sub( eye, Np );
    Matrix Jtask = GetKrisMatrix(JointMappingToActive( GetStackedJacobian( q, dq, 2 )));
    Vector dqtask(dq1.size(),0.0);

    if ( Jtask.numCols() != 0 && Jtask.numRows() !=0  )
    {
        Vector Vtask = GetStackedVelocity( q, dq, 2 );
        if( HasNaN(Vtask) ) {
            cout << "Vtask has nan" << endl;
            cout << "Vtask :" << Vtask << endl;
            for(int i=0;i<Vtask.size();i++){
              if(IsNaN(Vtask[i])) Vtask[i] = 0.0;
            }
        }
        Matrix JtaskN; JtaskN.mul( Jtask, N );
        //assert np.isfinite(Jtask).all(); TODO
        Vector Jtasktmp; Jtask.mul( dq1, Jtasktmp );
        Vector Vtask_m_resid; Vtask_m_resid.sub( Vtask , Jtasktmp );
        Vector z;
        if( HasNaN(JtaskN) ) {
            cout << "JtaskN has nan" << endl;
        }
        if( HasNaN(Jtasktmp) ) {
            cout << "Jtasktmp has nan" << endl;
        }
        if( HasNaN(Vtask_m_resid) ) {
            cout << "Vtask_m_resid has nan" << endl;
        }
        try
        {
            Matrix JtaskNinv;
            Math::SVDecomposition<double> svd(JtaskN);
            svd.getInverse( JtaskNinv );
            JtaskNinv.mul( Vtask_m_resid, z );
//            cout << "JtaskNinv : " << JtaskNinv << endl;
//            cout << "Vtask_m_resid : " << Vtask_m_resid << endl;
//            cout << "z : " << z << endl;
        }
        catch(...)
        {
            //except np.linalg.LinAlgError:
            // print "SVD failed, trying lstsq"
            //z = np.linalg.lstsq(Jtask,Vtask_m_resid,rcond=1e-3)[0];
        }
        N.mul( z, dqtask );
        //cout << "N : " << N << endl;
        //cout << "z : " << z << endl;
        //cout << "dqtask : " << dqtask << endl;
    }
    else
    {
        dqtask = Vector(dq1.size(),0.0);
    }

    // compose the velocities together
    dqdes_ = Vector( Vector(JointMappingToFull(dq1)) + Vector(JointMappingToFull(dqtask)) );

//    cout << "dq1 : RAP : " << Vector(JointMappingToFull(dq1))[GetLinkIdByName("Body_RAP")] << endl;
//    cout << "dq1 : LAP : " << Vector(JointMappingToFull(dq1))[GetLinkIdByName("Body_LAP")] << endl;

//    cout << "dqtask : RAP : " << Vector(JointMappingToFull(dqtask))[GetLinkIdByName("Body_RAP")] << endl;
//    cout << "dqtask : LAP : " << Vector(JointMappingToFull(dqtask))[GetLinkIdByName("Body_LAP")] << endl;

    CheckMax(1);

    Config q_tmp_out( q );
    q_tmp_out.madd( dqdes_, dt_ );
    qdes_ = q_tmp_out;

    cout << dt_ << " " << Vector(dqdes_).norm() << endl;
    cout << "qdes_" << Vector(qdes_) << endl;
    cout << "dqdes_" << Vector(dqdes_) << endl;

    std::pair<OpVect,OpVect> out; // return configuration and velocity
    out.first = qdes_;
    out.second = dqdes_;

    cout << "dqdes_ : RAP : " << out.second[GetLinkIdByName("Body_RAP")] << endl;
    cout << "dqdes_ : LAP : " << out.second[GetLinkIdByName("Body_LAP")] << endl;

    cout << "qdes_ : RAP : " << out.first[GetLinkIdByName("Body_RAP")] << endl;
    cout << "qdes_ : LAP : " << out.first[GetLinkIdByName("Body_LAP")] << endl;

    return out;
}

/*
Eigen::MatrixXd GetEigenMatrix(const Matrix& m)
{
    Eigen::MatrixXd out(m.numRows(),m.numCols());

    for(int i=0;i<m.numRows();i++)
    {
        for(int j=0;j<m.numCols();j++)
        {
            out(i,j) = m(i,j);
        }
    }
    return out;
}

Eigen::VectorXd GetEigenVector(const Vector& v)
{
    Eigen::VectorXd out(v.size());
    for(int i=0;i<v.size();i++)
    {
        out[i] = v[i];
    }
    return out;
}

Vector GetKrisVector(const Eigen::VectorXd& v)
{
    Vector out(v.size());
    for(int i=0;i<v.size();i++)
    {
        out[i] = v[i];
    }
    return out;
}

std::pair<Vector,Vector> OperationalSpaceController::Solve( const Config& q_tmp, const Vector& dq_tmp, double dt )
{
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        taskList_[i]->UpdateState( q_tmp, dq_tmp, dt );
    }

    Eigen::VectorXd q(q_tmp);
    Eigen::VectorXd dq(dq_tmp);

    // priority 1
    Eigen::MatrixXd J1 = GetEigenMatrix( GetStackedJacobian( q_tmp, dq_tmp, 1 ) );
    Eigen::VectorXd v1 = GetEigenVector( GetStackedVelocity( q_tmp, dq_tmp, 1 ) );
    Eigen::MatrixXd J1inv = J1.pinverse();

    Math::SVDecomposition<double> svd(J1);
    svd.getInverse( J1inv );
    Vector dq1; J1inv.mul( v1, dq1 );

    // priority 2
    Matrix eye(dq1.size(),dq1.size()); eye.setIdentity();
    Matrix Np(dq1.size(),dq1.size()); Np.mul( J1inv, J1 );
    Matrix N; N.sub( eye, Np );
    Matrix Jtask = GetStackedJacobian( q, dq, 2 );

    Vector dqtask;

    if ( Jtask.numCols() != 0 && Jtask.numRows() !=0  )
    {
        Vector Vtask = GetStackedVelocity( q, dq, 2 );
        if( HasNaN(Vtask) ) {
            cout << "Vtask has nan" << endl;
            cout << "Vtask :" << Vtask << endl;
            for(int i=0;i<Vtask.size();i++){
              if(IsNaN(Vtask[i])) Vtask[i] = 0.0;
            }
        }
        Matrix JtaskN; JtaskN.mul( Jtask, N );
        //assert np.isfinite(Jtask).all(); TODO
        Vector Jtasktmp; Jtask.mul( dq1, Jtasktmp );
        Vector Vtask_m_resid; Vtask_m_resid.sub( Vtask , Jtasktmp );
        Vector z;
        if( HasNaN(JtaskN) ) {
            cout << "JtaskN has nan" << endl;
        }
        if( HasNaN(Jtasktmp) ) {
            cout << "Jtasktmp has nan" << endl;
        }
        if( HasNaN(Vtask_m_resid) ) {
            cout << "Vtask_m_resid has nan" << endl;
        }
        try
        {
            Matrix JtaskNinv;
            Math::SVDecomposition<double> svd(JtaskN);
            svd.getInverse( JtaskNinv );
            JtaskNinv.mul( Vtask_m_resid, z );
//            cout << "JtaskNinv : " << JtaskNinv << endl;
//            cout << "Vtask_m_resid : " << Vtask_m_resid << endl;
//            cout << "z : " << z << endl;
        }
        catch(...)
        {
            //except np.linalg.LinAlgError:
            // print "SVD failed, trying lstsq"
            //z = np.linalg.lstsq(Jtask,Vtask_m_resid,rcond=1e-3)[0];
        }
        N.mul( z, dqtask );
        //cout << "N : " << N << endl;
        //cout << "z : " << z << endl;
        //cout << "dqtask : " << dqtask << endl;
    }
    else
    {
        dqtask = Vector(dq1.size(),0.0);
    }

    //cout << "dq1 : " << dq1 << endl;

    // compose the velocities together
    dqdes_ = dq1 + dqtask;
    CheckMax(1);

    cout << "dq1 : RAR : " << dq1[GetLinkIdByName("Body_RAR")] << endl;
    cout << "dq1 : LAR : " << dq1[GetLinkIdByName("Body_LAR")] << endl;

    cout << "dqtask : RAR : " << dqtask[GetLinkIdByName("Body_RAR")] << endl;
    cout << "dqtask : LAR : " << dqtask[GetLinkIdByName("Body_LAR")] << endl;

    Config q_tmp(q);
    q_tmp.madd( dqdes_, dt_ );
    qdes_ = q_tmp;
    cout << dt_ << " " << dqdes_.norm() << endl;
    //cout << "dqdes_ : " << dqdes_ << endl;

    std::pair<Vector,Vector> out; // return configuration and velocity
    out.first = qdes_;
    out.second = dqdes_;
    return out;
}
*/
void OperationalSpaceController::Advance( const OpVect& q, const OpVect& dq, double dt )
{
    for( int i=0; i<int(taskList_.size()); i++ )
    {
        taskList_[i]->Advance( q, dq ,dt );
    }
}
