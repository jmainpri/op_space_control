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

#include "tasks.h"

#include "math/SVDecomposition.h"
#include "math/DiagonalMatrix.h"
#include "robotics/RobotDynamics3D.h"

#include "controller.h"
#include "op_utils.h"

#include <map>
#include <sstream>
#include <iomanip>

#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/QR>

using namespace OpSpaceControl;
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
        const Vector& weight = taskList_[i]->GetWeight();

        if( weight.empty() )
            continue;
        if( weight == 0.0 )
            continue;

        if( taskList_[i]->GetPriority() == priority )
        {
            Matrix Jtemp = taskList_[i]->GetJacobian(q);

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
        const Vector& weight = taskList_[i]->GetWeight();

        if( weight.empty() )
            continue;
        if( weight == 0.0 )
            continue;

        if( taskList_[i]->GetPriority() == priority )
        {
            // scale by weight
            Vector Vtemp =  taskList_[i]->GetCommandVelocity( q, dq, dt_);

            if( weight.size() == 1  )
            {
                for( int j=0;j<Vtemp.size();j++)
                {
                    Vtemp[j] *= weight[0];
                }
            }
            else if (weight.size() == Vtemp.size())
            {
                for( int j=0;j<weight.size();j++)
                {
                    Vtemp[j] *= weight[j]; // TODO why size 0???
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
                //cout << "Vtemp 1 " << Vtemp << endl;
                V = Vtemp;
            }
            else{
                //cout << "Vtemp 2 " << Vtemp << endl;
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

void OperationalSpaceController::SetZeros( OpVect& q )
{
    //rs_fingerL = range(13,22);
    //rs_fingerR = range(33,45);
    //rs_leg = range(47,60);

    for( int i=33; i<=45;i++)
        q[i] = 0;
    for( int i=47; i<=60;i++)
        q[i] = 0;
    for( int i=13; i<=22;i++)
        q[i] = 0;
}

int counter = 1;
/*
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
    Math::SVDecomposition<double> svd( J1 );
    svd.getInverse( J1inv );

    cout << "J1 : " << endl << J1 << endl;
    cout << "J1inv : " <<  endl << J1inv << endl;
//    cout << "v1 : " <<  endl << v1 << endl;
    Vector dq1; J1inv.mul( v1, dq1 );

    // priority 2
    Matrix eye(dq1.size(),dq1.size()); eye.setIdentity();
    Matrix Np(dq1.size(),dq1.size()); Np.mul( J1inv, J1 );
    Matrix N; N.sub( eye, Np );
    Matrix Jtask = GetKrisMatrix( JointMappingToActive( GetStackedJacobian( q, dq, 2 )));
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

//    cout << dt_ << " " << Vector(dqdes_).norm() << endl;
//    cout << "qdes_" << Vector(qdes_) << endl;
//    cout << "dqdes_" << Vector(dqdes_) << endl;

    std::pair<OpVect,OpVect> out; // return configuration and velocity

    SetZeros(qdes_);
    SetZeros(dqdes_);

    out.first = qdes_;
    out.second = dqdes_;

    counter++;
    cout << "counter : " << counter << endl;
//    cout << "dqdes_ : RAP : " << out.second[GetLinkIdByName("Body_RAP")] << endl;
//    cout << "dqdes_ : LAP : " << out.second[GetLinkIdByName("Body_LAP")] << endl;

//    cout << "qdes_ : RAP : " << out.first[GetLinkIdByName("Body_RAP")] << endl;
//    cout << "qdes_ : LAP : " << out.first[GetLinkIdByName("Body_LAP")] << endl;

    return out;
}
*/

void print_matrix_to_python( const Eigen::MatrixXd &mat )
{
    for( int i=0;i<mat.rows();i++)
    {
        if( i == 0)
            cout <<"[";
        for( int j=0;j<mat.cols();j++)
        {
            if( j == 0)
                cout <<"[";

            cout <<  mat(i,j) ;

            if( j == mat.cols()-1 )
                cout <<"]";
            else
                cout <<",";
        }
        if( i == mat.rows()-1 )
            cout <<"]";
        else
            cout << ",";
    }
    cout << endl;
}

// Derived from code by Yohann Solaro ( http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00187.html )
// see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method
Eigen::MatrixXd pinv( const Eigen::MatrixXd &b, double rcond )
{
    // TODO: Figure out why it wants fewer rows than columns
//    if ( a.rows()<a.cols() )
//        return false;
    bool flip = false;
    Eigen::MatrixXd a;
    if( a.rows() < a.cols() )
    {
        a = b.transpose();
        flip = true;
    }
    else
        a = b;

    // SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svdA;
    svdA.compute( a, Eigen::ComputeFullU | Eigen::ComputeThinV );

    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType vSingular = svdA.singularValues();

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::VectorXd vPseudoInvertedSingular( svdA.matrixV().cols() );

    for (int iRow=0; iRow<vSingular.rows(); iRow++)
    {
        if ( fabs(vSingular(iRow)) <= rcond ) // Todo : Put epsilon in parameter
        {
            vPseudoInvertedSingular(iRow)=0.;
        }
        else
            vPseudoInvertedSingular(iRow)=1./vSingular(iRow);
    }

    // A little optimization here
    Eigen::MatrixXd mAdjointU = svdA.matrixU().adjoint().block( 0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols() );

    // Pseudo-Inversion : V * S * U'
    Eigen::MatrixXd a_pinv = (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;

    if( flip )
    {
        a = a.transpose();
        a_pinv = a_pinv.transpose();
    }

    return a_pinv;
}

Eigen::MatrixXd GetEigenMatrix(const Matrix& m)
{
    Eigen::MatrixXd out(m.numRows(),m.numCols());

    for(int i=0;i<m.numRows();i++)
        for(int j=0;j<m.numCols();j++)
            out(i,j) = m(i,j);

    return out;
}

Eigen::VectorXd GetEigenVector(const Vector& v)
{
    Eigen::VectorXd out(v.size());

    for(int i=0;i<out.size();i++)
        out[i] = v[i];

    return out;
}

Vector GetKrisVector(const Eigen::VectorXd& v)
{
    Vector out(v.size());

    for(int i=0;i<out.size();i++)
        out[i] = v[i];

    return out;
}

std::pair<OpVect,OpVect> OperationalSpaceController::Solve( const OpVect& q_tmp, const OpVect& dq_tmp, double dt )
{
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        taskList_[i]->UpdateState( q_tmp, dq_tmp, dt );
    }

    // priority 1
    Eigen::MatrixXd J1 = GetEigenMatrix( GetKrisMatrix( GetStackedJacobian( q_tmp, dq_tmp, 1 )));
    Eigen::VectorXd v1 = GetEigenVector( GetStackedVelocity( q_tmp, dq_tmp, 1 ) );
    Eigen::MatrixXd J1inv = pinv( J1, 1e-2 );
    //cout << "v1.size() : " << v1.size() << endl;
    Eigen::VectorXd dq1 = J1inv * v1;

//    cout << " J1 : " << endl << J1 << endl;
    cout << "J1 : " << endl;
    print_matrix_to_python( J1 );
//    cout << " J1inv : " << endl << J1inv << endl;
//    cout << " v1 : " << v1.transpose() << endl;

    // priority 2
    Eigen::MatrixXd Jtask = GetEigenMatrix( GetKrisMatrix( GetStackedJacobian( q_tmp, dq_tmp, 2 )));

    if ( Jtask.cols() != 0 && Jtask.rows() != 0  )
    {
        int m = dq1.size();
        Eigen::VectorXd dqtask( Eigen::VectorXd::Zero(m) );
        Eigen::VectorXd Vtask = GetEigenVector( GetStackedVelocity( q_tmp, dq_tmp, 2 ) );
        Eigen::MatrixXd N = Eigen::MatrixXd::Identity(m,m) - ( J1inv * J1 );
        Eigen::MatrixXd JtaskN = Jtask * N;
        Eigen::VectorXd Vtask_m_resid = Vtask - Jtask * dq1;
        Eigen::MatrixXd JtaskNinv = pinv( JtaskN, 1e-2 );
        Eigen::VectorXd z = JtaskNinv * Vtask_m_resid;
        dqtask = N * z;

        // compose the velocities together
        dqdes_ = GetKrisVector( dq1 + dqtask );
    }
    else {
        dqdes_ =  GetKrisVector( dq1 );
    }

    CheckMax(1);

    Config q_tmp_out( q_tmp );
    q_tmp_out.madd( dqdes_, dt_ );
    qdes_ = q_tmp_out;

    std::pair<OpVect,OpVect> out; // return configuration and velocity

    SetZeros( qdes_ );
    SetZeros( dqdes_ );

    out.first = qdes_;
    out.second = dqdes_;

    counter++;
//    cout << "counter : " << counter << endl;
//    cout << "dt : " << dt << endl;
    return out;
}

void OperationalSpaceController::Advance( const OpVect& q, const OpVect& dq, double dt )
{
    for( int i=0; i<int(taskList_.size()); i++ )
    {
        taskList_[i]->Advance( q, dq ,dt );
    }
}
