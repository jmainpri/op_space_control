#include "tasks.h"
#include "robotics/RobotDynamics3D.h"
#include "math/SVDecomposition.h"
#include "math/DiagonalMatrix.h"
#include "controller.h"
#include "op_utils.h"

#include <map>
#include <sstream>
#include <iomanip>

using namespace op_space_control;
using std::cout;
using std::endl;

OperationalSpaceController::OperationalSpaceController( RobotDynamics3D& robot, double dt) : robot_(robot)
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

void OperationalSpaceController::SetDesiredValuesFromConfig( const Config& qdes )
{
    SetDesiredValuesFromConfig( qdes, taskList_ );
}

void OperationalSpaceController::SetDesiredValuesFromConfig( const Config& qdes, const std::vector<OperationalSpaceTask*>& tasks )
{
    for( int i=0;i< int(tasks.size()); i++ )
    {
        taskList_[i]->SetDesiredValue( taskList_[i]->GetSensedValue(qdes) );
    }
}

void OperationalSpaceController::SetDesiredVelocityFromDifference( const Config& qdes0, const Config& qdes1, double dt )
{
    SetDesiredVelocityFromDifference( qdes0, qdes1, dt, taskList_ );
}

void OperationalSpaceController::SetDesiredVelocityFromDifference( const Config& qdes0, const Config& qdes1, double dt,
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

void OperationalSpaceController::PrintStatus(const Config& q)
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
        cout << "Priority" << p << "weighted error^2" << totalerrors[p] << endl;
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

Matrix OperationalSpaceController::GetStackedJacobian( const Config& q, const Vector& dq, int priority )
{
    Matrix J;

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
    return J;
}

Vector OperationalSpaceController::GetStackedVelocity( const Config& q, const Vector& dq, int priority)
{
    Vector V;
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetWeight().empty() )
            continue;
        if( taskList_[i]->GetWeight()[0] == 0.0 )
            continue;

        if( taskList_[i]->GetPriority() == priority )
        {
            // scale by weight
            Vector Vtemp = taskList_[i]->GetCommandVelocity( q, dq, dt_) * taskList_[i]->GetWeight()[0]; // TODO why size 0???

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

void OperationalSpaceController::CheckMax(double limit)
{
    Vector limits = robot_.velMax;

    for(int i=0;i<dqdes_.size();i++)
        limits[i] = dqdes_[i] / limits[i];

    double m = limits.maxElement();

    if( m > limit )
    {
        for( int i=0; i<dqdes_.size();i++) // Scale down velocity
            dqdes_[i] /= m;
    }
}

Vector OperationalSpaceController::Solve( const Config& q, const Vector& dq, double dt )
{
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        taskList_[i]->UpdateState( q, dq, dt );
    }

    // priority 1
    Matrix J1 = GetStackedJacobian( q, dq, 1 );
    Vector v1 = GetStackedVelocity( q, dq, 1 );
    Matrix J1inv;
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
        Matrix JtaskN; JtaskN.mul( Jtask, N );
        //assert np.isfinite(Jtask).all(); TODO
        Vector Jtasktmp; Jtask.mul( dq1, Jtasktmp );
        Vector Vtask_m_resid; Vtask_m_resid.sub( Vtask , Jtasktmp );
        Vector z;
        try
        {
            Matrix JtaskNinv;
            Math::SVDecomposition<double> svd(JtaskN);
            svd.getInverse( JtaskNinv );
            JtaskNinv.mul( Vtask_m_resid, z );
        }
        catch(...)
        {
            //except np.linalg.LinAlgError:
            // print "SVD failed, trying lstsq"
            //z = np.linalg.lstsq(Jtask,Vtask_m_resid,rcond=1e-3)[0];
        }
        N.mul( z, dqtask );
    }
    else
    {
        dqtask = Vector(dq1.size(),0.0);
    }

    // compose the velocities together
    dqdes_ = dq1 + dqtask;
    CheckMax(1);

    Config q_tmp(q);
    q_tmp.madd( dqdes_, dt_ );
    qdes_ = q_tmp;

    //return (dqdes_, qdes_); // TODO stack them
    return dqdes_;
}

void OperationalSpaceController::Advance( const Config& q, const Vector& dq, double dt )
{
    for( int i=0; i<int(taskList_.size()); i++ )
    {
        taskList_[i]->Advance( q, dq ,dt );
    }
}
