#include "tasks.h"
#include "robotics/RobotDynamics3D.h"
#include "math/SVDecomposition.h"
#include "math/DiagonalMatrix.h"
#include "controller.h"

#include <map>

using namespace op_space_control;
using std::cout;
using std::endl;

OperationalSpaceController::OperationalSpaceController( RobotDynamics3D& robot, double dt) : robot_(robot)
{
    //    taskList_ = []; TODO
    //    dqdes_ = None;
    //    qdes_ = None;
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

OperationalSpaceTask* OperationalSpaceController::GetTaskByName(std::string taskName)
{
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetName() == taskName )
            return taskList_[i];

        return NULL;
    }
}

void OperationalSpaceController::SetDesiredValuesFromConfig(Config qdes)
{
    SetDesiredValuesFromConfig( qdes, taskList_ );
}

void OperationalSpaceController::SetDesiredValuesFromConfig(Config qdes, const std::vector<OperationalSpaceTask*>& tasks )
{
    for( int i=0;i< int(tasks.size()); i++ )
    {
        taskList_[i]->SetDesiredValue( taskList_[i]->GetSensedValue(qdes) );
    }
}

void OperationalSpaceController::SetDesiredVelocityFromDifference( Config qdes0, Config qdes1, double dt )
{
    SetDesiredVelocityFromDifference( qdes0, qdes1, dt, taskList_ );
}

void OperationalSpaceController::SetDesiredVelocityFromDifference( Config qdes0, Config qdes1, double dt, const std::vector<OperationalSpaceTask*>& tasks )
{
    for( int i=0;i< int(tasks.size()); i++ )
    {
        Vector xdes0 = taskList_[i]->GetSensedValue(qdes0);
        Vector xdes1 = taskList_[i]->GetSensedValue(qdes1);
        Vector dx = taskList_[i]->TaskDifference(xdes1,xdes0) / dt;
        taskList_[i]->SetDesiredVelocity(dx);
    }
}

void OperationalSpaceController::PrintStatus(Config q)
{
    std::vector<int> priorities;
    std::map<int,std::string> names;
    std::map<int,double> errors;
    std::map<int,double> totalerrors;

    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetWeight().empty() )
            continue;
        priorities.push_back( taskList_[i]->GetPriority() );
        Vector err = taskList_[i]->GetSensedError(q);
        names[taskList_[i]->GetPriority()] = taskList_[i]->GetName();
//        names.setdefault(t.level,[]).append(s);
//        errors.setdefault(t.level,[]).append("%.3f"%(vectorops.norm(err)),);
//        werrsq = vectorops.normSquared(vectorops.mul(err,t.weight));
//        totalerrors[t.level] = totalerrors.get(t.level,0.0) + werrsq;
    }
    int cols = 5;
    int colwidth = 10;
    for( int p=0;p< int(priorities.size()); p++ )
    {
        cout << "Priority" << p << "weighted error^2" << totalerrors[p] << endl;
        std::string pnames = names[p];
        double perrs = errors[p];
        int start = 0;
        while( start < pnames.size() )
        {
            int last = std::min( start+cols, int(pnames.size()) );
            cout << "  Name:  ";
            for( int j=start; j<last; j++ )
            {
                //cout << pnames[i] << ' '*(colwidth-len(pnames[i])) << endl; TODO
            }
            cout << "  Error: ";
            for( int j=start; j<last; j++ )
            {
                //cout << perrs[i] << ' '*(colwidth-len(perrs[i])) << endl; TODO
            }
            cout << endl;
            start=last;
        }
    }
}

Matrix OperationalSpaceController::GetStackedJacobian( Config q, Vector dq, int priority )
{
    Matrix J;
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetWeight().empty() )
            continue;

        if( taskList_[i]->GetPriority() == priority )
        {
            Matrix Jtemp = taskList_[i]->GetJacobian(q);
            // scale by weight
            if( taskList_[i]->GetWeight().size() > 1 )
            {
                assert(taskList_[i]->GetWeight().size()==Jtemp.numCols()); // TODO check collums

                Vector row;
                // treat as an elementwise weight
                for( int j=0;j< int(Jtemp.numRows()); j++ )
                {
                    Jtemp.getRowRef( j, row );
                    row = Jtemp.row(j) * taskList_[j]->GetWeight()[j]; // TODO check that it's rows or coll ??
                }
            }
            else
            {
                Vector row;
                // treat as an elementwise weight
                for( int j=0;j< int(Jtemp.numRows()); j++ )
                {
                    Jtemp.getRowRef( j, row );
                    row = Jtemp.row(j) * taskList_[j]->GetWeight()[0]; // TODO check that it's rows or coll ??
                }
            }

            if( J.numRows() == 0 && J.numCols() == 0 ) // TODO
                J = Jtemp;
            else
            {
                //Math::VStack(Jtemp,J); // TODO
            }
        }
    }
    return J;
}

Vector OperationalSpaceController::GetStackedVelocity(Config q, Vector dq, int priority)
{
    Vector V;
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        if( taskList_[i]->GetWeight().empty() )
            continue;

        if( taskList_[i]->GetPriority() == priority )
        {
            // scale by weight
            Vector Vtemp = taskList_[i]->GetCommandVelocity( q, dq, dt_) * taskList_[i]->GetWeight()[0]; // TODO

            if( V.empty() ) // TODO
            {
                V = Vtemp;
            }
            else
            {
                //Math::HStack( Vtemp, V );
                //V = np.hstack((V, )); // TODO
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
        for( int i=0; i<dqdes_.size();i++)
        {
            dqdes_[i] /= m;
        }
    }
}

Vector OperationalSpaceController::Solve( Config q, Vector dq, double dt )
{
    for( int i=0;i< int(taskList_.size()); i++ )
    {
        taskList_[i]->UpdateState( q, dq, dt );
    }

    // priority 1
    Matrix J1 = GetStackedJacobian(q,dq,1);
    Vector v1 = GetStackedVelocity(q,dq,1);
    //Matrix J1inv = np.linalg.pinv(np.array(J1), rcond=1e-3);
    Matrix J1inv;
    Math::SVDecomposition<double> svd(J1);
    svd.getInverse( J1inv );
    Vector dq1; J1inv.mul(v1,dq1);

    // priority 2
    Matrix eye(dq1.size(),dq1.size()); eye.setIdentity();
    Matrix Np; J1inv.mul( J1, Np );
    Matrix N; N.sub( eye, Np );
    Matrix Jtask = GetStackedJacobian(q,dq,2);

    Vector dqtask;

    if ( Jtask.numCols() != 0 && Jtask.numRows() )
    {
        Vector Vtask = GetStackedVelocity(q,dq,2);
        Matrix JtaskN; Jtask.mul(Jtask,N);
        //assert np.isfinite(Jtask).all(); TODO
        Vector Jtasktmp; Jtask.mul( dq1, Jtasktmp );
        Vector Vtask_m_resid; Vtask_m_resid.sub( Vtask , Jtasktmp );
        Vector z;
        try
        {
            Matrix JtaskNinv;
            Math::SVDecomposition<double> svd(JtaskN);
            svd.getInverse( JtaskNinv );
            JtaskNinv.mul(Vtask_m_resid,z);
            //JtaskNinv = np.linalg.pinv(JtaskN, rcond=1e-3);
            //z = JtaskNinv.dot(Vtask_m_resid);
        }
        catch(...)
        {
            //except np.linalg.LinAlgError:
                // print "SVD failed, trying lstsq"
                //z = np.linalg.lstsq(Jtask,Vtask_m_resid,rcond=1e-3)[0];
                //dqtask = np.dot(N, z)
        }
    }
    else
    {
        // dqtask = [0.0]*len(dq1); TODO
    }

    // compose the velocities together
    dqdes_.add( dq1, dqtask );
    CheckMax(1);

    qdes_.madd( q, dt_); // TODO Check

    //return (dqdes_, qdes_); // TODO stack them
    return dqdes_;
}

void OperationalSpaceController::Advance( Config q, Vector dq, double dt )
{
    for( int i=0; i<int(taskList_.size()); i++ )
    {
        taskList_[i]->Advance(q,dq,dt);
    }
}
