#include "tasks.h"
#include "robotics/RobotDynamics3D.h"
#include "math/SVDecomposition.h"
#include "math/DiagonalMatrix.h"
#include "controller.h"

#include <map>

using namespace op_space_control;
using std::cout;
using std::endl;

OperationalSpaceController::OperationalSpaceController( RobotDynamics3D& robot, double dt) : _robot(robot)
{
    //    _taskList = []; TODO
    //    _dqdes = None;
    //    _qdes = None;
    _dt = dt;
}

void OperationalSpaceController::AddTask( OperationalSpaceTask* task )
{
    _taskList.push_back(task);
}

OperationalSpaceTask* OperationalSpaceController::GetTaskByName(std::string taskName)
{
    for( int i=0;i< int(_taskList.size()); i++ )
    {
        if( _taskList[i]->GetName() == taskName )
            return _taskList[i];

        return NULL;
    }
}

void OperationalSpaceController::SetDesiredValuesFromConfig(Config qdes)
{
    SetDesiredValuesFromConfig( qdes, _taskList );
}

void OperationalSpaceController::SetDesiredValuesFromConfig(Config qdes, const std::vector<OperationalSpaceTask*>& tasks )
{
    for( int i=0;i< int(tasks.size()); i++ )
    {
        _taskList[i]->SetDesiredValue( _taskList[i]->GetSensedValue(qdes) );
    }
}

void OperationalSpaceController::SetDesiredVelocityFromDifference( Config qdes0, Config qdes1, double dt )
{
    SetDesiredVelocityFromDifference( qdes0, qdes1, dt, _taskList );
}

void OperationalSpaceController::SetDesiredVelocityFromDifference( Config qdes0, Config qdes1, double dt, const std::vector<OperationalSpaceTask*>& tasks )
{
    for( int i=0;i< int(tasks.size()); i++ )
    {
        Vector xdes0 = _taskList[i]->GetSensedValue(qdes0);
        Vector xdes1 = _taskList[i]->GetSensedValue(qdes1);
        Vector dx = _taskList[i]->TaskDifference(xdes1,xdes0) / dt;
        _taskList[i]->SetDesiredVelocity(dx);
    }
}

void OperationalSpaceController::PrintStatus(Config q)
{
    std::vector<int> priorities;
    std::map<int,std::string> names;
    std::map<int,double> errors;
    std::map<int,double> totalerrors;

    for( int i=0;i< int(_taskList.size()); i++ )
    {
        if( _taskList[i]->GetWeight().empty() )
            continue;
        priorities.push_back( _taskList[i]->GetPriority() );
        Vector err = _taskList[i]->GetSensedError(q);
        names[_taskList[i]->GetPriority()] = _taskList[i]->GetName();
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
    for( int i=0;i< int(_taskList.size()); i++ )
    {
        if( _taskList[i]->GetWeight().empty() )
            continue;

        if( _taskList[i]->GetPriority() == priority )
        {
            Matrix Jtemp = _taskList[i]->GetJacobian(q);
            // scale by weight
            if( _taskList[i]->GetWeight().size() > 1 )
            {
                assert(_taskList[i]->GetWeight().size()==Jtemp.numCols()); // TODO check collums

                Vector row;
                // treat as an elementwise weight
                for( int j=0;j< int(Jtemp.numRows()); j++ )
                {
                    Jtemp.getRowRef( j, row );
                    row = Jtemp.row(j) * _taskList[j]->GetWeight()[j]; // TODO check that it's rows or coll ??
                }
            }
            else
            {
                Vector row;
                // treat as an elementwise weight
                for( int j=0;j< int(Jtemp.numRows()); j++ )
                {
                    Jtemp.getRowRef( j, row );
                    row = Jtemp.row(j) * _taskList[j]->GetWeight()[0]; // TODO check that it's rows or coll ??
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
    for( int i=0;i< int(_taskList.size()); i++ )
    {
        if( _taskList[i]->GetWeight().empty() )
            continue;

        if( _taskList[i]->GetPriority() == priority )
        {
            // scale by weight
            Vector Vtemp = _taskList[i]->GetCommandVelocity( q, dq, _dt) * _taskList[i]->GetWeight()[0]; // TODO

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
    Vector limits = _robot.velMax;

    for(int i=0;i<_dqdes.size();i++)
        limits[i] = _dqdes[i] / limits[i];

    double m = limits.maxElement();

    if( m > limit )
    {
        for( int i=0; i<_dqdes.size();i++)
        {
            _dqdes[i] /= m;
        }
    }
}

Vector OperationalSpaceController::Solve( Config q, Vector dq, double dt )
{
    for( int i=0;i< int(_taskList.size()); i++ )
    {
        _taskList[i]->UpdateState( q, dq, dt );
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
    _dqdes.add( dq1, dqtask );
    CheckMax(1);

    _qdes.madd( q, _dt); // TODO Check

    //return (_dqdes, _qdes); // TODO stack them
    return _dqdes;
}

void OperationalSpaceController::Advance( Config q, Vector dq, double dt )
{
    for( int i=0; i<int(_taskList.size()); i++ )
    {
        _taskList[i]->Advance(q,dq,dt);
    }
}
