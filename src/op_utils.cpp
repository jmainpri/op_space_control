#include "op_utils.h"

#include "robotics/RobotDynamics3D.h"

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

std::vector<int> GetStdVector(int value)
{
    std::vector<int> vect;
    vect.push_back(value);
    return vect;
}

std::vector<double> GetStdVector(double value)
{
    std::vector<double> vect;
    vect.push_back(value);
    return vect;
}

std::vector<double> GetStdVector(const Vector3& pos)
{
    std::vector<double> vect(3);
    vect[0] = pos[0];
    vect[1] = pos[1];
    vect[2] = pos[2];
    return vect;
}

Vector3 GetVector3(const Vector& vect)
{
    Vector3 pos;
    pos[0] = vect[0];
    pos[1] = vect[1];
    pos[2] = vect[2];
    return pos;
}

void PushRotationToVector( const Matrix3& R, Vector& x )
{
    int size = x.size();
    Vector x_temp = x;
    x.resize(size+9);

    for(int i=0;i<size;i++) // Copy the old values to resized vector
        x[i] = x_temp[i];

    x[size+0] = R(0,0); x[size+1] = R(0,1); x[size+2] = R(0,2);
    x[size+3] = R(1,0); x[size+4] = R(1,1); x[size+5] = R(1,2);
    x[size+6] = R(2,0); x[size+7] = R(2,1); x[size+8] = R(2,2);
}

void PopRotationFromVector( Vector& x, Matrix3& R )
{
    int size = x.size();
    R(0,0) = x[size-9]; R(0,1) = x[size-8]; R(0,2) = x[size-7];
    R(1,0) = x[size-6]; R(1,1) = x[size-5]; R(1,2) = x[size-4];
    R(2,0) = x[size-3]; R(2,1) = x[size-2]; R(2,2) = x[size-1];

    Vector x_temp = x;
    x.resize(size-9);

    for(int i=0;i<x.size();i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];
}

void PushPosToVector( const Vector3& p, Vector& x )
{
    int size = x.size();
    Vector x_temp = x;
    x.resize(size+3);

    for(int i=0;i<size;i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];

    x[size+0] = p[0];
    x[size+1] = p[1];
    x[size+2] = p[2];
}

void PopPosFromVector( Vector& x, Vector3& p )
{
    int size = x.size();
    p[0]   = x[size-3];
    p[1]   = x[size-2];
    p[2]   = x[size-1];

    Vector x_temp = x;
    x.resize(size-3);

    for(int i=0;i<x.size();i++) // TODO is that necessary?? Copy the old values to resized vector
        x[i] = x_temp[i];
}

void PushFrameToVector( const Frame3D& T, Vector& x )
{
    PushRotationToVector( T.R, x );
    PushPosToVector( T.t, x );
}

void PopFrameFromVector( Vector& x, Frame3D& T )
{
    PopPosFromVector( x, T.t );
    PopRotationFromVector( x, T.R );
}

Vector GetPushedFrame( const Frame3D& T )
{
    Vector vect;
    PushFrameToVector(T,vect);
    return vect;
}

// used in python interface
Matrix GetJacobian( RobotDynamics3D& robot, int index, Vector3 p )
{
    Matrix Jmat;
    robot.GetFullJacobian( p, index, Jmat );
    return Jmat;
}

// used in python interface
Matrix GetPositionJacobian( RobotDynamics3D& robot, int index,  Vector3 p )
{
    Matrix Jmat;
    robot.GetPositionJacobian( p, index, Jmat );
    return Jmat;
}

// used in python interface
Matrix GetOrientationJacobian( RobotDynamics3D& robot, int index )
{
    Matrix Jmat;
    // TODO exeption
    //throw PyException("Orientation Jacobian not supported yet");
    //robot.GetOrientationJacobian(index,Jmat);
    return Jmat;
}
