#include "op_utils.h"

#include "robotics/RobotDynamics3D.h"

#include <cmath>

using std::cout;
using std::endl;

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

std::vector<int> op_space_control::GetStdVector(int value)
{
    std::vector<int> vect;
    vect.push_back(value);
    return vect;
}

std::vector<double> op_space_control::GetStdVector(double value)
{
    std::vector<double> vect;
    vect.push_back(value);
    return vect;
}

std::vector<double> op_space_control::GetStdVector(const Vector3& pos)
{
    std::vector<double> vect(3);
    vect[0] = pos[0];
    vect[1] = pos[1];
    vect[2] = pos[2];
    return vect;
}

Vector3 op_space_control::GetVector3(const Vector& vect)
{
    Vector3 pos;
    pos[0] = vect[0];
    pos[1] = vect[1];
    pos[2] = vect[2];
    return pos;
}

float* op_space_control::GetGlVector3(const Math3D::Vector3& pos)
{
    float* vect = new float[3];
    vect[0] = pos[0];
    vect[1] = pos[1];
    vect[2] = pos[2];
    return vect;
}

Vector op_space_control::GetVector(const Math3D::Vector3& pos)
{
    Vector vect(3);
    vect[0] = pos[0];
    vect[1] = pos[1];
    vect[2] = pos[2];
    return vect;
}

void op_space_control::PushRotationToVector( const Matrix3& R, Vector& x )
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

void op_space_control::PopRotationFromVector( Vector& x, Matrix3& R )
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

void op_space_control::PushPosToVector( const Vector3& p, Vector& x )
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

void op_space_control::PopPosFromVector( Vector& x, Vector3& p )
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

void op_space_control::PushFrameToVector( const Frame3D& T, Vector& x )
{
    op_space_control::PushRotationToVector( T.R, x );
    op_space_control::PushPosToVector( T.t, x );
}

void op_space_control::PopFrameFromVector( Vector& x, Frame3D& T )
{
    op_space_control::PopPosFromVector( x, T.t );
    op_space_control::PopRotationFromVector( x, T.R );
}

Vector op_space_control::GetPushedFrame( const Frame3D& T )
{
    Vector vect;
    op_space_control::PushFrameToVector(T,vect);
    return vect;
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

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

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

double angle(const Matrix3& R)
{
    double ctheta = (R.trace() - 1.0)*0.5;
    return std::acos(std::max(std::min(ctheta,1.0),-1.0));
}

//! Returns the moment w (exponential map) representation of R such
//! that e^[w] = R. Equivalent to axis-angle representation with
//! w/||w||=axis; ||w||=angle;
Vector3 Moment(const Matrix3& R)
{
    double theta = angle(R);

    if( std::abs(theta-M_PI) < 1e-5 )
    {
        //can't do normal version because the scale factor reaches a singularity
        double x2=(R[0]+1.)*0.5;
        double y2=(R[4]+1.)*0.5;
        double z2=(R[8]+1.)*0.5;
        if( x2 < 0) {
            assert(x2>-1e-5);
            x2=0;
        }
        if( y2 < 0 ) {
            assert(y2>-1e-5);
            y2=0;
        }
        if( z2 < 0 ) {
            assert(z2>-1e-5);
            z2=0;
        }

        double x = M_PI*std::sqrt(x2);
        double y = M_PI*std::sqrt(y2);
        double z = M_PI*std::sqrt(z2);
        //determined up to sign changes, we know r12=2xy,r13=2xz,r23=2yz
        double xy=R[3];
        double xz=R[6];
        double yz=R[7];

        if(x > y) {
            if(x > z) {
                //x is largest
                if(xy < 0) y=-y;
                if(xz < 0) z=-z;
            }
            else {
                //z is largest
                if(yz < 0) y=-y;
                if(xz < 0) x=-x;
            }
        }
        else {
            if(y > z) {
                //y is largest
                if(xy < 0) x=-x;
                if(yz < 0) z=-z;
            }
            else {
                //z is largest
                if(yz < 0) y=-y;
                if(xz < 0) x=-x;
            }
        }
        return Vector3(x,y,z);
    }

    //normal
    double scale = 0.5;
    if( std::abs(theta) > 1e-5 )
        scale = 0.5*theta/std::sin(theta);
    double x = (R[3+2]-R[6+1]) * scale;
    double y = (R[6+0]-R[0+2]) * scale;
    double z = (R[0+1]-R[3+0]) * scale;
    return Vector3(x,y,z);
}

Vector3 op_space_control::Error( const Matrix3& R1, const Matrix3& R2 )
{
    Matrix3 R2inv;
    R2inv.setInverse( R2 );
    return Moment( R1 * R2inv );
}

//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------
//--------------------------------------------------------------------

Matrix op_space_control::VStack( const Matrix& mat1, const Matrix& mat2 )
{
    if( mat1.numCols() != mat2.numCols()) {
        cout << "Error in Vstack" << endl;
        return Matrix();
    }

    std::vector<Vector> mattmp;

    for(int i=0;i<mat1.numRows();i++)
        mattmp.push_back( mat1.row(i) );

    for(int i=0;i<mat2.numRows();i++)
        mattmp.push_back( mat1.row(i) );

    Matrix out( mattmp.size(), mat1.numCols() );
    for(int i=0;i<int(mattmp.size());i++)
    {
        Vector row;
        out.getRowRef( i, row );
        row = mattmp[i];
    }

    return out;
}

Vector op_space_control::HStack(const Vector& vec1, const Vector& vec2 )
{
    Vector out( vec1.size() + vec2.size() );

    for(int i=0;i<vec1.size();i++)
        out[i] = vec1[i];

    for(int i=vec1.size();i<(vec1.size()+vec2.size());i++)
        out[i] = vec2[i];

    return out;
}
