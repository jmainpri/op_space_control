#ifndef OP_SPACE_UTILS_H
#define OP_SPACE_UTILS_H

#include "math/vector.h"
#include "math/matrix.h"
#include "math3d/primitives.h"
#include "robotics/Frame.h"
#include <vector>

namespace op_space_control
{
std::vector<int> GetStdVector(int value);
std::vector<double> GetStdVector(double value);
std::vector<double> GetStdVector(const Math3D::Vector3& pos);

Math3D::Vector3 GetVector3(const Math3D::Vector& vect);
float* GetGlVector3(const Math3D::Vector3& pos);
Vector GetVector(const Math3D::Vector3& pos);

void PushRotationToVector( const Math3D::Matrix3& R, Math::Vector& x );
void PopRotationFromVector( Math::Vector& x, Math3D::Matrix3& R );

void PushPosToVector( const Math3D::Vector3& p, Math::Vector& x );
void PopPosFromVector( Math::Vector& x, Math3D::Vector3& p );

void PushFrameToVector( const Frame3D& T, Math::Vector& x );
void PopFrameFromVector( Math::Vector& x, Frame3D& T );

Math::Vector GetPushedFrame( const Frame3D& T );

Vector3 Error(const Matrix3& R1, const Matrix3& R2);

Matrix VStack(const Matrix& mat1, const Matrix& mat2 );
Vector HStack(const Vector& vec1, const Vector& vec2 );
}

#endif // UTILS_H
