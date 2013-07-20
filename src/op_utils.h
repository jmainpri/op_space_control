#ifndef OP_SPACE_UTILS_H
#define OP_SPACE_UTILS_H

#include "math/vector.h"
#include "math/matrix.h"
#include "math3d/primitives.h"
#include "robotics/Frame.h"
#include <vector>

std::vector<int> GetStdVector(int value);
std::vector<double> GetStdVector(double value);

std::vector<double> GetStdVector(const Math3D::Vector3& pos);
Math3D::Vector3 GetVector3(const Math3D::Vector& vect);

void PushRotationToVector( const Math3D::Matrix3& R, Math::Vector& x );
void PopRotationFromVector( Math::Vector& x, Math3D::Matrix3& R );

void PushPosToVector( const Math3D::Vector3& p, Math::Vector& x );
void PopPosFromVector( Math::Vector& x, Math3D::Vector3& p );

void PushFrameToVector( const Frame3D& T, Math::Vector& x );
void PopFrameFromVector( Math::Vector& x, Frame3D& T );

Math::Vector GetPushedFrame( const Frame3D& T );

#endif // UTILS_H
