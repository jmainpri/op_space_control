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

#ifndef OP_SPACE_UTILS_H
#define OP_SPACE_UTILS_H

#include "math/vector.h"
#include "math/matrix.h"
#include "math3d/primitives.h"
#include "robotics/Frame.h"
#include <vector>

//! structure conversions and stacking
namespace OpSpaceControl
{
std::vector<int> GetStdVector(int value);
std::vector<double> GetStdVector(double value);
std::vector<double> GetStdVector(const Math3D::Vector3& pos);

Math3D::Vector3 GetVector3(const Math3D::Vector& vect);
float* GetGlVector3(const Math3D::Vector3& pos);
Vector GetVector(const Math3D::Vector3& pos);

std::vector< std::vector<double> > GetStdMatrix( const Matrix& mat );
Matrix GetKrisMatrix( const std::vector< std::vector<double> >& mat );

void PushRotationToVector( const Math3D::Matrix3& R, Math::Vector& x );
void PopRotationFromVector( Math::Vector& x, Math3D::Matrix3& R );

void PushPosToVector( const Math3D::Vector3& p, Math::Vector& x );
void PopPosFromVector( Math::Vector& x, Math3D::Vector3& p );

void PushFrameToVector( const Frame3D& T, Math::Vector& x );
void PopFrameFromVector( Math::Vector& x, Frame3D& T );

Math::Vector GetPushedFrame( const Frame3D& T );

Vector3 Error(const Matrix3& R1, const Matrix3& R2);

Math::Matrix VStack(const Math::Matrix& mat1, const Math::Matrix& mat2 );
Math::Vector HStack(const Math::Vector& vec1, const Math::Vector& vec2 );

}

void g3d_draw_solid_sphere(double x_, double y_, double z_, double radius, int nbSegments);
void g3d_draw_frame( const Frame3D& T, double length);

#endif // UTILS_H
