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
 *      Kris Hauser
 */

#ifndef URDFPARSER_H
#define URDFPARSER_H

#include "robotics/RobotDynamics3D.h"

#include <boost/shared_ptr.hpp>

namespace OpSpaceControl
{

//! Determines the effects of an actuator on the robot configuration
struct RobotJointDriver
{
  /* Types
  Normal: normal
  Affine: scaling/offset of a single control to multiple outputs linkIndices stores the mapping
  Translation: controls are a direct force applied to the body. linkIndices[0] is the "driver" link,
  linkIndices[1] is the affected link
  Rotation: controls are a direct moment applied to the body linkIndices[0] is the "driver" link,
  linkIndices[1] is the affected link
  Custom: in the future will hold more sophisticated mappings
  */
    enum Type { Normal, Affine, Translation, Rotation, Custom };

    int NumControls() const;  //number of input controls
    int NumLinks() const;
    bool Affects(int link) const;

    Type type;
    std::vector<int> linkIndices;
    Real qmin,qmax;           //min/max values
    Real vmin,vmax;           //min/max velocities
    Real amin,amax;           //min/max accelerations
    Real tmin,tmax;           //min/max torques
    std::vector<Real> affScaling;  //for Affine joints
    std::vector<Real> affOffset;   //for Affine joints
    Real servoP,servoI,servoD;  //servo parameters
    Real dryFriction;           //friction coefficient
};

//! Additional joint properties
struct RobotJoint
{
    /* Types:
     Weld: completely fixed to the parent
     Normal: regular fixed axis joint with finite range
     Spin: infinitely spinnable rotational joint
     Floating: free-floating base
     FloatingPlanar: a free-floating 2D base
     BallAndSocket: ball and socket joint
     Closed: a closed chain loop
  */
    enum Type { Weld, Normal, Spin, Floating, FloatingPlanar, BallAndSocket, Closed };
    //  UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    Type type;
    ///The affected link.  For Floating/BallAndSocket joints, the last
    ///link in the chain.  For Closed joints, the `free' link
    int linkIndex;
    ///For Floating/BallAndSocket joints, the first link in the chain.
    ///For Closed joints, the attachment link
    int baseIndex;
    ///For closed joints
    Vector3 localPt,attachmentPt;
};

//! Loads a robot from a urdf file
class UrdfRobotParser : public RobotDynamics3D
{
public:
    UrdfRobotParser() : RobotDynamics3D() { }

    // Main function of the urdf parser
    bool LoadUrdf( std::string filename );

    std::vector<std::string> GetLinkNames() { return linkNames; }

private:

    // Index link names
    std::vector<std::string> linkNames;

    Vector accMax;   //conservative acceleration limits, used by DynamicPath
    std::vector<RobotJoint> joints;
    std::vector<RobotJointDriver> drivers;
    std::vector<std::string> driverNames;
    Matrix lipschitzMatrix;
};

}

#endif // URDFPARSER_H
