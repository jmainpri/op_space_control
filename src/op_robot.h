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

#ifndef OP_ROBOT_H
#define OP_ROBOT_H

#include <vector>
#include <string>

class RobotLink3D;
class RobotDynamics3D;

namespace OpSpaceControl
{

typedef std::vector<double> OpVect;
typedef std::vector< std::vector<double> > OpMatrix;

// TODO make abstract for link and robot
// use them through out the project for ease of porting
class OpLink
{
public:
    OpLink(RobotLink3D* link) : link_(link) {  }

private:
    RobotLink3D* link_;
};

class OpRobot
{
public:
    OpRobot(RobotDynamics3D* robot);

    int GetNumLinks() { return 0; }
    void SetAndUpdate( const OpVect& q ) { }
    OpLink* GetLinkByName( const std::string& name) { return NULL; }

private:
    RobotDynamics3D* robot_;
};

}

#endif // OP_ROBOT_H
