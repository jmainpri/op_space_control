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

#ifndef ROBOTSIMPARSER_H
#define ROBOTSIMPARSER_H

#include "Modeling/Robot.h"

namespace OpSpaceControl
{

//! derives the robotsim structure to get access
//! to the robotsim parser
//! WARNING this is only available when directly linked
//! to RobotSim
class RobotsimParser : public Robot
{
public:
    RobotsimParser();
    ~RobotsimParser();

    std::vector<std::string> GetLinkNames();
};

}

#endif // ROBOTSIMPARSER_H