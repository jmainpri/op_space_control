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

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <string>
#include "math/vector.h"

namespace OpSpaceControl
{

//! Simple class that interpolates linearly
//! between a set of milestones
class Trajectory
{
public:
    Trajectory();

    //! Parse a trajectory in the RobotSim format
    //! i.e. one milestone -> (time, q) per line
    bool parse(std::string file);

    //! Compute the total length of the trajectory
    void compute_length();

    //! Evaluate the configuration at time t
    Math::Vector eval(double t);

    //! Configuration along the trajectory
    std::vector< std::pair<double,Math::Vector> > milestones_;

    //! Total of the trajectory
    double length_;

private:
    //! interpolate linearly between two configuration
    Math::Vector interpolate(const Math::Vector& a, const Math::Vector& b, double u);
};
}

#endif // TRAJECTORY_H
