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

#include "trajectory.h"
#include <fstream>

using namespace OpSpaceControl;

using std::cout;
using std::endl;

Trajectory::Trajectory()
{

}

bool Trajectory::parse( std::string file )
{
    std::ifstream in( file.c_str() /*,std::ios::in*/ );

    if(!in) {
        cout << "Warning, couldn't open file : " << file << endl;
        return false;
    }

    double t;
    Math::Vector x;
    while(in) {
        in >> t >> x;
        if(in) {
            milestones_.push_back(std::make_pair(t,x));
        }
    }
    if(in.bad()) {
        cout << "Error during read of file : " << file << endl;
        return false;
    }
    in.close();
    compute_length();
    return true;
}

void Trajectory::compute_length()
{
    length_ = milestones_.back().first;
}

//! Evaluates the trajectory using piecewise linear
//! interpolation
Math::Vector Trajectory::eval(double t)
{
    if(milestones_.empty()) {
        cout << "Empty trajectory" << endl;
        return Math::Vector(1,0.0);
    }
    if(milestones_.size() == 1)
        return milestones_[0].second;

    if( t > milestones_.back().first )
    {
        return milestones_.back().second;
    }
    int i=0;
    for(int j=0;j<int(milestones_.size());j++)
    {
        if( t < milestones_[j].first )
        {
            i = j; break;
        }
    }

    if( i==0 )
    {
        return milestones_[0].second;
    }

    int p=i-1;
    double u=(t-milestones_[p].first)/(milestones_[i].first-milestones_[p].first);

    //assert u >= 0 and u <= 1 // TODO
    //linear interpolate between milestones[p] and milestones[i]
    return interpolate( milestones_[p].second, milestones_[i].second, u );
}

Math::Vector Trajectory::interpolate(const Math::Vector& a, const Math::Vector& b, double u)
{
    Math::Vector out(a);
    out.madd(b-a,u);
    return out;
}
