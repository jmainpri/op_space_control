#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <string>
#include "math/vector.h"

namespace op_space_control
{
class Trajectory
{
public:
    Trajectory();

    //! Parse a trajectory in the robot sim format
    bool parse_robotsim(std::string file);

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
