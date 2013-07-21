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

    bool parse_robotsim(std::string file);

    void compute_length();
    Math::Vector eval(double t);
    Math::Vector interpolate(const Math::Vector& a, const Math::Vector& b, double u);

    std::vector< std::pair<double,Math::Vector> > milestones_;
    double length_;
};
}

#endif // TRAJECTORY_H
