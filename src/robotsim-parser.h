#ifndef ROBOTSIMPARSER_H
#define ROBOTSIMPARSER_H

#include "Modeling/Robot.h"

namespace op_space_control
{

class RobotsimParser : public Robot
{
public:
    RobotsimParser();
    ~RobotsimParser();

    std::string LinkName(int i);
};

}

#endif // ROBOTSIMPARSER_H
