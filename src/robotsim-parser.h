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

    std::vector<std::string> GetLinkNames();
};

}

#endif // ROBOTSIMPARSER_H
