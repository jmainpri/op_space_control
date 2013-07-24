#include "robotsim-parser.h"

using namespace op_space_control;

RobotsimParser::RobotsimParser()
{

}

RobotsimParser::~RobotsimParser()
{

}

std::vector<std::string> RobotsimParser::GetLinkNames()
{
    return linkNames;
}
