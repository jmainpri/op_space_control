#ifndef DRCHUBO_CONTOLLER_H
#define DRCHUBO_CONTOLLER_H

#include "controller.h"

namespace op_space_control
{
class DRCHubo_contoller
{
public:
    DRCHubo_contoller();

    void init();

private:
    OperationalSpaceController* conroller_;
};
}

#endif // DRCHUBO_CONTOLLER_H
