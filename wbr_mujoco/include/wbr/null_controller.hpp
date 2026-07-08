#pragma once

#include "wbr/controller.hpp"

namespace wbr
{

class NullController final : public RobotController
{
public:
    RobotAction Step(const RobotInterface &robot, const RobotObservation &obs) override;
};

}  // namespace wbr
