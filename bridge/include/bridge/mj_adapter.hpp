#pragma once

#include "bridge/msg_convert.hpp"
#include "bridge/robot_io.hpp"

#include <mujoco_interface/robot_interface.hpp>

namespace bridge
{

class mj_adapter : public IRobotIO
{
public:
    explicit mj_adapter(mujoco_interface::RobotInterface& robot) : robot_(&robot) {}

    void ReadState(robot_msgs::LowState& state) const override
    {
        mujoco_interface::RobotState native{};
        robot_->ReadState(native);
        ToRobotMsgs(native, state);
    }

    void WriteCommand(const robot_msgs::LowCmd& cmd) override
    {
        mujoco_interface::RobotCommand native{};
        FromRobotMsgs(cmd, native);
        robot_->WriteCommand(native);
    }

    [[nodiscard]] std::uint32_t NumMotors() const override { return robot_->NumMotors(); }

private:
    mujoco_interface::RobotInterface* robot_ = nullptr;
};

}  // namespace bridge
