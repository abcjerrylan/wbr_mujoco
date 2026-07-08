#pragma once

#include <robot_msgs/types.hpp>

#include <cstdint>

namespace bridge
{

class IRobotIO
{
public:
    virtual ~IRobotIO() = default;

    virtual void ReadState(robot_msgs::LowState& state) const = 0;
    virtual void WriteCommand(const robot_msgs::LowCmd& cmd) = 0;

    [[nodiscard]] virtual std::uint32_t NumMotors() const = 0;
};

inline void Runcontrol_loop(IRobotIO& backend, robot_msgs::LowState& state, robot_msgs::LowCmd& cmd,
                            bool have_command)
{
    if (have_command)
    {
        backend.WriteCommand(cmd);
    }
    backend.ReadState(state);
}

}  // namespace bridge
