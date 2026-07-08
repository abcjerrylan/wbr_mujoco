#pragma once

#include "bridge/robot_io.hpp"

#include <robot_msgs/types.hpp>

namespace bridge
{

class IIOTransport
{
public:
    virtual ~IIOTransport() = default;

    virtual void PushState(const robot_msgs::LowState& state) = 0;
    virtual bool PullCommand(robot_msgs::LowCmd& cmd) = 0;

    virtual void Reset() {}
};

inline void ExchangeOnce(IRobotIO& backend, IIOTransport& transport, robot_msgs::LowState& state,
                       robot_msgs::LowCmd& cmd)
{
    const bool have_command = transport.PullCommand(cmd);
    Runcontrol_loop(backend, state, cmd, have_command);
    transport.PushState(state);
}

}  // namespace bridge
