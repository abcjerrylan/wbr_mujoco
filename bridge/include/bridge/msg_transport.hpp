#pragma once

#include "bridge/io_transport.hpp"

#include <msg/msg.hpp>
#include <robot_msgs/types.hpp>

namespace bridge
{

class msg_io : public IIOTransport
{
public:
    bool Init();
    void Reset() override;

    void PushState(const robot_msgs::LowState& state) override;
    bool PullCommand(robot_msgs::LowCmd& cmd) override;

    msg::topic* state_topic() const { return state_topic_; }
    msg::topic* cmd_topic() const { return cmd_topic_; }

private:
    msg::topic* state_topic_ = nullptr;
    msg::topic* cmd_topic_ = nullptr;
    msg::subscriber cmd_sub_{msg::subscriber::invalid()};
    bool have_cmd_ = false;
    robot_msgs::LowCmd last_cmd_{};
};

}  // namespace bridge
