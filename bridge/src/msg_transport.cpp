#include "bridge/msg_transport.hpp"

namespace bridge
{

bool msg_io::Init()
{
    Reset();
    state_topic_ = msg::create<robot_msgs::LowState>();
    cmd_topic_ = msg::create<robot_msgs::LowCmd>();
    if (state_topic_ == nullptr || cmd_topic_ == nullptr)
    {
        return false;
    }
    cmd_sub_ = msg::subscribe(cmd_topic_);
    return cmd_sub_.valid();
}

void msg_io::Reset()
{
    have_cmd_ = false;
    last_cmd_ = robot_msgs::LowCmd{};
}

void msg_io::PushState(const robot_msgs::LowState& state)
{
    if (state_topic_ != nullptr)
    {
        msg::publish(state_topic_, state);
    }
}

bool msg_io::PullCommand(robot_msgs::LowCmd& cmd)
{
    if (cmd_sub_.valid() && msg::available(cmd_sub_))
    {
        robot_msgs::LowCmd incoming{};
        if (msg::read(cmd_sub_, incoming) == msg::status::ok)
        {
            last_cmd_ = incoming;
            have_cmd_ = true;
        }
    }

    if (have_cmd_)
    {
        cmd = last_cmd_;
        return true;
    }
    return false;
}

}  // namespace bridge
