#include "bridge/shm_bridge.hpp"

#include <algorithm>

namespace bridge
{

bool shm_bridge::Init(mj_adapter& adapter, const std::string& ipc_prefix, int decimation, std::string& error)
{
    adapter_ = &adapter;
    decimation_ = std::max(1, decimation);
    step_counter_ = 0;
    enabled_ = false;
    Reset();

    state_out_ = robot_ipc::ShmChannel<robot_msgs::LowState>::open_server(ipc_prefix + "_lowstate");
    cmd_in_ = robot_ipc::ShmChannel<robot_msgs::LowCmd>::open_server(ipc_prefix + "_lowcmd");

    if (!state_out_.valid() || !cmd_in_.valid())
    {
        error = "failed to create shm channels with prefix: " + ipc_prefix;
        return false;
    }

    enabled_ = true;
    return true;
}

void shm_bridge::Reset()
{
    step_counter_ = 0;
    have_cmd_ = false;
    state_ = robot_msgs::LowState{};
    cmd_ = robot_msgs::LowCmd{};
    last_cmd_ = robot_msgs::LowCmd{};
}

void shm_bridge::PushState(const robot_msgs::LowState& state)
{
    if (enabled_)
    {
        state_out_.write(state);
    }
}

bool shm_bridge::PullCommand(robot_msgs::LowCmd& cmd)
{
    if (!enabled_)
    {
        return false;
    }

    robot_msgs::LowCmd incoming{};
    if (cmd_in_.read(incoming, false) == robot_ipc::channel_status::ok)
    {
        last_cmd_ = incoming;
        have_cmd_ = true;
    }

    if (have_cmd_)
    {
        cmd = last_cmd_;
        return true;
    }
    return false;
}

void shm_bridge::Step()
{
    if (!enabled_ || adapter_ == nullptr)
    {
        return;
    }

    ++step_counter_;
    if (step_counter_ % decimation_ != 0)
    {
        return;
    }

    const bool have_command = PullCommand(cmd_);
    if (have_command)
    {
        adapter_->WriteCommand(cmd_);
    }
    adapter_->ReadState(state_);
    PushState(state_);
}

}  // namespace bridge
