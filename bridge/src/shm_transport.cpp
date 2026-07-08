#include "bridge/shm_transport.hpp"

namespace bridge
{

bool shm_io::Init(const std::string& ipc_prefix, std::string& error)
{
    Reset();
    state_out_ = robot_ipc::ShmChannel<robot_msgs::LowState>::open_server(ipc_prefix + "_lowstate");
    cmd_in_ = robot_ipc::ShmChannel<robot_msgs::LowCmd>::open_server(ipc_prefix + "_lowcmd");

    if (!valid())
    {
        error = "failed to create shm channels with prefix: " + ipc_prefix;
        return false;
    }
    return true;
}

void shm_io::Reset()
{
    have_cmd_ = false;
    last_cmd_ = robot_msgs::LowCmd{};
}

void shm_io::PushState(const robot_msgs::LowState& state)
{
    if (valid())
    {
        state_out_.write(state);
    }
}

bool shm_io::PullCommand(robot_msgs::LowCmd& cmd)
{
    if (!valid())
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

}  // namespace bridge
