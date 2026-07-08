#pragma once

#include "bridge/mj_adapter.hpp"

#include <robot_ipc/channel.hpp>
#include <robot_msgs/types.hpp>

#include <string>

namespace bridge
{

class shm_bridge
{
public:
    bool Init(mj_adapter& adapter, const std::string& ipc_prefix, int decimation, std::string& error);
    void Reset();
    void Step();

    [[nodiscard]] bool enabled() const { return enabled_; }

private:
    bool PullCommand(robot_msgs::LowCmd& cmd);
    void PushState(const robot_msgs::LowState& state);

    mj_adapter* adapter_ = nullptr;
    robot_ipc::ShmChannel<robot_msgs::LowState> state_out_;
    robot_ipc::ShmChannel<robot_msgs::LowCmd> cmd_in_;
    int decimation_ = 1;
    int step_counter_ = 0;
    bool enabled_ = false;
    bool have_cmd_ = false;
    robot_msgs::LowState state_{};
    robot_msgs::LowCmd cmd_{};
    robot_msgs::LowCmd last_cmd_{};
};

}  // namespace bridge
