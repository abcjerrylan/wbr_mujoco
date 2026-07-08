#pragma once

#include "bridge/io_transport.hpp"

#include <robot_ipc/channel.hpp>
#include <robot_msgs/types.hpp>

#include <string>

namespace bridge
{

class shm_io : public IIOTransport
{
public:
    bool Init(const std::string& ipc_prefix, std::string& error);
    void Reset() override;

    void PushState(const robot_msgs::LowState& state) override;
    bool PullCommand(robot_msgs::LowCmd& cmd) override;

    [[nodiscard]] bool valid() const { return state_out_.valid() && cmd_in_.valid(); }

private:
    robot_ipc::ShmChannel<robot_msgs::LowState> state_out_;
    robot_ipc::ShmChannel<robot_msgs::LowCmd> cmd_in_;
    bool have_cmd_ = false;
    robot_msgs::LowCmd last_cmd_{};
};

}  // namespace bridge
