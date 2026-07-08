#pragma once

#include "bridge/io_transport.hpp"
#include "bridge/robot_io.hpp"

namespace bridge
{

class control_loop
{
public:
    void Init(IRobotIO& backend, IIOTransport& transport, int decimation = 1);
    void Reset();
    void Step();

    [[nodiscard]] bool enabled() const { return enabled_; }

private:
    IRobotIO* backend_ = nullptr;
    IIOTransport* transport_ = nullptr;
    int decimation_ = 1;
    int step_counter_ = 0;
    bool enabled_ = false;
    robot_msgs::LowState state_{};
    robot_msgs::LowCmd cmd_{};
};

}  // namespace bridge
