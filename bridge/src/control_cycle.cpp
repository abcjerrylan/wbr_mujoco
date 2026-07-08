#include "bridge/control_cycle.hpp"

#include <algorithm>

namespace bridge
{

void control_loop::Init(IRobotIO& backend, IIOTransport& transport, int decimation)
{
    backend_ = &backend;
    transport_ = &transport;
    decimation_ = std::max(1, decimation);
    step_counter_ = 0;
    enabled_ = true;
}

void control_loop::Reset()
{
    step_counter_ = 0;
    state_ = robot_msgs::LowState{};
    cmd_ = robot_msgs::LowCmd{};
    if (transport_ != nullptr)
    {
        transport_->Reset();
    }
}

void control_loop::Step()
{
    if (!enabled_ || backend_ == nullptr || transport_ == nullptr)
    {
        return;
    }

    ++step_counter_;
    if (step_counter_ % decimation_ != 0)
    {
        return;
    }

    ExchangeOnce(*backend_, *transport_, state_, cmd_);
}

}  // namespace bridge
