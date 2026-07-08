#pragma once

#include "bridge/control_cycle.hpp"
#include "bridge/robot_io.hpp"
#include "bridge/shm_transport.hpp"

#include <string>

namespace bridge
{

class shm_bridge
{
public:
    bool Init(IRobotIO& robot, const std::string& ipc_prefix, int decimation, std::string& error);
    void Reset();
    void Step();

    [[nodiscard]] bool enabled() const { return cycle_.enabled(); }

private:
    shm_io transport_;
    control_loop cycle_;
};

}  // namespace bridge
