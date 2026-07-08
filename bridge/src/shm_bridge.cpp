#include "bridge/shm_bridge.hpp"

namespace bridge
{

bool shm_bridge::Init(IRobotIO& robot, const std::string& ipc_prefix, int decimation, std::string& error)
{
    if (!transport_.Init(ipc_prefix, error))
    {
        return false;
    }
    cycle_.Init(robot, transport_, decimation);
    return true;
}

void shm_bridge::Reset()
{
    transport_.Reset();
    cycle_.Reset();
}

void shm_bridge::Step()
{
    cycle_.Step();
}

}  // namespace bridge
