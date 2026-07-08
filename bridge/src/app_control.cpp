#include "bridge/app_control.hpp"

namespace bridge
{

bool app_control::Init(mujoco_interface::RobotInterface& robot, const std::string& config_path,
                         std::string& error)
{
    robot_ = &robot;
    adapter_ = std::make_unique<mj_adapter>(*robot_);

    if (!load_sim_opts(config_path, options_, error))
    {
        return false;
    }

    const std::string prefix =
        ipc_prefix_override_.empty() ? options_.ipc_prefix : ipc_prefix_override_;

    if (!bridge_.Init(*adapter_, prefix, options_.decimation, error))
    {
        enabled_ = false;
        return false;
    }

    enabled_ = true;
    return true;
}

void app_control::Reset(mujoco_interface::RobotInterface& robot)
{
    (void)robot;
    bridge_.Reset();
}

void app_control::Step(mujoco_interface::RobotInterface& robot)
{
    (void)robot;
    if (enabled_)
    {
        bridge_.Step();
    }
}

}  // namespace bridge
