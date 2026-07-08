#pragma once

#include "mujoco_interface/sim_control.hpp"
#include "bridge/mj_adapter.hpp"
#include "bridge/shm_bridge.hpp"
#include "bridge/sim_config.hpp"

#include <memory>
#include <string>

namespace bridge
{

class app_control : public mujoco_interface::ISimControl
{
public:
    void SetIpcPrefixOverride(std::string prefix) { ipc_prefix_override_ = std::move(prefix); }

    bool Init(mujoco_interface::RobotInterface& robot, const std::string& config_path,
              std::string& error) override;
    void Reset(mujoco_interface::RobotInterface& robot) override;
    void Step(mujoco_interface::RobotInterface& robot) override;
    [[nodiscard]] bool enabled() const override { return enabled_; }

private:
    mujoco_interface::RobotInterface* robot_ = nullptr;
    std::unique_ptr<mj_adapter> adapter_;
    shm_bridge bridge_;
    sim_opts options_{};
    std::string ipc_prefix_override_;
    bool enabled_ = false;
};

}  // namespace bridge
