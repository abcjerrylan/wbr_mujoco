#pragma once

#include "mujoco_interface/sim_control.hpp"
#include "bridge/mj_adapter.hpp"
#include "bridge/shm_bridge.hpp"
#include "bridge/sim_config.hpp"

#include <memory>
#include <string>

namespace bridge
{

class app_control : public mujoco_interface::sim_control
{
public:
    void set_ipc_prefix_override(std::string prefix) { ipc_prefix_override_ = std::move(prefix); }
    void set_print_state_hz(double hz) { print_state_hz_ = hz; }

    bool init(mujoco_interface::robot_interface& robot, const std::string& config_path,
              std::string& error) override;
    void reset(mujoco_interface::robot_interface& robot) override;
    void step(mujoco_interface::robot_interface& robot) override;
    [[nodiscard]] bool enabled() const override { return enabled_; }

private:
    void maybe_print_state(const mujoco_interface::robot_interface& robot);

    mujoco_interface::robot_interface* robot_ = nullptr;
    std::unique_ptr<mj_adapter> adapter_;
    shm_bridge bridge_;
    sim_opts options_{};
    std::string ipc_prefix_override_;
    double print_state_hz_ = 0.0;
    int print_every_steps_ = 0;
    int step_counter_ = 0;
    bool enabled_ = false;
};

}  // namespace bridge
