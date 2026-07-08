#include "bridge/app_control.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace bridge
{

namespace
{

void PrintLowState(const robot_msgs::LowState& state, const mujoco_interface::RobotConfig& config)
{
    std::printf("[t=%.3f] imu quat=(%.3f %.3f %.3f %.3f) gyro=(%.3f %.3f %.3f) accel=(%.3f %.3f %.3f)\n",
                state.time, state.imu.quat[0], state.imu.quat[1], state.imu.quat[2], state.imu.quat[3],
                state.imu.gyro[0], state.imu.gyro[1], state.imu.gyro[2], state.imu.accel[0], state.imu.accel[1],
                state.imu.accel[2]);

    for (std::uint32_t i = 0; i < state.num_motors && i < config.motors.size(); ++i)
    {
        const char* name = config.motors[i].name.c_str();
        const auto& m = state.motors[i];
        std::printf("  %s: q=%.4f dq=%.4f tau=%.4f\n", name, m.q, m.dq, m.tau_est);
    }
}

}  // namespace

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

    if (print_state_hz_ > 0.0)
    {
        const double dt = robot.SimTimestep();
        print_every_steps_ = dt > 0.0 ? std::max(1, static_cast<int>(std::lround(1.0 / (print_state_hz_ * dt))))
                                      : 1;
        std::printf("state print enabled: %.1f Hz (every %d sim steps)\n", print_state_hz_, print_every_steps_);
    }

    enabled_ = true;
    return true;
}

void app_control::Reset(mujoco_interface::RobotInterface& robot)
{
    (void)robot;
    step_counter_ = 0;
    bridge_.Reset();
}

void app_control::Step(mujoco_interface::RobotInterface& robot)
{
    (void)robot;
    if (enabled_)
    {
        bridge_.Step();
        MaybePrintState(robot);
    }
}

void app_control::MaybePrintState(const mujoco_interface::RobotInterface& robot)
{
    if (print_every_steps_ <= 0)
    {
        return;
    }

    ++step_counter_;
    if (step_counter_ % print_every_steps_ != 0)
    {
        return;
    }

    robot_msgs::LowState state{};
    adapter_->ReadState(state);
    PrintLowState(state, robot.config());
}

}  // namespace bridge
