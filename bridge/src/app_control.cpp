#include "bridge/app_control.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace bridge
{

namespace
{

void print_low_state(const robot_msgs::LowState& state, const mujoco_interface::robot_config& config)
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
    std::fflush(stdout);
}

}  // namespace

bool app_control::init(mujoco_interface::robot_interface& robot, const std::string& config_path,
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
        const double dt = robot.sim_timestep();
        print_every_steps_ = dt > 0.0 ? std::max(1, static_cast<int>(std::lround(1.0 / (print_state_hz_ * dt))))
                                      : 1;
        std::printf("state print enabled: %.1f Hz (every %d sim steps)\n", print_state_hz_, print_every_steps_);
    }

    enabled_ = true;
    return true;
}

void app_control::reset(mujoco_interface::robot_interface& robot)
{
    (void)robot;
    step_counter_ = 0;
    bridge_.Reset();
}

void app_control::step(mujoco_interface::robot_interface& robot)
{
    (void)robot;
    if (enabled_)
    {
        bridge_.Step();
        maybe_print_state(robot);
    }
}

void app_control::maybe_print_state(const mujoco_interface::robot_interface& robot)
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
    adapter_->read_state(state);
    print_low_state(state, robot.config());
}

}  // namespace bridge
