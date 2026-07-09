#include "bridge/app_control.hpp"

#include "mujoco_interface/input_hub.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace bridge
{

namespace
{

void print_low_state(const robot_msgs::LowState& state, const mujoco_interface::robot::config& config)
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

// Trial: WASD -> first four leg joints in wbr.yaml order (indices 0,1,3,4).
void apply_keyboard_joint_trial(const mujoco_interface::input::snapshot& input, mj_adapter& adapter,
                                std::uint32_t num_motors)
{
    namespace in = mujoco_interface::input;

    struct binding
    {
        in::key key;
        std::uint32_t motor;
        float tau;
    };

    const binding bindings[] = {
        {in::key::w, 0, 2.0f},  // ljoint1
        {in::key::a, 1, 2.0f},  // ljoint4
        {in::key::s, 3, 2.0f},  // rjoint1
        {in::key::d, 4, 2.0f},  // rjoint4
    };

    bool active = false;
    for (const auto& b : bindings)
    {
        if (input.keyboard.is_down(b.key))
        {
            active = true;
            break;
        }
    }
    if (!active)
    {
        return;
    }

    robot_msgs::LowCmd cmd{};
    cmd.num_motors = num_motors;
    for (const auto& b : bindings)
    {
        if (b.motor >= num_motors || !input.keyboard.is_down(b.key))
        {
            continue;
        }
        cmd.motors[b.motor].mode = robot_msgs::kMotorModeTorque;
        cmd.motors[b.motor].tau = b.tau;
    }
    adapter.write_command(cmd);
}

}  // namespace

bool app_control::init(mujoco_interface::robot::interface& robot, const std::string& config_path,
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

    mujoco_interface::input::hub::instance().capture_control_keys();
    std::printf("keyboard trial: W/A/S/D -> ljoint1/ljoint4/rjoint1/rjoint4 (2 Nm); 2/3/4 = viewer lighting\n");

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

void app_control::reset(mujoco_interface::robot::interface& robot)
{
    (void)robot;
    step_counter_ = 0;
    bridge_.Reset();
}

void app_control::step(mujoco_interface::robot::interface& robot,
                       const mujoco_interface::input::snapshot& input)
{
    if (enabled_)
    {
        bridge_.Step();
        apply_keyboard_joint_trial(input, *adapter_, robot.num_motors());
        maybe_print_state(robot);
    }
}

void app_control::maybe_print_state(const mujoco_interface::robot::interface& robot)
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
    print_low_state(state, robot.robot_config());
}

}  // namespace bridge
