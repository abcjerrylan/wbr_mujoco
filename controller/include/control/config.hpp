#pragma once

#include "control/msgs.hpp"

#include <cstdint>

namespace control
{

enum class pid_mode : std::uint8_t
{
    position = 0x01,
    dvel = 0x02
};

struct pid_params
{
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float max_out = 0.0f;
    float max_i_out = 0.0f;
    pid_mode mode = pid_mode::position;
};

struct phi_state_pid
{
    float kp = 0.0f;
    float kd = 0.0f;
    float slope = 0.005f;
};

struct leg_pid_config
{
    pid_params len{5000.0f, 0.0f, -1500.0f, 125.0f, 0.0f, pid_mode::dvel};
    pid_params phi{0.7f, 0.0f, 1.4f, 40.0f, 0.005f};
};

struct fsm_pid_config
{
    pid_params roll{0.7f, 0.0001f, 1.4f, 0.05f, 0.005f};
    pid_params len_balance{2000.0f, 0.0f, -400.0f, 125.0f, 0.0f};
    phi_state_pid recover{20.0f, 10.0f, 0.005f};
    float recover_kick_torque = 35.0f;
    phi_state_pid flatten_high{10.0f, 5.0f, 0.01f};
    phi_state_pid flatten_low{5.0f, 10.0f, 0.002f};
    phi_state_pid neutral{30.0f, 10.0f, 0.003f};
};

struct chassis_config
{
    float l1 = 0.220f;
    float l2 = 0.260f;
    float lmin = 0.16f;
    float lmid = 0.24f;
    float lmax = 0.36f;
    float lfly = 0.22f;
    float lswitch = 0.24f;
    float rwheel = 0.06f;
    float mwheel = 0.21f;
    float tk_wheel = 1400.0f;
    float thip_max = 40.0f;
    float twheel_max = 10.0f;
    float gff = 67.865f;
    float fspring = 450.0f;
    float dspring1 = 0.03f;
    float dspring2 = 0.05f;
    float ang_spring = 0.2164f;
    float alpha_eq_coeff[3] = {0.280918f, -1.101757f, 1.232768f};
    float imu_offset_x = 0.0f;
    float control_dt = 0.001f;
    bool force_relax = false;
    leg_pid_config leg_pid{};
    fsm_pid_config fsm_pid{};

    // Motor mechanical zero offsets (rad), indexed like msg_raw_state_t::motors[]
    // Order: 0 ljoint1, 1 ljoint4, 2 lwheel, 3 rjoint1, 4 rjoint4, 5 rwheel
    float motor_zero_rad[6] = {};
};

inline constexpr chassis_config k_default_chassis{};

enum class imu_mode : std::uint8_t
{
    bypass = 0,
    mahony
};

struct imu_sim_config
{
    float gyro_noise_std = 0.0f;
    float accel_noise_std = 0.0f;
    float gyro_bias[3] = {};
    float lever_arm_x = 0.0f;
};

}  // namespace control
