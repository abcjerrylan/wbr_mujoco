#pragma once

#include "control/msgs.hpp"

namespace control
{

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
    float alpha_eq_coeff[3] = {0.280918f, -1.101757f, 1.232768f};
    float imu_offset_x = 0.0f;
    float control_dt = 0.001f;
    bool force_relax = false;
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
