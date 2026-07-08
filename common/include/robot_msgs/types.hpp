#pragma once

// Wire layout matches mujoco_interface::RobotState / RobotCommand (see mujoco_interface/INTERFACE.md).
// Keep in sync when kInterfaceVersion changes.

#include <cstdint>

namespace robot_msgs
{

inline constexpr std::uint8_t kMaxMotors = 32;

inline constexpr std::uint8_t kMotorModeTorque = 0;
inline constexpr std::uint8_t kMotorModePd = 1;

struct MotorState
{
    float q = 0.0f;
    float dq = 0.0f;
    float tau_est = 0.0f;
};

struct MotorCmd
{
    float q = 0.0f;
    float dq = 0.0f;
    float tau = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    std::uint8_t mode = kMotorModeTorque;
    std::uint8_t _pad[3] = {};
};

struct ImuState
{
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    float gyro[3] = {};
    float accel[3] = {};
};

struct LowState
{
    std::uint32_t num_motors = 0;
    double time = 0.0;
    ImuState imu{};
    MotorState motors[kMaxMotors]{};
};

struct LowCmd
{
    std::uint32_t num_motors = 0;
    MotorCmd motors[kMaxMotors]{};
};

}  // namespace robot_msgs
