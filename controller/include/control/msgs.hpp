#pragma once

#include <cstdint>

namespace control
{

enum class chassis_state : std::uint8_t
{
    relax = 0,
    recover,
    flatten,
    neutral,
    normal,
    offground,
    spin,
    gostair,
    jump
};

struct motor_feedback
{
    float q = 0.0f;
    float dq = 0.0f;
    float tau = 0.0f;
};

struct msg_ins_t
{
    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float total_yaw = 0.0f;
    float gyro_r = 0.0f;
    float gyro_p = 0.0f;
    float gyro_y = 0.0f;
    float accel[3] = {};
};

struct msg_cmd_t
{
    float x = 0.0f;
    float v = 0.0f;
    float len = 0.16f;
    float yaw = 0.0f;
    float dyaw = 0.0f;
    bool move = false;
    bool spin = false;
};

struct msg_ctrl_t
{
    float Tl[2] = {};
    float Tr[2] = {};
    float Twl = 0.0f;
    float Twr = 0.0f;
};

struct msg_motor_cmd_t
{
    float tau[6] = {};
};

struct msg_pendulum_t
{
    float x = 0.0f;
};

struct msg_odometry_t
{
    float x = 0.0f;
    float v = 0.0f;
    float a_z = 0.0f;
};

struct msg_raw_state_t
{
    double time = 0.0;
    float gyro[3] = {};
    float accel[3] = {};
    float quat_gt[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    motor_feedback motors[6] = {};
};

struct input_snapshot_t
{
    bool w = false;
    bool s = false;
    bool a = false;
    bool d = false;
    bool q = false;
    bool e = false;
    bool space = false;
    bool f = false;
};

struct msg_log_t
{
    double time = 0.0;

    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float gyro[3] = {};
    float accel[3] = {};

    float x = 0.0f;
    float v = 0.0f;
    float az = 0.0f;

    float l_len = 0.0f;
    float l_dlen = 0.0f;
    float l_alpha = 0.0f;
    float l_dalpha = 0.0f;
    float l_phi = 0.0f;
    float l_dphi = 0.0f;
    float l_alpha_eq = 0.0f;
    float l_t_hip = 0.0f;
    float l_total_phi = 0.0f;
    float l_n = 0.0f;
    float l_f = 0.0f;
    float l_fs = 0.0f;
    float l_flat = 0.0f;
    float l_neutral = 0.0f;

    float r_len = 0.0f;
    float r_dlen = 0.0f;
    float r_alpha = 0.0f;
    float r_dalpha = 0.0f;
    float r_phi = 0.0f;
    float r_dphi = 0.0f;
    float r_alpha_eq = 0.0f;
    float r_t_hip = 0.0f;
    float r_total_phi = 0.0f;
    float r_n = 0.0f;
    float r_f = 0.0f;
    float r_fs = 0.0f;
    float r_flat = 0.0f;
    float r_neutral = 0.0f;

    float m_q[6] = {};
    float m_dq[6] = {};
    float m_tau[6] = {};

    float cmd_tau[6] = {};

    float Tl0 = 0.0f;
    float Tl1 = 0.0f;
    float Tr0 = 0.0f;
    float Tr1 = 0.0f;
    float Twl = 0.0f;
    float Twr = 0.0f;

    std::uint8_t fsm = 0;
    float n_total = 0.0f;

    float cmd_v = 0.0f;
    float cmd_len = 0.0f;
    bool cmd_move = false;
};

}  // namespace control
