#pragma once

#include <algorithm>
#include <cmath>

namespace control
{

inline constexpr float k_pi = 3.14159265358979f;
inline constexpr float k_pi_x2 = 6.283185307f;
inline constexpr float k_gravity = 9.78f;

inline float clamp(float v, float lo, float hi)
{
    return std::max(lo, std::min(v, hi));
}

inline float loop_clamp(float v, float lo, float hi)
{
    while (v > hi)
    {
        v -= hi - lo;
    }
    while (v < lo)
    {
        v += hi - lo;
    }
    return v;
}

inline void quat_rotate_vec(const float q[4], const float v[3], float out[3])
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float tx = 2.0f * (qy * v[2] - qz * v[1]);
    const float ty = 2.0f * (qz * v[0] - qx * v[2]);
    const float tz = 2.0f * (qx * v[1] - qy * v[0]);

    out[0] = v[0] + qw * tx + (qy * tz - qz * ty);
    out[1] = v[1] + qw * ty + (qz * tx - qx * tz);
    out[2] = v[2] + qw * tz + (qx * ty - qy * tx);
}

inline void quat_to_euler(const float q[4], float& roll, float& pitch, float& yaw)
{
    roll = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch = -std::asin(clamp(2.0f * (q[1] * q[3] - q[0] * q[2]), -1.0f, 1.0f));
    yaw = std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
}

}  // namespace control
