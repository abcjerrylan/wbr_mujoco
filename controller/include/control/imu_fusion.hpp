#pragma once

// Portable 6-axis IMU attitude fusion (Mahony complementary filter).
// Dependencies: <cmath> only — suitable for MCU firmware (copy this header).
// Frame: body-fixed IMU, quaternion maps body -> world (w, x, y, z).
// Accel at rest should read specific force opposite gravity (e.g. +Z when upright).

#include <cmath>

namespace control
{

struct imu_vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct imu_attitude_t
{
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float total_yaw = 0.0f;
};

struct mahony_config
{
    float kp = 2.0f;
    float ki = 0.0f;
    float accel_norm_min = 1e-3f;
};

class mahony_filter
{
public:
    imu_attitude_t attitude{};

    void reset()
    {
        attitude = {};
        integral_ = {};
        prev_yaw_ = 0.0f;
        yaw_round_count_ = 0;
        initialized_ = false;
    }

    void configure(const mahony_config& cfg) { cfg_ = cfg; }

    void set_kp(float kp) { cfg_.kp = kp; }

    void set_ki(float ki) { cfg_.ki = ki; }

    bool init_from_accel(float ax, float ay, float az)
    {
        if (!quat_from_accel(ax, ay, az, attitude.q))
        {
            return false;
        }
        update_euler();
        initialized_ = true;
        return true;
    }

    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
    {
        if (!initialized_)
        {
            if (!init_from_accel(ax, ay, az))
            {
                return;
            }
        }

        if (cfg_.ki > 0.0f)
        {
            integral_.x += cfg_.ki * gx * dt;
            integral_.y += cfg_.ki * gy * dt;
            integral_.z += cfg_.ki * gz * dt;
        }

        float gx_c = gx - integral_.x;
        float gy_c = gy - integral_.y;
        float gz_c = gz - integral_.z;

        const float an = std::sqrt(ax * ax + ay * ay + az * az);
        if (an > cfg_.accel_norm_min)
        {
            const float axn = ax / an;
            const float ayn = ay / an;
            const float azn = az / an;

            imu_vec3 g{};
            gravity_body_from_quat(attitude.q, g);

            const float ex = ayn * g.z - azn * g.y;
            const float ey = azn * g.x - axn * g.z;
            const float ez = axn * g.y - ayn * g.x;

            gx_c += cfg_.kp * ex;
            gy_c += cfg_.kp * ey;
            gz_c += cfg_.kp * ez;
        }

        const float half_dt = 0.5f * dt;
        float* q = attitude.q;
        q[0] += (-q[1] * gx_c - q[2] * gy_c - q[3] * gz_c) * half_dt;
        q[1] += (q[0] * gx_c + q[2] * gz_c - q[3] * gy_c) * half_dt;
        q[2] += (q[0] * gy_c - q[1] * gz_c + q[3] * gx_c) * half_dt;
        q[3] += (q[0] * gz_c + q[1] * gy_c - q[2] * gx_c) * half_dt;
        quat_normalize(q);
        update_euler();
    }

private:
    static float clamp(float v, float lo, float hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    static void quat_normalize(float q[4])
    {
        const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n > 1e-9f)
        {
            q[0] /= n;
            q[1] /= n;
            q[2] /= n;
            q[3] /= n;
        }
    }

    static void gravity_body_from_quat(const float q[4], imu_vec3& out)
    {
        out.x = 2.0f * (q[1] * q[3] - q[0] * q[2]);
        out.y = 2.0f * (q[0] * q[1] + q[2] * q[3]);
        out.z = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    }

    static bool quat_from_accel(float ax, float ay, float az, float q[4])
    {
        const float n = std::sqrt(ax * ax + ay * ay + az * az);
        if (n < 1e-3f)
        {
            return false;
        }

        const float axn = ax / n;
        const float ayn = ay / n;
        const float azn = az / n;
        const float roll = std::atan2(ayn, azn);
        const float pitch = std::atan2(-axn, std::sqrt(ayn * ayn + azn * azn));

        const float cr = std::cos(roll * 0.5f);
        const float sr = std::sin(roll * 0.5f);
        const float cp = std::cos(pitch * 0.5f);
        const float sp = std::sin(pitch * 0.5f);

        q[0] = cr * cp;
        q[1] = sr * cp;
        q[2] = cr * sp;
        q[3] = sr * sp;
        quat_normalize(q);
        return true;
    }

    static void quat_to_euler(const float q[4], float& roll, float& pitch, float& yaw)
    {
        roll = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
        pitch = -std::asin(clamp(2.0f * (q[1] * q[3] - q[0] * q[2]), -1.0f, 1.0f));
        yaw = std::atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    }

    void update_euler()
    {
        quat_to_euler(attitude.q, attitude.roll, attitude.pitch, attitude.yaw);

        constexpr float k_pi = 3.14159265358979f;
        constexpr float k_pi_x2 = 6.283185307f;
        if (attitude.yaw - prev_yaw_ > k_pi)
        {
            --yaw_round_count_;
        }
        else if (attitude.yaw - prev_yaw_ < -k_pi)
        {
            ++yaw_round_count_;
        }
        prev_yaw_ = attitude.yaw;
        attitude.total_yaw = k_pi_x2 * static_cast<float>(yaw_round_count_) + attitude.yaw;
    }

    mahony_config cfg_{};
    imu_vec3 integral_{};
    float prev_yaw_ = 0.0f;
    int yaw_round_count_ = 0;
    bool initialized_ = false;
};

}  // namespace control
