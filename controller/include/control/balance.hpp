#pragma once

#include "control/config.hpp"
#include "control/lqr_coeffs.hpp"
#include "control/math.hpp"
#include "control/msgs.hpp"
#include "control/leg.hpp"

#include <cmath>
#include <cstring>

namespace control
{

class lqr_solver
{
public:
    float tout[4] = {};

    lqr_mode mode = lqr_mode::low;

    void update(float llen, float rlen, const float ref_x[10], const float obs_x[10], const chassis_config& cfg)
    {
        llen = clamp(llen, cfg.lmin, cfg.lmax);
        rlen = clamp(rlen, cfg.lmin, cfg.lmax);
        llen = std::round(llen * 100.0f) / 100.0f;
        rlen = std::round(rlen * 100.0f) / 100.0f;

        const float (*table)[6] = k_lqr_low;
        if (mode == lqr_mode::high)
        {
            table = k_lqr_high;
        }
        else if (mode == lqr_mode::spin)
        {
            table = k_lqr_spin;
        }

        float kbuf[40] = {};
        for (int i = 0; i < 40; ++i)
        {
            kbuf[i] = table[i][0] + table[i][1] * llen + table[i][2] * rlen + table[i][3] * llen * llen +
                      table[i][4] * llen * rlen + table[i][5] * rlen * rlen;
        }

        float err[10] = {};
        for (int i = 0; i < 10; ++i)
        {
            err[i] = obs_x[i] - ref_x[i];
        }

        for (int i = 0; i < 4; ++i)
        {
            float temp = 0.0f;
            for (int j = 0; j < 10; ++j)
            {
                temp += kbuf[i * 10 + j] * err[j];
            }
            tout[i] = temp;
        }

        tout[0] = clamp(tout[0], -cfg.twheel_max, cfg.twheel_max);
        tout[1] = clamp(tout[1], -cfg.twheel_max, cfg.twheel_max);
        tout[2] = clamp(tout[2], -cfg.thip_max, cfg.thip_max);
        tout[3] = clamp(tout[3], -cfg.thip_max, cfg.thip_max);
    }
};

class odometry
{
public:
    float x = 0.0f;
    float v = 0.0f;
    float az = 0.0f;

    void reset()
    {
        x = 0.0f;
        v = 0.0f;
        std::memset(x_hat_, 0, sizeof(x_hat_));
        std::memcpy(p_, p_init_, sizeof(p_));
    }

    void update(const float quaternion[4], const float acc[3], float vel_meas, float yaw, float dt)
    {
        float a_body[3] = {acc[0], acc[1], acc[2]};
        float a_world[3] = {};
        quat_rotate_vec(quaternion, a_body, a_world);
        const float a_x = a_world[0] * std::cos(yaw) + a_world[1] * std::sin(yaw);

        az = a_world[2];

        const float z[2] = {vel_meas, a_x};
        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;

        float f[9] = {1.0f, dt, 0.5f * dt2, 0.0f, 1.0f, dt, 0.0f, 0.0f, 1.0f};
        float x_minus[3] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                x_minus[i] += f[i * 3 + j] * x_hat_[j];
            }
        }

        float p_minus[9] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    p_minus[i * 3 + j] += f[i * 3 + k] * p_[k * 3 + j];
                }
            }
        }
        float fp[9] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    fp[i * 3 + j] += p_minus[i * 3 + k] * f[j * 3 + k];
                }
            }
        }
        for (int i = 0; i < 9; ++i)
        {
            p_minus[i] = fp[i] + q_[i];
        }

        const float h[6] = {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        float s[4] = {};
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    const float hp = h[i * 3 + k];
                    for (int m = 0; m < 3; ++m)
                    {
                        s[i * 2 + j] += hp * p_minus[k * 3 + m] * h[j * 3 + m];
                    }
                }
                s[i * 2 + j] += r_[i * 2 + j];
            }
        }

        const float det = s[0] * s[3] - s[1] * s[2];
        const float inv_s[4] = {s[3] / det, -s[1] / det, -s[2] / det, s[0] / det};

        float ph_t[6] = {};
        for (int i = 0; i < 2; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                for (int k = 0; k < 3; ++k)
                {
                    ph_t[i * 3 + j] += p_minus[j * 3 + k] * h[i * 3 + k];
                }
            }
        }

        float k_gain[6] = {};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                k_gain[i * 2 + j] = ph_t[j * 3 + i] * inv_s[j * 2 + j];
            }
        }

        float y[2] = {z[0] - x_minus[1], z[1] - x_minus[2]};
        for (int i = 0; i < 3; ++i)
        {
            x_hat_[i] = x_minus[i];
            for (int j = 0; j < 2; ++j)
            {
                x_hat_[i] += k_gain[i * 2 + j] * y[j];
            }
        }

        x = x_hat_[0];
        v = x_hat_[1];
    }

private:
    float x_hat_[3] = {};
    float p_[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    float p_init_[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    float q_[9] = {0.00025f, 0.00125f, 0.005f, 0.00125f, 0.005f, 0.05f, 0.005f, 0.05f, 0.5f};
    float r_[4] = {0.1f, 0.0f, 0.0f, 50.0f};
};

class command_fusion
{
public:
    void reset(const chassis_config& cfg)
    {
        msg_ = {};
        msg_.len = cfg.lmin;
        move_enabled_ = false;
        space_prev_ = false;
        space_armed_ = false;
        vel_slope_.set_default(0.0f);
        vel_slope_.set_path(0.006f);
        yaw_slope_.set_default(0.0f);
        yaw_slope_.set_path(0.006f);
        len_target_ = cfg.lmin;
        yaw_ref_initialized_ = false;
        yaw_active_prev_ = false;
    }

    void update(const input_snapshot_t& input, const msg_pendulum_t& pendulum, const msg_ins_t& ins,
                const chassis_config& cfg, float dt)
    {
        if (input.w && !input.s)
        {
            vel_slope_.update_val(vel_slope_.value() + 0.0006f);
        }
        else if (input.s && !input.w)
        {
            vel_slope_.update_val(vel_slope_.value() - 0.0006f);
        }
        else
        {
            vel_slope_.update_val(0.0f);
        }

        const bool yaw_active = input.a != input.d;
        if (!yaw_ref_initialized_)
        {
            msg_.yaw = ins.total_yaw;
            yaw_ref_initialized_ = true;
        }

        if (input.a && !input.d)
        {
            yaw_slope_.update_val(yaw_slope_.value() + 0.0006f);
        }
        else if (input.d && !input.a)
        {
            yaw_slope_.update_val(yaw_slope_.value() - 0.0006f);
        }
        else
        {
            yaw_slope_.set_default(0.0f);
            if (yaw_active_prev_)
            {
                msg_.yaw = ins.total_yaw;
            }
        }
        if (input.q)
        {
            len_target_ = cfg.lmin;
        }
        if (input.e)
        {
            len_target_ = cfg.lmid;
        }
        if (input.f)
        {
            len_target_ = cfg.lmax;
        }

        if (!space_armed_)
        {
            if (!input.space)
            {
                space_armed_ = true;
            }
        }
        else if (input.space && !space_prev_)
        {
            move_enabled_ = !move_enabled_;
        }
        space_prev_ = input.space;

        msg_.move = move_enabled_;
        msg_.v = vel_slope_.value();
        msg_.dyaw = yaw_slope_.value();
        msg_.len = len_target_;

        if (std::fabs(msg_.v) < 1e-4f)
        {
            msg_.x = pendulum.x;
        }
        else
        {
            msg_.x += msg_.v * dt;
        }

        if (yaw_active)
        {
            msg_.yaw += msg_.dyaw * dt;
        }
        yaw_active_prev_ = yaw_active;
    }

    const msg_cmd_t& msg() const { return msg_; }

private:
    msg_cmd_t msg_{};
    bool move_enabled_ = false;
    bool space_prev_ = false;
    bool space_armed_ = false;
    bool yaw_ref_initialized_ = false;
    bool yaw_active_prev_ = false;
    slope vel_slope_{0.0f, 0.006f};
    slope yaw_slope_{0.0f, 0.006f};
    float len_target_ = 0.16f;
};

}  // namespace control
