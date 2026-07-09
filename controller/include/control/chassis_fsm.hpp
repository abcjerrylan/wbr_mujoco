#pragma once

#include "control/balance.hpp"
#include "control/config.hpp"
#include "control/leg.hpp"
#include "control/math.hpp"
#include "control/msgs.hpp"

#include <cmath>
#include <cstdint>

namespace control
{

struct fsm_inputs
{
    const msg_ins_t& ins;
    const msg_cmd_t& cmd;
    const msg_odometry_t& odom;
    leg_controller& left;
    leg_controller& right;
    float observed_x[10] = {};
    float n_total = 0.0f;
    bool chassis_dead = false;
};

struct fsm_outputs
{
    msg_ctrl_t ctrl{};
    msg_pendulum_t pendulum{};
    msg_motor_cmd_t motor{};
    chassis_state next_state{};
};

class chassis_fsm
{
public:
    void init(const chassis_config& cfg)
    {
        cfg_ = cfg;
        reset();
    }

    void reset()
    {
        state_ = chassis_state::relax;
        air_protect_cnt_ = 0;
        flipover_cnt_ = 0;
        landing_cnt_ = 0;
        flying_ = false;
        going_stair_ = false;
        target_len_ = cfg_.lmin;
        len_slope_.set_default(cfg_.lmin);
        len_slope_.set_asymmetric(0.0002f, 0.0007f);
        roll_pd_ = pid(cfg_.fsm_pid.roll);
        std::memset(ref_x_, 0, sizeof(ref_x_));
    }

    chassis_state state() const { return state_; }

    void step(const fsm_inputs& in, fsm_outputs& out)
    {
        const auto& ll = in.left.link();
        const auto& rl = in.right.link();

        out.next_state = state_;
        out.ctrl = {};
        out.motor = {};
        out.pendulum.x = in.odom.x;
        out.pendulum.v = in.odom.v;
        out.pendulum.len = (ll.len_ + rl.len_) * 0.5f;
        out.pendulum.lalpha = ll.alpha_;
        out.pendulum.ralpha = rl.alpha_;
        out.pendulum.pitch = in.ins.pitch;
        out.pendulum.N = in.n_total;

        const float pitch = in.ins.pitch;
        const float dpitch = in.ins.gyro_p;

        if (!in.cmd.move || cfg_.force_relax)
        {
            state_ = chassis_state::relax;
        }
        else if (in.ins.accel[2] < 0.0f && state_ != chassis_state::recover)
        {
            if (++flipover_cnt_ > 1000)
            {
                in.left.delta_init_ = false;
                in.right.delta_init_ = false;
                state_ = chassis_state::relax;
                flipover_cnt_ = 0;
            }
        }
        else
        {
            flipover_cnt_ = 0;
        }

        float fl[2] = {};
        float fr[2] = {};
        float twl = 0.0f;
        float twr = 0.0f;

        switch (state_)
        {
        case chassis_state::relax:
            step_relax(in, out, pitch, fl, fr, twl, twr);
            break;
        case chassis_state::recover:
            step_recover(in, pitch, fl, fr, twl, twr);
            break;
        case chassis_state::flatten:
            step_flatten(in, out, fl, fr, twl, twr);
            break;
        case chassis_state::neutral:
            step_neutral(in, fl, fr, twl, twr);
            break;
        case chassis_state::normal:
            step_normal(in, out, pitch, fl, fr, twl, twr);
            break;
        case chassis_state::offground:
            step_offground(in, pitch, fl, fr, twl, twr);
            break;
        case chassis_state::spin:
            step_spin(in, fl, fr, twl, twr);
            break;
        default:
            in.left.relax();
            in.right.relax();
            break;
        }

        out.ctrl.Tl[0] = fl[0];
        out.ctrl.Tl[1] = fl[1];
        out.ctrl.Tr[0] = fr[0];
        out.ctrl.Tr[1] = fr[1];
        out.ctrl.Twl = twl;
        out.ctrl.Twr = twr;
        in.left.apply_torque(fl, twl, out.motor.tau[0], out.motor.tau[1], out.motor.tau[2]);
        in.right.apply_torque(fr, twr, out.motor.tau[3], out.motor.tau[4], out.motor.tau[5]);
        out.next_state = state_;
    }

private:
    void step_relax(const fsm_inputs& in, fsm_outputs& out, float pitch, float fl[2], float fr[2], float& twl,
                    float& twr)
    {
        in.left.relax();
        in.right.relax();
        in.left.delta_init_ = false;
        in.right.delta_init_ = false;
        target_len_ = cfg_.lmin;
        len_slope_.set_default(in.left.link().len_);
        out.pendulum.recovered = in.ins.accel[2] >= 0.0f;
        out.pendulum.normal = false;
        if (!cfg_.force_relax && in.cmd.move && !in.chassis_dead)
        {
            state_ = in.ins.accel[2] < 0.0f ? chassis_state::recover : chassis_state::flatten;
        }
        (void)pitch;
        (void)fl;
        (void)fr;
        twl = 0.0f;
        twr = 0.0f;
    }

    void step_recover(const fsm_inputs& in, float pitch, float fl[2], float fr[2], float& twl, float& twr)
    {
        auto& l = in.left;
        auto& r = in.right;
        const bool upright = in.ins.accel[2] > 5.0f;
        const bool pitch_ok = std::fabs(pitch) < 0.35f && std::fabs(in.ins.gyro_p) < 1.0f;
        if (upright && pitch_ok)
        {
            l.relax();
            r.relax();
            state_ = chassis_state::flatten;
            return;
        }

        const auto& rp = cfg_.fsm_pid.recover;
        if (pitch > 0.0f)
        {
            fl[1] = l.phi_control(0.0f, rp.kp, rp.kd, rp.slope, true);
            fr[1] = r.phi_control(0.0f, rp.kp, rp.kd, rp.slope, true);
            if (std::fabs(l.link().phi_) < 0.1f && std::fabs(r.link().phi_) < 0.1f)
            {
                fl[1] = -cfg_.fsm_pid.recover_kick_torque;
                fr[1] = cfg_.fsm_pid.recover_kick_torque;
            }
        }
        else
        {
            fl[1] = l.phi_control(k_pi, rp.kp, rp.kd, rp.slope, false);
            fr[1] = r.phi_control(k_pi, rp.kp, rp.kd, rp.slope, false);
            if (std::fabs(l.link().phi_ - k_pi) < 0.1f && std::fabs(r.link().phi_ - k_pi) < 0.1f)
            {
                fl[1] = cfg_.fsm_pid.recover_kick_torque;
                fr[1] = -cfg_.fsm_pid.recover_kick_torque;
            }
        }
        twl = 0.0f;
        twr = 0.0f;
    }

    void step_flatten(const fsm_inputs& in, fsm_outputs& out, float fl[2], float fr[2], float& twl, float& twr)
    {
        auto& l = in.left;
        auto& r = in.right;
        out.pendulum.normal = true;
        out.pendulum.recovered = true;

        const auto& fh = cfg_.fsm_pid.flatten_high;
        const auto& flw = cfg_.fsm_pid.flatten_low;

        if (l.link().flat_)
        {
            fl[0] = fl[1] = 0.0f;
            l.relax();
            l.delta_init_ = false;
        }
        else if (l.link().phi_ >= k_pi * 0.4f)
        {
            fl[1] = l.phi_control(k_pi, fh.kp, fh.kd, fh.slope, true);
        }
        else
        {
            fl[1] = l.phi_control(k_pi, flw.kp, flw.kd, flw.slope, false);
        }

        if (r.link().flat_)
        {
            fr[0] = fr[1] = 0.0f;
            r.relax();
            r.delta_init_ = false;
        }
        else if (r.link().phi_ >= k_pi * 0.4f)
        {
            fr[1] = r.phi_control(k_pi, fh.kp, fh.kd, fh.slope, true);
        }
        else
        {
            fr[1] = r.phi_control(k_pi, flw.kp, flw.kd, flw.slope, false);
        }

        fl[0] = fr[0] = 0.0f;
        twl = twr = in.cmd.v > 0.0f ? 1.5f : 0.0f;
        if (l.link().flat_ && r.link().flat_)
        {
            state_ = chassis_state::neutral;
        }
    }

    void step_neutral(const fsm_inputs& in, float fl[2], float fr[2], float& twl, float& twr)
    {
        auto& l = in.left;
        auto& r = in.right;
        const auto& ll = l.link();
        const auto& rl = r.link();
        const auto& np = cfg_.fsm_pid.neutral;
        fl[0] = l.len_control(ll.len_ + (cfg_.lmin - ll.len_) * 0.6f) - ll.fs_;
        fr[0] = r.len_control(rl.len_ + (cfg_.lmin - rl.len_) * 0.6f) - rl.fs_;
        fl[1] = l.phi_control(k_pi * 0.5f, np.kp, np.kd, np.slope, ll.phi_ < k_pi * 0.45f);
        fr[1] = r.phi_control(k_pi * 0.5f, np.kp, np.kd, np.slope, rl.phi_ < k_pi * 0.45f);
        if (in.cmd.v > 0.005f)
        {
            twl = twr = 1.5f;
        }
        else if (in.cmd.v < -0.005f)
        {
            twl = twr = -1.5f;
        }
        if (ll.neutral_ && rl.neutral_)
        {
            state_ = chassis_state::normal;
            l.delta_init_ = false;
            r.delta_init_ = false;
            target_len_ = cfg_.lmin;
            len_slope_.set_default(ll.len_);
        }
    }

    void step_normal(const fsm_inputs& in, fsm_outputs& out, float pitch, float fl[2], float fr[2], float& twl,
                     float& twr)
    {
        auto& l = in.left;
        auto& r = in.right;
        const auto& ll = l.link();
        const auto& rl = r.link();

        ref_x_[0] = in.cmd.x;
        ref_x_[1] = in.cmd.v;
        ref_x_[2] = in.cmd.yaw;
        ref_x_[3] = in.cmd.dyaw;
        ref_x_[4] = ll.alpha_eq_ - 0.02f;
        ref_x_[5] = 0.0f;
        ref_x_[6] = rl.alpha_eq_ - 0.02f;
        ref_x_[7] = 0.0f;
        ref_x_[8] = 0.0f;
        ref_x_[9] = 0.0f;

        lqr_.mode = ((ll.len_ + rl.len_) * 0.5f > cfg_.lswitch) ? lqr_mode::high : lqr_mode::low;
        lqr_.update(ll.len_, rl.len_, ref_x_, in.observed_x, cfg_);
        twl = lqr_.tout[0];
        twr = lqr_.tout[1];
        fl[1] = lqr_.tout[2];
        fr[1] = lqr_.tout[3];

        roll_pd_.ref = 0.0f;
        roll_pd_.fdb = in.ins.roll;
        roll_pd_.update_result(in.ins.gyro_r);

        if (in.cmd.spin)
        {
            state_ = chassis_state::spin;
        }

        if (!going_stair_ && !flying_)
        {
            target_len_ = clamp(in.cmd.len, cfg_.lmin, cfg_.lmax);
        }

        const float cmd_len = len_slope_.update_val(target_len_);
        fl[0] = l.len_control(cmd_len + roll_pd_.result_) + cfg_.gff - ll.fs_;
        fr[0] = r.len_control(cmd_len - roll_pd_.result_) + cfg_.gff - rl.fs_;

        if (++air_protect_cnt_ < 500)
        {
            out.pendulum.normal = false;
        }
        else
        {
            out.pendulum.normal = true;
            len_slope_.set_asymmetric(0.00035f, 0.0007f);
        }

        const bool offground =
            (in.n_total < 0.0f) && (ll.dlen_ > 0.05f) && (rl.dlen_ > 0.05f) && (air_protect_cnt_ > 500);
        if (offground)
        {
            state_ = chassis_state::offground;
            landing_cnt_ = 0;
        }

        if (ll.alpha_ >= 0.8f || rl.alpha_ >= 0.8f)
        {
            state_ = chassis_state::relax;
            l.delta_init_ = false;
            r.delta_init_ = false;
        }

        const auto& lb = cfg_.fsm_pid.len_balance;
        l.tune_len_pd(lb.kp, lb.ki, lb.kd);
        r.tune_len_pd(lb.kp, lb.ki, lb.kd);
        (void)pitch;
    }

    void step_offground(const fsm_inputs& in, float pitch, float fl[2], float fr[2], float& twl, float& twr)
    {
        auto& l = in.left;
        auto& r = in.right;
        const auto& ll = l.link();
        const auto& rl = r.link();

        ref_x_[0] = in.observed_x[0];
        ref_x_[1] = in.observed_x[1];
        ref_x_[2] = in.observed_x[2];
        ref_x_[3] = in.observed_x[3];
        ref_x_[4] = pitch;
        ref_x_[5] = 0.0f;
        ref_x_[6] = pitch;
        ref_x_[7] = 0.0f;
        ref_x_[8] = in.observed_x[8];
        ref_x_[9] = in.observed_x[9];

        lqr_.mode = lqr_mode::low;
        lqr_.update(ll.len_, rl.len_, ref_x_, in.observed_x, cfg_);
        twl = twr = 0.0f;
        fl[1] = lqr_.tout[2] * 0.4f;
        fr[1] = lqr_.tout[3] * 0.4f;
        fl[0] = fr[0] = 0.0f;

        const bool landing = flying_ ? (ll.dlen_ < -0.05f && rl.dlen_ < -0.05f && ++landing_cnt_ > 100)
                                     : (ll.dlen_ < 0.0f && rl.dlen_ < 0.0f);

        if (ll.alpha_ >= 0.8f || rl.alpha_ >= 0.8f)
        {
            state_ = chassis_state::relax;
            l.delta_init_ = false;
            r.delta_init_ = false;
        }
        else if (landing)
        {
            landing_cnt_ = 0;
            air_protect_cnt_ = 0;
            state_ = chassis_state::normal;
            target_len_ = cfg_.lmin;
            len_slope_.set_default(ll.len_);
            len_slope_.set_path(flying_ ? 0.0006f : 0.0009f);
            flying_ = false;
        }
    }

    void step_spin(const fsm_inputs& in, float fl[2], float fr[2], float& twl, float& twr)
    {
        auto& l = in.left;
        auto& r = in.right;
        const auto& ll = l.link();
        const auto& rl = r.link();

        ref_x_[0] = in.cmd.x;
        ref_x_[1] = in.cmd.v;
        ref_x_[2] = in.cmd.yaw;
        ref_x_[3] = in.cmd.dyaw;
        ref_x_[4] = ll.alpha_eq_ - 0.03f;
        ref_x_[5] = 0.0f;
        ref_x_[6] = rl.alpha_eq_ - 0.03f;
        ref_x_[7] = 0.0f;
        ref_x_[8] = 0.0f;
        ref_x_[9] = 0.0f;

        lqr_.mode = lqr_mode::spin;
        lqr_.update(ll.len_, rl.len_, ref_x_, in.observed_x, cfg_);
        twl = lqr_.tout[0];
        twr = lqr_.tout[1];
        fl[1] = lqr_.tout[2];
        fr[1] = lqr_.tout[3];

        roll_pd_.ref = 0.0f;
        roll_pd_.fdb = in.ins.roll;
        roll_pd_.update_result(in.ins.gyro_r);

        const float cmd_len = len_slope_.update_val(in.cmd.len);
        fl[0] = l.len_control(cmd_len + roll_pd_.result_) + cfg_.gff - ll.fs_;
        fr[0] = r.len_control(cmd_len - roll_pd_.result_) + cfg_.gff - rl.fs_;

        if (!in.cmd.spin)
        {
            state_ = chassis_state::normal;
        }

        const auto& lb = cfg_.fsm_pid.len_balance;
        l.tune_len_pd(lb.kp, lb.ki, lb.kd);
        r.tune_len_pd(lb.kp, lb.ki, lb.kd);
    }

    chassis_config cfg_ = k_default_chassis;
    chassis_state state_ = chassis_state::relax;
    lqr_solver lqr_;
    pid roll_pd_{k_default_chassis.fsm_pid.roll};
    slope len_slope_{0.16f, 0.0002f};
    float ref_x_[10] = {};
    float target_len_ = 0.16f;
    std::uint32_t air_protect_cnt_ = 0;
    std::uint32_t flipover_cnt_ = 0;
    std::uint32_t landing_cnt_ = 0;
    bool flying_ = false;
    bool going_stair_ = false;
};

}  // namespace control
