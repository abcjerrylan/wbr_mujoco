#include "controller/services.hpp"

#include "controller/ecal_io.hpp"

#include "control/balance.hpp"
#include "control/chassis_fsm.hpp"
#include "control/imu_fusion.hpp"
#include "control/leg.hpp"
#include "control/math.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace controller
{

namespace
{

void sleep_until_tick(std::chrono::steady_clock::time_point& next, float hz)
{
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(hz)));
    std::this_thread::sleep_until(next);
}

void publish_ins(const control::imu_attitude_t* att, const control::msg_raw_state_t& raw)
{
    control::msg_ins_t ins{};
    if (att != nullptr)
    {
        for (int i = 0; i < 4; ++i)
        {
            ins.quaternion[i] = att->q[i];
        }
        ins.roll = att->roll;
        ins.pitch = att->pitch;
        ins.yaw = att->yaw;
        ins.total_yaw = att->total_yaw;
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            ins.quaternion[i] = raw.quat_gt[i];
        }
        control::quat_to_euler(ins.quaternion, ins.roll, ins.pitch, ins.yaw);
        ins.total_yaw = ins.yaw;
    }
    ins.gyro_r = raw.gyro[0];
    ins.gyro_p = raw.gyro[1];
    ins.gyro_y = raw.gyro[2];
    for (int i = 0; i < 3; ++i)
    {
        ins.accel[i] = raw.accel[i];
    }
    msg::publish(ins, {false});
}

void fill_leg_log(control::msg_log_t& log, const control::link_solver& leg, bool left)
{
    if (left)
    {
        log.l_len = leg.len_;
        log.l_dlen = leg.dlen_;
        log.l_alpha = leg.alpha_;
        log.l_dalpha = leg.dalpha_;
        log.l_phi = leg.phi_;
        log.l_dphi = leg.dphi_;
        log.l_alpha_eq = leg.alpha_eq_;
        log.l_t_hip = leg.treal_hip_;
        log.l_total_phi = leg.total_phi_;
        log.l_n = leg.n_;
        log.l_f = leg.freal_;
        log.l_fs = leg.fs_;
        log.l_flat = leg.flat_ ? 1.0f : 0.0f;
        log.l_neutral = leg.neutral_ ? 1.0f : 0.0f;
        return;
    }

    log.r_len = leg.len_;
    log.r_dlen = leg.dlen_;
    log.r_alpha = leg.alpha_;
    log.r_dalpha = leg.dalpha_;
    log.r_phi = leg.phi_;
    log.r_dphi = leg.dphi_;
    log.r_alpha_eq = leg.alpha_eq_;
    log.r_t_hip = leg.treal_hip_;
    log.r_total_phi = leg.total_phi_;
    log.r_n = leg.n_;
    log.r_f = leg.freal_;
    log.r_fs = leg.fs_;
    log.r_flat = leg.flat_ ? 1.0f : 0.0f;
    log.r_neutral = leg.neutral_ ? 1.0f : 0.0f;
}

control::msg_log_t make_log(double time, const control::msg_ins_t& ins, const control::msg_raw_state_t& raw,
                            const control::link_solver& left, const control::link_solver& right,
                            const control::msg_ctrl_t& ctrl, const control::msg_motor_cmd_t& motor_cmd,
                            const control::msg_cmd_t& cmd, control::chassis_state fsm, float n_total, float x,
                            float v, float az)
{
    control::msg_log_t log{};
    log.time = time;
    std::memcpy(log.quaternion, ins.quaternion, sizeof(log.quaternion));
    log.roll = ins.roll;
    log.pitch = ins.pitch;
    log.yaw = ins.yaw;
    log.gyro[0] = ins.gyro_r;
    log.gyro[1] = ins.gyro_p;
    log.gyro[2] = ins.gyro_y;
    std::memcpy(log.accel, ins.accel, sizeof(log.accel));
    log.x = x;
    log.v = v;
    log.az = az;
    fill_leg_log(log, left, true);
    fill_leg_log(log, right, false);
    for (int i = 0; i < 6; ++i)
    {
        log.m_q[i] = raw.motors[i].q;
        log.m_dq[i] = raw.motors[i].dq;
        log.m_tau[i] = raw.motors[i].tau;
        log.cmd_tau[i] = motor_cmd.tau[i];
    }
    log.Tl0 = ctrl.Tl[0];
    log.Tl1 = ctrl.Tl[1];
    log.Tr0 = ctrl.Tr[0];
    log.Tr1 = ctrl.Tr[1];
    log.Twl = ctrl.Twl;
    log.Twr = ctrl.Twr;
    log.fsm = static_cast<std::uint8_t>(fsm);
    log.n_total = n_total;
    log.cmd_v = cmd.v;
    log.cmd_len = cmd.len;
    log.cmd_move = cmd.move;
    return log;
}

void print_state_block(const control::msg_log_t& log)
{
    std::printf("[t=%.3f] imu quat=(%.3f %.3f %.3f %.3f) rpy_rad=(%.4f %.4f %.4f) gyro=(%.3f %.3f %.3f) "
                "accel=(%.3f %.3f %.3f)\n",
                log.time, log.quaternion[0], log.quaternion[1], log.quaternion[2], log.quaternion[3], log.roll,
                log.pitch, log.yaw, log.gyro[0], log.gyro[1], log.gyro[2], log.accel[0], log.accel[1],
                log.accel[2]);

    auto print_motor = [&](const char* name, int i) {
        std::printf("  %s: q=%.4f dq=%.4f tau_est=%.4f tau_cmd=%.4f\n", name, log.m_q[i], log.m_dq[i],
                    log.m_tau[i], log.cmd_tau[i]);
    };
    auto print_link = [&](const char* tag, bool left, int motor_base) {
        const float len = left ? log.l_len : log.r_len;
        const float dlen = left ? log.l_dlen : log.r_dlen;
        const float phi = left ? log.l_phi : log.r_phi;
        const float dphi = left ? log.l_dphi : log.r_dphi;
        const float alpha = left ? log.l_alpha : log.r_alpha;
        const float dalpha = left ? log.l_dalpha : log.r_dalpha;
        const float alpha_eq = left ? log.l_alpha_eq : log.r_alpha_eq;
        const float n = left ? log.l_n : log.r_n;
        const float fs = left ? log.l_fs : log.r_fs;
        const float f_est = left ? log.l_f : log.r_f;
        const float t_hip = left ? log.l_t_hip : log.r_t_hip;
        const float total_phi = left ? log.l_total_phi : log.r_total_phi;
        const float flat = left ? log.l_flat : log.r_flat;
        const float neutral = left ? log.l_neutral : log.r_neutral;
        const float f0 = left ? log.Tl0 : log.Tr0;
        const float f1 = left ? log.Tl1 : log.Tr1;
        std::printf("  %s_link: len=%.4f dlen=%.4f phi=%.4f dphi=%.4f alpha=%.4f dalpha=%.4f alpha_eq=%.4f "
                    "n=%.4f Fs=%.4f F=(%.4f,%.4f) tau=(%.4f,%.4f,%.4f) F_est=%.4f tau_hip_est=%.4f "
                    "total_phi=%.4f flat=%d neutral=%d\n",
                    tag, len, dlen, phi, dphi, alpha, dalpha, alpha_eq, n, fs, f0, f1,
                    log.cmd_tau[motor_base], log.cmd_tau[motor_base + 1], log.cmd_tau[motor_base + 2],
                    f_est, t_hip, total_phi, static_cast<int>(flat), static_cast<int>(neutral));
    };

    print_motor("ljoint1", 0);
    print_motor("ljoint4", 1);
    print_motor("lwheel", 2);
    print_link("left", true, 0);
    print_motor("rjoint1", 3);
    print_motor("rjoint4", 4);
    print_motor("rwheel", 5);
    print_link("right", false, 3);
    std::printf("  state: fsm=%u cmd.len=%.4f cmd.v=%.4f Tw=(%.4f,%.4f) move=%d n_total=%.4f "
                "odom=(x=%.4f v=%.4f az=%.4f)\n",
                static_cast<unsigned>(log.fsm), log.cmd_len, log.cmd_v, log.Twl, log.Twr,
                static_cast<int>(log.cmd_move), log.n_total, log.x, log.v, log.az);
    std::fflush(stdout);
}

}  // namespace

actuator_service::actuator_service(const app_config& cfg, ecal_io& io, std::atomic<bool>& running)
    : cfg_(cfg), io_(io), running_(running), thread_([this] { loop(); })
{
    io_.update_motor_cmd({});
}

actuator_service::~actuator_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void actuator_service::loop()
{
    const auto tick_wait = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(cfg_.control_hz)));
    while (running_.load())
    {
        io_.poll();

        control::msg_motor_cmd_t latest{};
        if (msg::read(sub_motor_cmd_, latest) == msg::status::ok)
        {
            motor_cmd_ = latest;
        }
        io_.update_motor_cmd(motor_cmd_);

        io_.wait_for_tick_and_commit(tick_wait);
    }
}

ins_service::ins_service(const app_config& cfg, ecal_io& io, std::atomic<bool>& running)
    : cfg_(cfg), io_(io), running_(running), thread_([this] { loop(); })
{
}

ins_service::~ins_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void ins_service::loop()
{
    control::mahony_filter mahony;
    mahony.reset();

    auto next = std::chrono::steady_clock::now();
    const float dt = 1.0f / cfg_.control_hz;

    while (running_.load())
    {
        control::msg_raw_state_t raw{};
        if (msg::read(sub_raw_state_, raw) == msg::status::ok)
        {
            io_.apply_imu_noise(raw);
            if (cfg_.imu_mode == control::imu_mode::bypass)
            {
                publish_ins(nullptr, raw);
            }
            else
            {
                mahony.update(raw.gyro[0], raw.gyro[1], raw.gyro[2], raw.accel[0], raw.accel[1], raw.accel[2],
                              dt);
                publish_ins(&mahony.attitude, raw);
            }
        }
        sleep_until_tick(next, cfg_.control_hz);
    }
}

command_service::command_service(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

command_service::~command_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void command_service::loop()
{
    control::command_fusion fusion;
    fusion.reset(cfg_.chassis);

    auto next = std::chrono::steady_clock::now();
    const float dt = 1.0f / cfg_.control_hz;

    while (running_.load())
    {
        control::input_snapshot_t latest{};
        if (msg::read(sub_input_, latest) == msg::status::ok)
        {
            input_ = latest;
        }

        control::msg_pendulum_t pendulum{};
        control::msg_ins_t ins{};
        msg::read(sub_pendulum_, pendulum);
        msg::read(sub_ins_, ins);

        fusion.update(input_, pendulum, ins, cfg_.chassis, dt);
        msg::publish(fusion.msg(), {false});
        sleep_until_tick(next, cfg_.control_hz);
    }
}

chassis_service::chassis_service(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

chassis_service::~chassis_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void chassis_service::loop()
{
    // Left-view positive: left leg is the reference frame, right leg is mirrored.
    control::leg_controller left(true, 0, 1, cfg_.chassis);
    control::leg_controller right(false, 3, 4, cfg_.chassis);
    control::odometry odom;
    control::chassis_fsm fsm;
    fsm.init(cfg_.chassis);

    auto next = std::chrono::steady_clock::now();
    const float dt = 1.0f / cfg_.control_hz;

    while (running_.load())
    {
        control::msg_raw_state_t raw{};
        control::msg_ins_t ins{};
        control::msg_cmd_t cmd{};
        if (msg::read(sub_raw_state_, raw) != msg::status::ok || msg::read(sub_ins_, ins) != msg::status::ok ||
            msg::read(sub_cmd_, cmd) != msg::status::ok)
        {
            sleep_until_tick(next, cfg_.control_hz);
            continue;
        }

        const float pitch = ins.pitch;
        const float dpitch = ins.gyro_p;
        const float yaw = ins.total_yaw;
        const float dyaw = ins.gyro_y;
        const float vl = raw.motors[2].dq * cfg_.chassis.rwheel * left.wheel_sign();
        const float vr = raw.motors[5].dq * cfg_.chassis.rwheel * right.wheel_sign();

        left.link().solve(pitch, dpitch, 0.0f, raw.motors[0], raw.motors[1]);
        right.link().solve(pitch, dpitch, 0.0f, raw.motors[3], raw.motors[4]);
        odom.update(ins.quaternion, ins.accel, (vl + vr) * 0.5f, yaw, dt);
        left.link().solve(pitch, dpitch, odom.az, raw.motors[0], raw.motors[1]);
        right.link().solve(pitch, dpitch, odom.az, raw.motors[3], raw.motors[4]);

        const auto& ll = left.link();
        const auto& rl = right.link();

        float observed_x[10] = {};
        observed_x[0] = odom.x;
        observed_x[1] = odom.v;
        observed_x[2] = yaw;
        observed_x[3] = dyaw;
        observed_x[4] = ll.alpha_;
        observed_x[5] = ll.dalpha_;
        observed_x[6] = rl.alpha_;
        observed_x[7] = rl.dalpha_;
        observed_x[8] = pitch;
        observed_x[9] = dpitch;

        control::msg_odometry_t odom_msg{};
        odom_msg.x = odom.x;
        odom_msg.v = odom.v;
        odom_msg.a_z = odom.az;
        msg::publish(odom_msg, {false});

        control::fsm_inputs fin{ins, cmd, odom_msg, left, right};
        std::memcpy(fin.observed_x, observed_x, sizeof(observed_x));
        fin.n_total = ll.n_ + rl.n_;
        fin.chassis_dead = false;

        control::fsm_outputs fout{};
        fsm.step(fin, fout);

        if (!cmd.move || cfg_.chassis.force_relax)
        {
            std::memset(&fout.motor, 0, sizeof(fout.motor));
        }

        msg::publish(fout.motor, {true});
        msg::publish(fout.pendulum, {false});
        msg::publish(make_log(raw.time, ins, raw, ll, rl, fout.ctrl, fout.motor, cmd, fsm.state(), fin.n_total,
                              odom.x, odom.v, odom.az),
                     {false});

        sleep_until_tick(next, cfg_.control_hz);
    }
}

sim_log_service::sim_log_service(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

sim_log_service::~sim_log_service()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void sim_log_service::loop()
{
    auto next = std::chrono::steady_clock::now();
    const int log_every = cfg_.logger.stdout_block && cfg_.logger.hz > 0.0f
                              ? static_cast<int>(cfg_.control_hz / cfg_.logger.hz + 0.5f)
                              : 0;
    int log_counter = 0;

    while (running_.load())
    {
        control::msg_log_t log{};
        if (log_every > 0 && msg::read(sub_log_, log) == msg::status::ok && ++log_counter >= log_every)
        {
            log_counter = 0;
            print_state_block(log);
        }
        sleep_until_tick(next, cfg_.control_hz);
    }
}

}  // namespace controller
