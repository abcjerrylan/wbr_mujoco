#include "controller/app.hpp"
#include "controller/ecal_io.hpp"

#include "control/balance.hpp"
#include "control/chassis_fsm.hpp"
#include "control/imu_fusion.hpp"
#include "control/math.hpp"

#include "msg/msg.hpp"

#include <chrono>
#include <cmath>
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

void print_state_block(double time, const control::msg_ins_t& ins, const control::msg_raw_state_t& raw,
                       const control::link_solver& left, const control::link_solver& right,
                       const control::msg_ctrl_t& ctrl, const control::msg_motor_cmd_t& motor_cmd,
                       const control::msg_cmd_t& cmd, control::chassis_state fsm, float n_total, float x,
                       float v, float az)
{
    std::printf("[t=%.3f] imu quat=(%.3f %.3f %.3f %.3f) rpy_rad=(%.4f %.4f %.4f) gyro=(%.3f %.3f %.3f) "
                "accel=(%.3f %.3f %.3f)\n",
                time, ins.quaternion[0], ins.quaternion[1], ins.quaternion[2], ins.quaternion[3], ins.roll,
                ins.pitch, ins.yaw, ins.gyro_r, ins.gyro_p, ins.gyro_y, ins.accel[0], ins.accel[1],
                ins.accel[2]);

    auto print_motor = [&](const char* name, int i) {
        std::printf("  %s: q=%.4f dq=%.4f tau_est=%.4f tau_cmd=%.4f\n", name, raw.motors[i].q, raw.motors[i].dq,
                    raw.motors[i].tau, motor_cmd.tau[i]);
    };
    auto link = [&](const char* tag, const control::link_solver& lk, const float f_cmd[2], int motor_base) {
        std::printf("  %s_link: len=%.4f dlen=%.4f phi=%.4f dphi=%.4f alpha=%.4f dalpha=%.4f alpha_eq=%.4f "
                    "n=%.4f Fs=%.4f F=(%.4f,%.4f) tau=(%.4f,%.4f,%.4f) F_est=%.4f tau_hip_est=%.4f "
                    "total_phi=%.4f flat=%d neutral=%d\n",
                    tag, lk.len_, lk.dlen_, lk.phi_, lk.dphi_, lk.alpha_, lk.dalpha_, lk.alpha_eq_, lk.n_, lk.fs_,
                    f_cmd[0], f_cmd[1], motor_cmd.tau[motor_base], motor_cmd.tau[motor_base + 1],
                    motor_cmd.tau[motor_base + 2], lk.freal_, lk.treal_hip_, lk.total_phi_,
                    static_cast<int>(lk.flat_), static_cast<int>(lk.neutral_));
    };

    print_motor("ljoint1", 0);
    print_motor("ljoint4", 1);
    print_motor("lwheel", 2);
    link("left", left, ctrl.Tl, 0);
    print_motor("rjoint1", 3);
    print_motor("rjoint4", 4);
    print_motor("rwheel", 5);
    link("right", right, ctrl.Tr, 3);
    std::printf("  state: fsm=%d cmd.len=%.4f cmd.v=%.4f Tw=(%.4f,%.4f) move=%d n_total=%.4f odom=(x=%.4f v=%.4f az=%.4f)\n",
                static_cast<int>(fsm), cmd.len, cmd.v, ctrl.Twl, ctrl.Twr, static_cast<int>(cmd.move), n_total,
                x, v, az);
    std::fflush(stdout);
}

}  // namespace

output_node::output_node(const app_config& cfg, ecal_io& io, std::atomic<bool>& running)
    : cfg_(cfg), io_(io), running_(running), thread_([this] { loop(); })
{
    io_.update_motor_cmd({});
}

output_node::~output_node()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void output_node::loop()
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

imu_node::imu_node(const app_config& cfg, ecal_io& io, std::atomic<bool>& running)
    : cfg_(cfg), io_(io), running_(running), thread_([this] { loop(); })
{
}

imu_node::~imu_node()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void imu_node::loop()
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

cmd_node::cmd_node(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

cmd_node::~cmd_node()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void cmd_node::loop()
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

control_node::control_node(const app_config& cfg, std::atomic<bool>& running)
    : cfg_(cfg), running_(running), thread_([this] { loop(); })
{
}

control_node::~control_node()
{
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void control_node::loop()
{
    // Left-view positive: left leg is the reference frame, right leg is mirrored.
    control::leg_controller left(true, 0, 1, cfg_.chassis);
    control::leg_controller right(false, 3, 4, cfg_.chassis);
    control::odometry odom;
    control::chassis_fsm fsm;
    fsm.init(cfg_.chassis);

    auto next = std::chrono::steady_clock::now();
    const float dt = 1.0f / cfg_.control_hz;
    const int log_every = cfg_.logger.stdout_block && cfg_.logger.hz > 0.0f
                              ? static_cast<int>(cfg_.control_hz / cfg_.logger.hz + 0.5f)
                              : 0;
    int log_counter = 0;

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

        if (log_every > 0 && ++log_counter >= log_every)
        {
            log_counter = 0;
            print_state_block(raw.time, ins, raw, ll, rl, fout.ctrl, fout.motor, cmd, fsm.state(), fin.n_total,
                              odom.x, odom.v, odom.az);
        }

        sleep_until_tick(next, cfg_.control_hz);
    }
}

controller_app::controller_app(const app_config& cfg)
    : cfg_(cfg), io_(std::make_unique<ecal_io>(cfg)), output_(cfg_, *io_, running_), imu_(cfg_, *io_, running_),
      cmd_(cfg_, running_), control_(cfg_, running_)
{
    if (!io_->valid())
    {
        std::fprintf(stderr, "failed to open eCAL transport (topic_ns=%s)\n", cfg_.ipc_prefix.c_str());
        running_.store(false);
    }
}

controller_app::~controller_app() = default;

void controller_app::run()
{
    while (running_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void controller_app::shutdown()
{
    running_.store(false);
}

}  // namespace controller
