#include "controller/app.hpp"

#include "control/balance.hpp"
#include "control/chassis_fsm.hpp"
#include "control/imu_fusion.hpp"
#include "control/math.hpp"

#include "msg/msg.hpp"
#include "robot_ipc/channel.hpp"
#include "robot_ipc/input_shm.hpp"
#include "robot_msgs/types.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <thread>

namespace controller
{

class shm_io
{
public:
    explicit shm_io(const app_config& cfg) { init(cfg); }

    bool valid() const { return state_in_.valid() && cmd_out_.valid(); }

    bool read_low_state(robot_msgs::LowState& state)
    {
        return state_in_.read(state, true, 100) == robot_ipc::channel_status::ok;
    }

    void write_low_cmd(const control::msg_motor_cmd_t& motor)
    {
        robot_msgs::LowState state{};
        if (state_in_.read(state, false) != robot_ipc::channel_status::ok)
        {
            state.num_motors = 6;
        }

        robot_msgs::LowCmd cmd{};
        cmd.num_motors = state.num_motors > 0 ? state.num_motors : 6;
        for (std::uint32_t i = 0; i < cmd.num_motors && i < 6; ++i)
        {
            cmd.motors[i].mode = robot_msgs::kMotorModeTorque;
            cmd.motors[i].tau = motor.tau[i] * cfg_.motor_sign_cmd[i];
        }
        cmd_out_.write(cmd);
    }

    control::msg_raw_state_t to_raw_state(const robot_msgs::LowState& state) const
    {
        control::msg_raw_state_t raw{};
        raw.time = state.time;
        for (int i = 0; i < 3; ++i)
        {
            raw.gyro[i] = state.imu.gyro[i];
            raw.accel[i] = state.imu.accel[i];
            raw.quat_gt[i] = state.imu.quat[i];
        }
        raw.quat_gt[3] = state.imu.quat[3];

        const std::uint32_t n = state.num_motors < 6 ? state.num_motors : 6;
        for (std::uint32_t i = 0; i < n; ++i)
        {
            raw.motors[i].q = state.motors[i].q;
            raw.motors[i].dq = state.motors[i].dq;
            raw.motors[i].tau = state.motors[i].tau_est;
        }
        return raw;
    }

    void apply_imu_noise(control::msg_raw_state_t& raw)
    {
        std::normal_distribution<float> gyro_noise(0.0f, cfg_.imu_sim.gyro_noise_std);
        std::normal_distribution<float> accel_noise(0.0f, cfg_.imu_sim.accel_noise_std);

        for (int i = 0; i < 3; ++i)
        {
            raw.gyro[i] += cfg_.imu_sim.gyro_bias[i];
            if (cfg_.imu_sim.gyro_noise_std > 0.0f)
            {
                raw.gyro[i] += gyro_noise(rng_);
            }
            if (cfg_.imu_sim.accel_noise_std > 0.0f)
            {
                raw.accel[i] += accel_noise(rng_);
            }
        }

        if (cfg_.imu_sim.lever_arm_x != 0.0f)
        {
            raw.accel[0] += cfg_.imu_sim.lever_arm_x * raw.gyro[2] * raw.gyro[2];
        }
    }

    control::input_snapshot_t read_input()
    {
        control::input_snapshot_t in{};
        robot_ipc::input_shm_t shm{};
        if (input_in_.valid() && input_in_.read(shm, false) == robot_ipc::channel_status::ok)
        {
            in.w = shm.w != 0;
            in.s = shm.s != 0;
            in.a = shm.a != 0;
            in.d = shm.d != 0;
            in.q = shm.q != 0;
            in.e = shm.e != 0;
            in.space = shm.space != 0;
            in.r = shm.r != 0;
            in.f = shm.f != 0;
        }
        return in;
    }

private:
    void init(const app_config& cfg)
    {
        cfg_ = cfg;
        state_in_ = robot_ipc::ShmChannel<robot_msgs::LowState>::open_client(cfg.ipc_prefix + "_lowstate");
        cmd_out_ = robot_ipc::ShmChannel<robot_msgs::LowCmd>::open_client(cfg.ipc_prefix + "_lowcmd");
        input_in_ = robot_ipc::ShmChannel<robot_ipc::input_shm_t>::open_client(cfg.ipc_prefix + "_input");
    }

    app_config cfg_{};
    robot_ipc::ShmChannel<robot_msgs::LowState> state_in_;
    robot_ipc::ShmChannel<robot_msgs::LowCmd> cmd_out_;
    robot_ipc::ShmChannel<robot_ipc::input_shm_t> input_in_;
    mutable std::mt19937 rng_{std::random_device{}()};
};

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
                       control::chassis_state fsm, float n_total, float x, float v, float az)
{
    std::printf("[t=%.3f] imu quat=(%.3f %.3f %.3f %.3f) rpy_rad=(%.4f %.4f %.4f) gyro=(%.3f %.3f %.3f) "
                "accel=(%.3f %.3f %.3f)\n",
                time, ins.quaternion[0], ins.quaternion[1], ins.quaternion[2], ins.quaternion[3], ins.roll,
                ins.pitch, ins.yaw, ins.gyro_r, ins.gyro_p, ins.gyro_y, ins.accel[0], ins.accel[1],
                ins.accel[2]);

    auto motor = [&](const char* name, int i) {
        std::printf("  %s: q=%.4f dq=%.4f tau=%.4f\n", name, raw.motors[i].q, raw.motors[i].dq,
                    raw.motors[i].tau);
    };
    auto link = [&](const char* tag, const control::link_solver& lk) {
        std::printf("  %s_link: len=%.4f dlen=%.4f phi=%.4f dphi=%.4f alpha=%.4f dalpha=%.4f alpha_eq=%.4f "
                    "n=%.4f f=%.4f t_hip=%.4f total_phi=%.4f flat=%d neutral=%d\n",
                    tag, lk.len_, lk.dlen_, lk.phi_, lk.dphi_, lk.alpha_, lk.dalpha_, lk.alpha_eq_, lk.n_,
                    lk.freal_, lk.treal_hip_, lk.total_phi_, static_cast<int>(lk.flat_),
                    static_cast<int>(lk.neutral_));
    };

    motor("ljoint1", 0);
    motor("ljoint4", 1);
    motor("lwheel", 2);
    link("left", left);
    motor("rjoint1", 3);
    motor("rjoint4", 4);
    motor("rwheel", 5);
    link("right", right);
    std::printf("  state: fsm=%d n_total=%.4f odom=(x=%.4f v=%.4f az=%.4f)\n", static_cast<int>(fsm), n_total,
                x, v, az);
    std::fflush(stdout);
}

}  // namespace

output_node::output_node(const app_config& cfg, shm_io& io, std::atomic<bool>& running)
    : cfg_(cfg), io_(io), running_(running), thread_([this] { loop(); })
{
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
    auto next = std::chrono::steady_clock::now();
    while (running_.load())
    {
        robot_msgs::LowState state{};
        if (io_.read_low_state(state))
        {
            msg::publish(io_.to_raw_state(state), {false});
        }

        control::msg_motor_cmd_t motor{};
        if (msg::read(sub_motor_cmd_, motor) != msg::status::ok)
        {
            std::memset(&motor, 0, sizeof(motor));
        }
        io_.write_low_cmd(motor);
        sleep_until_tick(next, cfg_.control_hz);
    }
}

imu_node::imu_node(const app_config& cfg, shm_io& io, std::atomic<bool>& running)
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

cmd_node::cmd_node(const app_config& cfg, shm_io& io, std::atomic<bool>& running)
    : cfg_(cfg), io_(io), running_(running), thread_([this] { loop(); })
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
        const control::input_snapshot_t input = io_.read_input();
        msg::publish(input, {false});

        control::msg_pendulum_t pendulum{};
        control::msg_ins_t ins{};
        msg::read(sub_pendulum_, pendulum);
        msg::read(sub_ins_, ins);

        fusion.update(input, pendulum, ins, cfg_.chassis, dt);
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
    control::leg_controller left(false, cfg_.chassis);
    control::leg_controller right(true, cfg_.chassis);
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
        const float vl = -raw.motors[2].dq * cfg_.chassis.rwheel;
        const float vr = raw.motors[5].dq * cfg_.chassis.rwheel;

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

        msg::publish(fout.motor, {false});
        msg::publish(fout.pendulum, {false});

        if (log_every > 0 && ++log_counter >= log_every)
        {
            log_counter = 0;
            print_state_block(raw.time, ins, raw, ll, rl, fsm.state(), fin.n_total, odom.x, odom.v, odom.az);
        }

        sleep_until_tick(next, cfg_.control_hz);
    }
}

controller_app::controller_app(const app_config& cfg)
    : cfg_(cfg), io_(std::make_unique<shm_io>(cfg)), output_(cfg_, *io_, running_), imu_(cfg_, *io_, running_),
      cmd_(cfg_, *io_, running_), control_(cfg_, running_)
{
    if (!io_->valid())
    {
        std::fprintf(stderr, "failed to open shm channels (ipc=%s)\n", cfg_.ipc_prefix.c_str());
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
