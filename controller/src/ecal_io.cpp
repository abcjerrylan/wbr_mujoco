#include "controller/ecal_io.hpp"

#include "msg/msg.hpp"

#include "mujoco_interface/protocol/messages.hpp"
#include "mujoco_interface/types.hpp"

#include <algorithm>
#include <cstdio>

namespace controller
{

namespace
{

control::msg_raw_state_t to_raw_state(const mujoco_interface::protocol::state_envelope& envelope)
{
    control::msg_raw_state_t raw{};
    const auto& state = envelope.body;
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

control::input_snapshot_t to_input_snapshot(const mujoco_interface::protocol::input_message& input)
{
    control::input_snapshot_t snap{};
    snap.w = input.w;
    snap.s = input.s;
    snap.a = input.a;
    snap.d = input.d;
    snap.q = input.q;
    snap.e = input.e;
    snap.space = input.space;
    snap.r = input.r;
    snap.f = input.f;
    return snap;
}

}  // namespace

ecal_io::ecal_io(const app_config& cfg) : cfg_(cfg)
{
    std::string error;
    if (!client_.start(cfg_.ipc_prefix, "wbr_ctrl", error))
    {
        std::fprintf(stderr, "ecal_io: start failed: %s\n", error.c_str());
        return;
    }

    client_.set_register_ack_handler(
        [this](const mujoco_interface::protocol::register_ack_message& ack)
        {
            if (!ack.accepted)
            {
                return;
            }

            num_motors_ = ack.num_motors > 0 ? ack.num_motors : 6;
            ack_epoch_ = ack.sync.epoch;
            const bool first = !registered_.exchange(true);
            if (first)
            {
                std::printf("ecal_io: registered with sim (%u motors, epoch=%u)\n", num_motors_, ack_epoch_);
            }
        });

    client_.set_state_handler(
        [](const mujoco_interface::protocol::state_envelope& state)
        {
            msg::publish(to_raw_state(state), {false});
        });

    client_.set_input_handler(
        [](const mujoco_interface::protocol::input_message& input)
        {
            msg::publish(to_input_snapshot(input), {true});
        });

    client_.set_tick_handler(
        [this](const mujoco_interface::protocol::tick_message& tick)
        {
            if (registered_.load() && tick.sync.epoch != ack_epoch_)
            {
                registered_.store(false);
                try_register();
                return;
            }

            if (!registered_.load())
            {
                try_register();
                return;
            }

            const std::lock_guard<std::mutex> lock(tick_mutex_);
            latest_tick_ = tick;
            latest_tick_received_ = std::chrono::steady_clock::now();
            has_pending_tick_ = true;
            tick_cv_.notify_one();
        });

    try_register();
    valid_.store(true);
}

ecal_io::~ecal_io()
{
    client_.stop();
}

void ecal_io::poll()
{
    client_.poll();

    const auto now = std::chrono::steady_clock::now();
    if (last_register_attempt_.time_since_epoch().count() == 0 ||
        now - last_register_attempt_ >= std::chrono::seconds(1))
    {
        try_register();
    }
}

bool ecal_io::wait_for_tick_and_commit(std::chrono::steady_clock::duration timeout)
{
    {
        std::unique_lock<std::mutex> lock(tick_mutex_);
        if (!has_pending_tick_)
        {
            tick_cv_.wait_for(lock, timeout, [this] { return has_pending_tick_ || !registered_.load(); });
        }
    }

    const std::uint64_t before = last_sent_tick_id_;
    flush_pending_commit();
    return last_sent_tick_id_ != before;
}

void ecal_io::flush_pending_commit()
{
    if (!registered_.load())
    {
        return;
    }

    mujoco_interface::protocol::tick_message tick{};
    std::chrono::steady_clock::time_point tick_received{};
    {
        const std::lock_guard<std::mutex> lock(tick_mutex_);
        if (!has_pending_tick_ || latest_tick_.sync.tick_id <= last_sent_tick_id_)
        {
            return;
        }
        tick = latest_tick_;
        tick_received = latest_tick_received_;
        has_pending_tick_ = false;
    }

    control::msg_motor_cmd_t motor{};
    {
        const std::lock_guard<std::mutex> lock(motor_mutex_);
        motor = motor_cmd_;
    }

    mujoco_interface::protocol::command_envelope commit{};
    commit.sync = tick.sync;
    commit.sync.session_id = 100;
    commit.sync.client_id = 1;
    commit.body.num_motors = num_motors_;
    for (std::uint32_t i = 0; i < num_motors_ && i < 6; ++i)
    {
        commit.body.motors[i].mode = mujoco_interface::robot::mode_torque;
        commit.body.motors[i].tau = motor.tau[i];
    }
    if (client_.send_commit(commit))
    {
        const auto sent_at = std::chrono::steady_clock::now();
        const double tick_to_commit_us =
            std::chrono::duration<double, std::micro>(sent_at - tick_received).count();
        ++stats_count_;
        stats_sum_tick_to_commit_us_ += tick_to_commit_us;
        stats_max_tick_to_commit_us_ = std::max(stats_max_tick_to_commit_us_, tick_to_commit_us);
        if (stats_sample_count_ < stats_samples_us_.size())
        {
            stats_samples_us_[stats_sample_count_++] = tick_to_commit_us;
        }
        if (tick.sync.tick_id > last_sent_tick_id_ + 1 && last_sent_tick_id_ != 0)
        {
            stats_missed_ticks_ += tick.sync.tick_id - last_sent_tick_id_ - 1;
        }
        last_sent_tick_id_ = tick.sync.tick_id;

        const auto now = sent_at;
        const auto stats_period = cfg_.logger.hz > 0.0f
                                      ? std::chrono::duration<double>(1.0 / static_cast<double>(cfg_.logger.hz))
                                      : std::chrono::duration<double>(1.0);
        if (cfg_.logger.stdout_block && stats_count_ > 0 &&
            (last_stats_print_.time_since_epoch().count() == 0 ||
             now - last_stats_print_ >=
                 std::chrono::duration_cast<std::chrono::steady_clock::duration>(stats_period)))
        {
            const double avg_us = stats_sum_tick_to_commit_us_ / static_cast<double>(stats_count_);
            std::sort(stats_samples_us_.begin(), stats_samples_us_.begin() + stats_sample_count_);
            const auto percentile = [this](double q) {
                if (stats_sample_count_ == 0)
                {
                    return 0.0;
                }
                const auto idx = static_cast<std::size_t>(
                    q * static_cast<double>(stats_sample_count_ - 1));
                return stats_samples_us_[idx];
            };
            std::printf("ecal_io: tick_commit_us avg=%.1f p50=%.1f p95=%.1f p99=%.1f max=%.1f "
                        "samples=%llu missed_ticks=%llu last_tick=%llu\n",
                        avg_us, percentile(0.50), percentile(0.95), percentile(0.99), stats_max_tick_to_commit_us_,
                        static_cast<unsigned long long>(stats_count_),
                        static_cast<unsigned long long>(stats_missed_ticks_),
                        static_cast<unsigned long long>(last_sent_tick_id_));
            stats_count_ = 0;
            stats_missed_ticks_ = 0;
            stats_sum_tick_to_commit_us_ = 0.0;
            stats_max_tick_to_commit_us_ = 0.0;
            stats_sample_count_ = 0;
            last_stats_print_ = now;
        }
    }
}

void ecal_io::try_register()
{
    last_register_attempt_ = std::chrono::steady_clock::now();

    mujoco_interface::protocol::register_message request{};
    request.client_id = 1;
    request.session_id = 100;
    request.motor_begin = 0;
    request.motor_count = 6;

    std::string error;
    if (!client_.register_client(request, error))
    {
        std::fprintf(stderr, "ecal_io: register send failed: %s\n", error.c_str());
    }
}

void ecal_io::update_motor_cmd(const control::msg_motor_cmd_t& motor)
{
    const std::lock_guard<std::mutex> lock(motor_mutex_);
    motor_cmd_ = motor;
}

void ecal_io::apply_imu_noise(control::msg_raw_state_t& raw) const
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

}  // namespace controller
