#include "controller/ecal_io.hpp"

#include "msg/msg.hpp"

#include "mujoco_interface/protocol/messages.hpp"
#include "mujoco_interface/transport/ecal.hpp"
#include "mujoco_interface/types.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <random>

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
    snap.f = input.f;
    return snap;
}

}  // namespace

struct ecal_io::impl
{
    explicit impl(const app_config& next_cfg) : cfg(next_cfg) {}

    app_config cfg;
    mujoco_interface::transport::client client;
    std::atomic<bool> valid{false};
    std::atomic<bool> registered{false};
    std::uint32_t num_motors = 6;
    std::uint32_t ack_epoch = 0;
    std::mutex motor_mutex;
    std::mutex tick_mutex;
    std::condition_variable tick_cv;
    control::msg_motor_cmd_t motor_cmd{};
    mujoco_interface::protocol::tick_message latest_tick{};
    std::chrono::steady_clock::time_point latest_tick_received{};
    bool has_pending_tick = false;
    std::uint64_t last_sent_tick_id = 0;
    mutable std::mt19937 rng{std::random_device{}()};
    std::chrono::steady_clock::time_point last_register_attempt{};

    std::uint64_t stats_count = 0;
    std::uint64_t stats_missed_ticks = 0;
    double stats_sum_tick_to_commit_us = 0.0;
    double stats_max_tick_to_commit_us = 0.0;
    std::array<double, 4096> stats_samples_us{};
    std::size_t stats_sample_count = 0;
    std::chrono::steady_clock::time_point last_stats_print{};
};

ecal_io::ecal_io(const app_config& cfg) : impl_(std::make_unique<impl>(cfg))
{
    std::string error;
    if (!impl_->client.start(impl_->cfg.ipc_prefix, "wbr_ctrl", error))
    {
        std::fprintf(stderr, "ecal_io: start failed: %s\n", error.c_str());
        return;
    }

    impl_->client.set_register_ack_handler(
        [this](const mujoco_interface::protocol::register_ack_message& ack)
        {
            if (!ack.accepted)
            {
                return;
            }

            impl_->num_motors = ack.num_motors > 0 ? ack.num_motors : 6;
            impl_->ack_epoch = ack.sync.epoch;
            const bool first = !impl_->registered.exchange(true);
            if (first)
            {
                std::printf("ecal_io: registered with sim (%u motors, epoch=%u)\n", impl_->num_motors,
                            impl_->ack_epoch);
            }
        });

    impl_->client.set_state_handler(
        [](const mujoco_interface::protocol::state_envelope& state)
        {
            msg::publish(to_raw_state(state), {false});
        });

    impl_->client.set_input_handler(
        [](const mujoco_interface::protocol::input_message& input)
        {
            msg::publish(to_input_snapshot(input), {true});
        });

    impl_->client.set_tick_handler(
        [this](const mujoco_interface::protocol::tick_message& tick)
        {
            if (impl_->registered.load() && tick.sync.epoch != impl_->ack_epoch)
            {
                impl_->registered.store(false);
                try_register();
                return;
            }

            if (!impl_->registered.load())
            {
                try_register();
                return;
            }

            const std::lock_guard<std::mutex> lock(impl_->tick_mutex);
            impl_->latest_tick = tick;
            impl_->latest_tick_received = std::chrono::steady_clock::now();
            impl_->has_pending_tick = true;
            impl_->tick_cv.notify_one();
        });

    try_register();
    impl_->valid.store(true);
}

ecal_io::~ecal_io()
{
    impl_->client.stop();
}

bool ecal_io::valid() const
{
    return impl_->valid.load();
}

void ecal_io::poll()
{
    impl_->client.poll();

    const auto now = std::chrono::steady_clock::now();
    if (impl_->last_register_attempt.time_since_epoch().count() == 0 ||
        now - impl_->last_register_attempt >= std::chrono::seconds(1))
    {
        try_register();
    }
}

bool ecal_io::wait_for_tick_and_commit(std::chrono::steady_clock::duration timeout)
{
    {
        std::unique_lock<std::mutex> lock(impl_->tick_mutex);
        if (!impl_->has_pending_tick)
        {
            impl_->tick_cv.wait_for(lock, timeout,
                                    [this] { return impl_->has_pending_tick || !impl_->registered.load(); });
        }
    }

    const std::uint64_t before = impl_->last_sent_tick_id;
    flush_pending_commit();
    return impl_->last_sent_tick_id != before;
}

void ecal_io::flush_pending_commit()
{
    if (!impl_->registered.load())
    {
        return;
    }

    mujoco_interface::protocol::tick_message tick{};
    std::chrono::steady_clock::time_point tick_received{};
    {
        const std::lock_guard<std::mutex> lock(impl_->tick_mutex);
        if (!impl_->has_pending_tick || impl_->latest_tick.sync.tick_id <= impl_->last_sent_tick_id)
        {
            return;
        }
        tick = impl_->latest_tick;
        tick_received = impl_->latest_tick_received;
        impl_->has_pending_tick = false;
    }

    control::msg_motor_cmd_t motor{};
    {
        const std::lock_guard<std::mutex> lock(impl_->motor_mutex);
        motor = impl_->motor_cmd;
    }

    mujoco_interface::protocol::command_envelope commit{};
    commit.sync = tick.sync;
    commit.sync.session_id = 100;
    commit.sync.client_id = 1;
    commit.body.num_motors = impl_->num_motors;
    for (std::uint32_t i = 0; i < impl_->num_motors && i < 6; ++i)
    {
        commit.body.motors[i].mode = mujoco_interface::robot::mode_torque;
        commit.body.motors[i].tau = motor.tau[i];
    }
    if (impl_->client.send_commit(commit))
    {
        const auto sent_at = std::chrono::steady_clock::now();
        const double tick_to_commit_us =
            std::chrono::duration<double, std::micro>(sent_at - tick_received).count();
        ++impl_->stats_count;
        impl_->stats_sum_tick_to_commit_us += tick_to_commit_us;
        impl_->stats_max_tick_to_commit_us = std::max(impl_->stats_max_tick_to_commit_us, tick_to_commit_us);
        if (impl_->stats_sample_count < impl_->stats_samples_us.size())
        {
            impl_->stats_samples_us[impl_->stats_sample_count++] = tick_to_commit_us;
        }
        if (tick.sync.tick_id > impl_->last_sent_tick_id + 1 && impl_->last_sent_tick_id != 0)
        {
            impl_->stats_missed_ticks += tick.sync.tick_id - impl_->last_sent_tick_id - 1;
        }
        impl_->last_sent_tick_id = tick.sync.tick_id;

        const auto now = sent_at;
        const auto stats_period = impl_->cfg.logger.hz > 0.0f
                                      ? std::chrono::duration<double>(1.0 / static_cast<double>(impl_->cfg.logger.hz))
                                      : std::chrono::duration<double>(1.0);
        if (impl_->cfg.logger.stdout_block && impl_->stats_count > 0 &&
            (impl_->last_stats_print.time_since_epoch().count() == 0 ||
             now - impl_->last_stats_print >=
                 std::chrono::duration_cast<std::chrono::steady_clock::duration>(stats_period)))
        {
            const double avg_us = impl_->stats_sum_tick_to_commit_us / static_cast<double>(impl_->stats_count);
            std::sort(impl_->stats_samples_us.begin(), impl_->stats_samples_us.begin() + impl_->stats_sample_count);
            const auto percentile = [this](double q) {
                if (impl_->stats_sample_count == 0)
                {
                    return 0.0;
                }
                const auto idx =
                    static_cast<std::size_t>(q * static_cast<double>(impl_->stats_sample_count - 1));
                return impl_->stats_samples_us[idx];
            };
            std::printf("ecal_io: tick_commit_us avg=%.1f p50=%.1f p95=%.1f p99=%.1f max=%.1f "
                        "samples=%llu missed_ticks=%llu last_tick=%llu\n",
                        avg_us, percentile(0.50), percentile(0.95), percentile(0.99),
                        impl_->stats_max_tick_to_commit_us, static_cast<unsigned long long>(impl_->stats_count),
                        static_cast<unsigned long long>(impl_->stats_missed_ticks),
                        static_cast<unsigned long long>(impl_->last_sent_tick_id));
            impl_->stats_count = 0;
            impl_->stats_missed_ticks = 0;
            impl_->stats_sum_tick_to_commit_us = 0.0;
            impl_->stats_max_tick_to_commit_us = 0.0;
            impl_->stats_sample_count = 0;
            impl_->last_stats_print = now;
        }
    }
}

void ecal_io::try_register()
{
    impl_->last_register_attempt = std::chrono::steady_clock::now();

    mujoco_interface::protocol::register_message request{};
    request.client_id = 1;
    request.session_id = 100;
    request.motor_begin = 0;
    request.motor_count = 6;

    std::string error;
    if (!impl_->client.register_client(request, error))
    {
        std::fprintf(stderr, "ecal_io: register send failed: %s\n", error.c_str());
    }
}

void ecal_io::update_motor_cmd(const control::msg_motor_cmd_t& motor)
{
    const std::lock_guard<std::mutex> lock(impl_->motor_mutex);
    impl_->motor_cmd = motor;
}

void ecal_io::apply_imu_noise(control::msg_raw_state_t& raw) const
{
    std::normal_distribution<float> gyro_noise(0.0f, impl_->cfg.imu_sim.gyro_noise_std);
    std::normal_distribution<float> accel_noise(0.0f, impl_->cfg.imu_sim.accel_noise_std);

    for (int i = 0; i < 3; ++i)
    {
        raw.gyro[i] += impl_->cfg.imu_sim.gyro_bias[i];
        if (impl_->cfg.imu_sim.gyro_noise_std > 0.0f)
        {
            raw.gyro[i] += gyro_noise(impl_->rng);
        }
        if (impl_->cfg.imu_sim.accel_noise_std > 0.0f)
        {
            raw.accel[i] += accel_noise(impl_->rng);
        }
    }

    if (impl_->cfg.imu_sim.lever_arm_x != 0.0f)
    {
        raw.accel[0] += impl_->cfg.imu_sim.lever_arm_x * raw.gyro[2] * raw.gyro[2];
    }
}

}  // namespace controller
