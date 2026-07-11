#pragma once

#include "controller/config.hpp"
#include "control/msgs.hpp"

#include "mujoco_interface/transport/ecal.hpp"

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <random>

namespace controller
{

class ecal_io
{
public:
    explicit ecal_io(const app_config& cfg);
    ~ecal_io();

    [[nodiscard]] bool valid() const { return valid_; }
    void poll();
    bool wait_for_tick_and_commit(std::chrono::steady_clock::duration timeout);
    void update_motor_cmd(const control::msg_motor_cmd_t& motor);
    void apply_imu_noise(control::msg_raw_state_t& raw) const;

    void try_register();
    void flush_pending_commit();

private:
    app_config cfg_;
    mujoco_interface::transport::client client_;
    std::atomic<bool> valid_{false};
    std::atomic<bool> registered_{false};
    std::uint32_t num_motors_ = 6;
    std::uint32_t ack_epoch_ = 0;
    std::mutex motor_mutex_;
    std::mutex tick_mutex_;
    std::condition_variable tick_cv_;
    control::msg_motor_cmd_t motor_cmd_{};
    mujoco_interface::protocol::tick_message latest_tick_{};
    std::chrono::steady_clock::time_point latest_tick_received_{};
    bool has_pending_tick_ = false;
    std::uint64_t last_sent_tick_id_ = 0;
    mutable std::mt19937 rng_{std::random_device{}()};
    std::chrono::steady_clock::time_point last_register_attempt_{};

    std::uint64_t stats_count_ = 0;
    std::uint64_t stats_missed_ticks_ = 0;
    double stats_sum_tick_to_commit_us_ = 0.0;
    double stats_max_tick_to_commit_us_ = 0.0;
    std::array<double, 4096> stats_samples_us_{};
    std::size_t stats_sample_count_ = 0;
    std::chrono::steady_clock::time_point last_stats_print_{};
};

}  // namespace controller
