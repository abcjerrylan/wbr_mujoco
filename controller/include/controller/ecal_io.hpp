#pragma once

#include "controller/config.hpp"
#include "control/msgs.hpp"

#include <chrono>
#include <memory>

namespace controller
{

class ecal_io
{
public:
    explicit ecal_io(const app_config& cfg);
    ~ecal_io();

    [[nodiscard]] bool valid() const;
    void poll();
    bool wait_for_tick_and_commit(std::chrono::steady_clock::duration timeout);
    void update_motor_cmd(const control::msg_motor_cmd_t& motor);
    void apply_imu_noise(control::msg_raw_state_t& raw) const;

    void try_register();
    void flush_pending_commit();

private:
    struct impl;
    std::unique_ptr<impl> impl_;
};

}  // namespace controller
