#pragma once

#include "controller/config.hpp"
#include "control/msgs.hpp"
#include "msg/msg.hpp"

#include <atomic>
#include <memory>
#include <thread>

namespace controller
{

class shm_io;

class output_node
{
public:
    output_node(const app_config& cfg, shm_io& io, std::atomic<bool>& running);
    ~output_node();

    output_node(const output_node&) = delete;
    output_node& operator=(const output_node&) = delete;

private:
    void loop();

    const app_config& cfg_;
    shm_io& io_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_motor_cmd_ = msg::subscribe<control::msg_motor_cmd_t>();
};

class imu_node
{
public:
    imu_node(const app_config& cfg, shm_io& io, std::atomic<bool>& running);
    ~imu_node();

    imu_node(const imu_node&) = delete;
    imu_node& operator=(const imu_node&) = delete;

private:
    void loop();

    const app_config& cfg_;
    shm_io& io_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_raw_state_ = msg::subscribe<control::msg_raw_state_t>();
};

class cmd_node
{
public:
    cmd_node(const app_config& cfg, shm_io& io, std::atomic<bool>& running);
    ~cmd_node();

    cmd_node(const cmd_node&) = delete;
    cmd_node& operator=(const cmd_node&) = delete;

private:
    void loop();

    const app_config& cfg_;
    shm_io& io_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_pendulum_ = msg::subscribe<control::msg_pendulum_t>();
    msg::subscriber sub_ins_ = msg::subscribe<control::msg_ins_t>();
};

class control_node
{
public:
    control_node(const app_config& cfg, std::atomic<bool>& running);
    ~control_node();

    control_node(const control_node&) = delete;
    control_node& operator=(const control_node&) = delete;

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_raw_state_ = msg::subscribe<control::msg_raw_state_t>();
    msg::subscriber sub_ins_ = msg::subscribe<control::msg_ins_t>();
    msg::subscriber sub_cmd_ = msg::subscribe<control::msg_cmd_t>();
};

class controller_app
{
public:
    explicit controller_app(const app_config& cfg);
    ~controller_app();
    void run();
    void shutdown();

private:
    app_config cfg_;
    std::atomic<bool> running_{true};
    std::unique_ptr<shm_io> io_;
    output_node output_;
    imu_node imu_;
    cmd_node cmd_;
    control_node control_;
};

}  // namespace controller
