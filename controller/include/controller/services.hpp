#pragma once

#include "controller/config.hpp"
#include "control/msgs.hpp"
#include "msg/msg.hpp"

#include <atomic>
#include <thread>

namespace controller
{

class ecal_io;

class actuator_service
{
public:
    actuator_service(const app_config& cfg, ecal_io& io, std::atomic<bool>& running);
    ~actuator_service();

private:
    void loop();

    const app_config& cfg_;
    ecal_io& io_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_motor_cmd_ = msg::subscribe<control::msg_motor_cmd_t>();
    control::msg_motor_cmd_t motor_cmd_{};
};

class ins_service
{
public:
    ins_service(const app_config& cfg, ecal_io& io, std::atomic<bool>& running);
    ~ins_service();

private:
    void loop();

    const app_config& cfg_;
    ecal_io& io_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_raw_state_ = msg::subscribe<control::msg_raw_state_t>();
};

class command_service
{
public:
    command_service(const app_config& cfg, std::atomic<bool>& running);
    ~command_service();

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_pendulum_ = msg::subscribe<control::msg_pendulum_t>();
    msg::subscriber sub_ins_ = msg::subscribe<control::msg_ins_t>();
    msg::subscriber sub_input_ = msg::subscribe<control::input_snapshot_t>();
    control::input_snapshot_t input_{};
};

class chassis_service
{
public:
    chassis_service(const app_config& cfg, std::atomic<bool>& running);
    ~chassis_service();

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_raw_state_ = msg::subscribe<control::msg_raw_state_t>();
    msg::subscriber sub_ins_ = msg::subscribe<control::msg_ins_t>();
    msg::subscriber sub_cmd_ = msg::subscribe<control::msg_cmd_t>();
};

class sim_log_service
{
public:
    sim_log_service(const app_config& cfg, std::atomic<bool>& running);
    ~sim_log_service();

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_log_ = msg::subscribe<control::msg_log_t>();
};

class web_visualizer_service
{
public:
    web_visualizer_service(const app_config& cfg, std::atomic<bool>& running);
    ~web_visualizer_service();

private:
    void loop();

    const app_config& cfg_;
    std::atomic<bool>& running_;
    std::thread thread_;
    msg::subscriber sub_log_ = msg::subscribe<control::msg_log_t>();
};

}  // namespace controller
