#pragma once

#include "control/config.hpp"

#include <string>

namespace controller
{

struct logger_config
{
    bool stdout_block = false;
    float hz = 5.0f;
};

struct app_config
{
    std::string ipc_prefix = "wbr";
    control::chassis_config chassis{};
    control::imu_sim_config imu_sim{};
    control::imu_mode imu_mode = control::imu_mode::mahony;
    float control_hz = 1000.0f;
    float motor_sign_cmd[6] = {-1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    logger_config logger{};
};

app_config load_config(int argc, char** argv);

}  // namespace controller
