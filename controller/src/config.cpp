#include "controller/config.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace controller
{

namespace
{

bool file_exists(const std::string& path)
{
    std::ifstream file(path);
    return file.good();
}

std::string find_config_path(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        if ((std::strcmp(argv[i], "-c") == 0 || std::strcmp(argv[i], "--config") == 0) && i + 1 < argc)
        {
            return argv[i + 1];
        }
    }

    const char* candidates[] = {
        "config/robots/wbr.yaml",
        "../config/robots/wbr.yaml",
        "../../config/robots/wbr.yaml",
    };
    for (const char* candidate : candidates)
    {
        if (file_exists(candidate))
        {
            return candidate;
        }
    }
    return "config/robots/wbr.yaml";
}

control::pid_mode parse_pid_mode(const std::string& mode)
{
    return mode == "dvel" ? control::pid_mode::dvel : control::pid_mode::position;
}

void parse_pid_params(const YAML::Node& node, control::pid_params& params)
{
    if (!node)
    {
        return;
    }
    if (node["kp"])
    {
        params.kp = node["kp"].as<float>();
    }
    if (node["ki"])
    {
        params.ki = node["ki"].as<float>();
    }
    if (node["kd"])
    {
        params.kd = node["kd"].as<float>();
    }
    if (node["max_out"])
    {
        params.max_out = node["max_out"].as<float>();
    }
    if (node["max_i"])
    {
        params.max_i_out = node["max_i"].as<float>();
    }
    if (node["mode"])
    {
        params.mode = parse_pid_mode(node["mode"].as<std::string>());
    }
}

void parse_phi_state_pid(const YAML::Node& node, control::phi_state_pid& params)
{
    if (!node)
    {
        return;
    }
    if (node["kp"])
    {
        params.kp = node["kp"].as<float>();
    }
    if (node["kd"])
    {
        params.kd = node["kd"].as<float>();
    }
    if (node["slope"])
    {
        params.slope = node["slope"].as<float>();
    }
}

void parse_fsm_guards(const YAML::Node& node, control::fsm_guards& guards)
{
    if (!node)
    {
        return;
    }
    if (node["enter_normal_alpha"])
    {
        guards.enter_normal_alpha = node["enter_normal_alpha"].as<float>();
    }
    if (node["enter_normal_ticks"])
    {
        guards.enter_normal_ticks = node["enter_normal_ticks"].as<std::uint32_t>();
    }
    if (node["exit_relax_alpha"])
    {
        guards.exit_relax_alpha = node["exit_relax_alpha"].as<float>();
    }
    if (node["exit_relax_ticks"])
    {
        guards.exit_relax_ticks = node["exit_relax_ticks"].as<std::uint32_t>();
    }
    if (node["offground_min_air_ticks"])
    {
        guards.offground_min_air_ticks = node["offground_min_air_ticks"].as<std::uint32_t>();
    }
}

void parse_pid_config(const YAML::Node& pid_node, control::chassis_config& chassis)
{
    if (!pid_node)
    {
        return;
    }

    if (pid_node["leg"])
    {
        const YAML::Node leg = pid_node["leg"];
        parse_pid_params(leg["len"], chassis.leg_pid.len);
        parse_pid_params(leg["phi"], chassis.leg_pid.phi);
    }

    if (pid_node["roll"])
    {
        parse_pid_params(pid_node["roll"], chassis.fsm_pid.roll);
    }

    if (pid_node["len_balance"])
    {
        parse_pid_params(pid_node["len_balance"], chassis.fsm_pid.len_balance);
    }

    if (pid_node["recover"])
    {
        const YAML::Node recover = pid_node["recover"];
        parse_phi_state_pid(recover, chassis.fsm_pid.recover);
        if (recover["kick_torque"])
        {
            chassis.fsm_pid.recover_kick_torque = recover["kick_torque"].as<float>();
        }
    }

    if (pid_node["flatten"])
    {
        const YAML::Node flatten = pid_node["flatten"];
        parse_phi_state_pid(flatten["high_phi"], chassis.fsm_pid.flatten_high);
        parse_phi_state_pid(flatten["low_phi"], chassis.fsm_pid.flatten_low);
    }

    if (pid_node["neutral"])
    {
        parse_phi_state_pid(pid_node["neutral"], chassis.fsm_pid.neutral);
    }
}

void parse_args(int argc, char** argv, app_config& cfg)
{
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
        {
            std::printf("Usage: %s [-c PATH] [--ipc-prefix NAME] [--imu-mode bypass|mahony]\n"
                        "       [--gyro-noise STD] [--accel-noise STD] [--lever-arm-x VAL]\n"
                        "       [--log-hz HZ] [--no-log]\n",
                        argv[0]);
            std::exit(0);
        }
        else if ((std::strcmp(argv[i], "-c") == 0 || std::strcmp(argv[i], "--config") == 0) && i + 1 < argc)
        {
            ++i;
        }
        else if (std::strcmp(argv[i], "--ipc-prefix") == 0 && i + 1 < argc)
        {
            cfg.ipc_prefix = argv[++i];
        }
        else if (std::strcmp(argv[i], "--imu-mode") == 0 && i + 1 < argc)
        {
            const char* mode = argv[++i];
            cfg.imu_mode =
                std::strcmp(mode, "bypass") == 0 ? control::imu_mode::bypass : control::imu_mode::mahony;
        }
        else if (std::strcmp(argv[i], "--gyro-noise") == 0 && i + 1 < argc)
        {
            cfg.imu_sim.gyro_noise_std = std::strtof(argv[++i], nullptr);
        }
        else if (std::strcmp(argv[i], "--accel-noise") == 0 && i + 1 < argc)
        {
            cfg.imu_sim.accel_noise_std = std::strtof(argv[++i], nullptr);
        }
        else if (std::strcmp(argv[i], "--lever-arm-x") == 0 && i + 1 < argc)
        {
            cfg.imu_sim.lever_arm_x = std::strtof(argv[++i], nullptr);
        }
        else if (std::strcmp(argv[i], "--log-hz") == 0 && i + 1 < argc)
        {
            cfg.logger.hz = std::strtof(argv[++i], nullptr);
        }
        else if (std::strcmp(argv[i], "--no-log") == 0)
        {
            cfg.logger.stdout_block = false;
        }
    }
}

bool load_yaml(const std::string& path, app_config& cfg, std::string& error)
{
    try
    {
        const YAML::Node root = YAML::LoadFile(path);

        if (root["ipc_prefix"])
        {
            cfg.ipc_prefix = root["ipc_prefix"].as<std::string>();
        }

        if (root["timestep"] && root["control"] && root["control"]["decimation"])
        {
            const float timestep = root["timestep"].as<float>();
            const int decimation = root["control"]["decimation"].as<int>();
            if (timestep > 0.0f && decimation > 0)
            {
                cfg.control_hz = 1.0f / (timestep * static_cast<float>(decimation));
            }
        }

        const YAML::Node control_node = root["control"];
        if (control_node && control_node["force_relax"])
        {
            cfg.chassis.force_relax = control_node["force_relax"].as<bool>();
        }
        if (control_node && control_node["motor_zero_rad"] && control_node["motor_zero_rad"].IsSequence())
        {
            const YAML::Node z = control_node["motor_zero_rad"];
            for (std::size_t i = 0; i < 6 && i < z.size(); ++i)
            {
                cfg.chassis.motor_zero_rad[i] = z[i].as<float>();
            }
        }
        if (control_node && control_node["pid"])
        {
            parse_pid_config(control_node["pid"], cfg.chassis);
        }
        if (control_node && control_node["fsm"])
        {
            parse_fsm_guards(control_node["fsm"], cfg.chassis.fsm);
        }

        const YAML::Node log_node = root["logger"];
        if (log_node)
        {
            if (log_node["hz"])
            {
                cfg.logger.hz = log_node["hz"].as<float>();
            }
            if (log_node["stdout"])
            {
                const std::string mode = log_node["stdout"].as<std::string>();
                cfg.logger.stdout_block =
                    mode == "block" || mode == "true" || mode == "on" || mode == "1";
            }
        }

        return true;
    }
    catch (const std::exception& ex)
    {
        error = ex.what();
        return false;
    }
}

}  // namespace

app_config load_config(int argc, char** argv)
{
    app_config cfg;
    const std::string config_path = find_config_path(argc, argv);

    if (file_exists(config_path))
    {
        std::string error;
        if (!load_yaml(config_path, cfg, error))
        {
            std::fprintf(stderr, "failed to load config %s: %s\n", config_path.c_str(), error.c_str());
            std::exit(1);
        }
    }
    else
    {
        std::fprintf(stderr,
                     "warning: config not found (%s); using defaults (logger off). "
                     "Run from repo root or pass -c config/robots/wbr.yaml\n",
                     config_path.c_str());
    }

    parse_args(argc, argv, cfg);
    return cfg;
}

}  // namespace controller
