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

std::string find_config_path(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        if ((std::strcmp(argv[i], "-c") == 0 || std::strcmp(argv[i], "--config") == 0) && i + 1 < argc)
        {
            return argv[i + 1];
        }
    }
    return "config/robots/wbr.yaml";
}

bool file_exists(const std::string& path)
{
    std::ifstream file(path);
    return file.good();
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

    parse_args(argc, argv, cfg);
    return cfg;
}

}  // namespace controller
