#include "bridge/sim_config.hpp"

#include <yaml-cpp/yaml.h>

namespace bridge
{

bool load_sim_opts(const std::string& config_path, sim_opts& options, std::string& error)
{
    try
    {
        const YAML::Node root = YAML::LoadFile(config_path);
        if (root["ipc_prefix"])
        {
            options.ipc_prefix = root["ipc_prefix"].as<std::string>();
        }
        if (root["control"])
        {
            const YAML::Node control = root["control"];
            if (control["decimation"])
            {
                options.decimation = control["decimation"].as<int>();
            }
            if (control["default_mode"])
            {
                options.default_mode = control["default_mode"].as<std::string>();
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

}  // namespace bridge
