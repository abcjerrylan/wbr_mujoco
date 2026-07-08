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
        if (root["control"] && root["control"]["decimation"])
        {
            options.decimation = root["control"]["decimation"].as<int>();
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
