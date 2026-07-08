#pragma once

#include <string>

namespace bridge
{

struct sim_opts
{
    std::string ipc_prefix = "wbr";
    int decimation = 1;
};

bool load_sim_opts(const std::string& config_path, sim_opts& options, std::string& error);

}  // namespace bridge
