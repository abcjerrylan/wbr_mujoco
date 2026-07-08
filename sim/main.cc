#include "mujoco_interface/sim_control.hpp"
#include "bridge/app_control.hpp"

#include <cstring>
#include <cstdio>
#include <string>

namespace
{

void ApplyIpcPrefixOverride(int argc, char** argv, bridge::app_control& control)
{
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--ipc-prefix") == 0 && i + 1 < argc)
        {
            control.SetIpcPrefixOverride(argv[++i]);
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    bridge::app_control control;
    ApplyIpcPrefixOverride(argc, argv, control);
    return mujoco_interface::RunSimulator(argc, argv, &control);
}
