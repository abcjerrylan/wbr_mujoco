#include "mujoco_interface/sim_control.hpp"
#include "bridge/app_control.hpp"

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>

namespace
{

void ApplyCliOverrides(int argc, char** argv, bridge::app_control& control)
{
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--ipc-prefix") == 0 && i + 1 < argc)
        {
            control.SetIpcPrefixOverride(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--print-state") == 0)
        {
            double hz = 5.0;
            if (i + 1 < argc && argv[i + 1][0] != '-')
            {
                char* end = nullptr;
                const double parsed = std::strtod(argv[i + 1], &end);
                if (end != argv[i + 1] && parsed > 0.0)
                {
                    hz = parsed;
                    ++i;
                }
            }
            control.SetPrintStateHz(hz);
        }
        else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
        {
            std::printf("Usage: sim [mujoco options] [--ipc-prefix NAME] [--print-state [HZ]]\n");
            std::printf("  --print-state [HZ]  print LowState to stdout (default 5 Hz)\n");
            std::exit(0);
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    bridge::app_control control;
    ApplyCliOverrides(argc, argv, control);
    return mujoco_interface::RunSimulator(argc, argv, &control);
}
