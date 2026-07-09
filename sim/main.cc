#include "mujoco_interface/sim_control.hpp"
#include "bridge/app_control.hpp"

#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>

namespace
{

void apply_cli_overrides(int argc, char** argv, bridge::app_control& control)
{
    bool has_robot_config = false;
    bool print_state_set = false;

    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "-c") == 0 || std::strcmp(argv[i], "--config") == 0)
        {
            if (i + 1 < argc)
            {
                has_robot_config = true;
                ++i;
            }
        }
        else if (std::strcmp(argv[i], "--ipc-prefix") == 0 && i + 1 < argc)
        {
            control.set_ipc_prefix_override(argv[++i]);
        }
        else if (std::strcmp(argv[i], "--print-state") == 0)
        {
            print_state_set = true;
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
            control.set_print_state_hz(hz);
        }
        else if (std::strcmp(argv[i], "--no-print-state") == 0)
        {
            print_state_set = true;
            control.set_print_state_hz(0.0);
        }
        else if (std::strcmp(argv[i], "--forward-input") == 0)
        {
            control.set_forward_input(true);
        }
        else if (std::strcmp(argv[i], "--keyboard-trial") == 0)
        {
            control.set_keyboard_trial(true);
        }
        else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
        {
            std::printf("Usage: sim [mujoco options] [--ipc-prefix NAME] [--forward-input] [--keyboard-trial]\n");
            std::printf("  [--print-state [HZ]] [--no-print-state]\n");
            std::printf("  With -c/--config, input forwarding and LowState print (5 Hz) are enabled by default.\n");
            std::printf("  --print-state [HZ]  override print rate (default 5 Hz)\n");
            std::printf("  --no-print-state    disable state printing\n");
            std::exit(0);
        }
    }

    if (has_robot_config)
    {
        control.set_forward_input(true);
        if (!print_state_set)
        {
            control.set_print_state_hz(5.0);
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    bridge::app_control control;
    apply_cli_overrides(argc, argv, control);
    return mujoco_interface::sim::run(argc, argv, &control);
}
