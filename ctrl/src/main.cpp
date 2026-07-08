#include <robot_ipc/channel.hpp>
#include <robot_msgs/types.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace
{

std::atomic<bool> g_running{true};

void HandleSignal(int)
{
    g_running.store(false);
}

void PrintUsage(const char* prog)
{
    std::printf("Usage: %s [--ipc-prefix NAME]\n", prog);
    std::printf("  Connects to mujoco_sim shared-memory channels:\n");
    std::printf("    /NAME_lowstate  (read)\n");
    std::printf("    /NAME_lowcmd    (write)\n");
}

}  // namespace

int main(int argc, char** argv)
{
    std::string ipc_prefix = "wbr";

    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
        {
            PrintUsage(argv[0]);
            return 0;
        }
        if (std::strcmp(argv[i], "--ipc-prefix") == 0 && i + 1 < argc)
        {
            ipc_prefix = argv[++i];
        }
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    robot_ipc::ShmChannel<robot_msgs::LowState> state_in =
        robot_ipc::ShmChannel<robot_msgs::LowState>::open_client(ipc_prefix + "_lowstate");
    robot_ipc::ShmChannel<robot_msgs::LowCmd> cmd_out =
        robot_ipc::ShmChannel<robot_msgs::LowCmd>::open_client(ipc_prefix + "_lowcmd");

    if (!state_in.valid() || !cmd_out.valid())
    {
        std::fprintf(stderr,
                     "Failed to open IPC channels with prefix '%s'. Start mujoco_sim first.\n",
                     ipc_prefix.c_str());
        return 1;
    }

    std::printf("ctrl connected, ipc prefix '%s'. Ctrl+C to exit.\n", ipc_prefix.c_str());
    std::printf("Replace this loop with your own control algorithm.\n");

    while (g_running.load())
    {
        robot_msgs::LowState state{};
        if (state_in.read(state, true, 100) != robot_ipc::channel_status::ok)
        {
            continue;
        }

        robot_msgs::LowCmd cmd{};
        cmd.num_motors = state.num_motors;

        // === User control logic goes here ===
        // Example: zero torque command
        for (std::uint32_t i = 0; i < cmd.num_motors && i < robot_msgs::kMaxMotors; ++i)
        {
            cmd.motors[i].mode = robot_msgs::kMotorModeTorque;
            cmd.motors[i].tau = 0.0f;
        }

        cmd_out.write(cmd);
    }

    return 0;
}
