#include "robot_ipc/channel.hpp"
#include "robot_msgs/types.hpp"

#include <cstdio>
#include <cstring>

int main()
{
    auto state_srv = robot_ipc::ShmChannel<robot_msgs::LowState>::open_server("/wbr_test_lowstate");
    auto cmd_srv = robot_ipc::ShmChannel<robot_msgs::LowCmd>::open_server("/wbr_test_lowcmd");
    if (!state_srv.valid() || !cmd_srv.valid())
    {
        std::fprintf(stderr, "server open failed\n");
        return 1;
    }

    auto state_cli = robot_ipc::ShmChannel<robot_msgs::LowState>::open_client("/wbr_test_lowstate");
    auto cmd_cli = robot_ipc::ShmChannel<robot_msgs::LowCmd>::open_client("/wbr_test_lowcmd");
    if (!state_cli.valid() || !cmd_cli.valid())
    {
        std::fprintf(stderr, "client open failed\n");
        return 1;
    }

    robot_msgs::LowState state{};
    state.num_motors = 6;
    state.time = 1.25;
    state.motors[0].q = 0.5f;
    state_srv.write(state);

    robot_msgs::LowState read_state{};
    if (state_cli.read(read_state, false) != robot_ipc::channel_status::ok ||
        read_state.num_motors != 6 || read_state.motors[0].q != 0.5f)
    {
        std::fprintf(stderr, "state roundtrip failed\n");
        return 1;
    }

    robot_msgs::LowCmd cmd{};
    cmd.num_motors = 6;
    cmd.motors[0].tau = 2.0f;
    cmd_cli.write(cmd);

    robot_msgs::LowCmd read_cmd{};
    if (cmd_srv.read(read_cmd, false) != robot_ipc::channel_status::ok ||
        read_cmd.motors[0].tau != 2.0f)
    {
        std::fprintf(stderr, "cmd roundtrip failed\n");
        return 1;
    }

    std::printf("test_shm ok\n");
    return 0;
}
