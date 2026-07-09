#include "mujoco_interface/robot_interface.hpp"
#include "bridge/mj_adapter.hpp"

#include <mujoco/mujoco.h>

#include <cstdio>
#include <string>

int main()
{
    const std::string config_path = "config/robots/wbr.yaml";
    std::string error;

    mujoco_interface::robot_interface robot;
    mujoco_interface::robot_config config;
    if (!mujoco_interface::load_robot_config(config_path, config, error))
    {
        std::fprintf(stderr, "load_robot_config failed: %s\n", error.c_str());
        return 1;
    }

    const std::string scene =
        mujoco_interface::resolve_path(config_path, config.scene);

    char load_error[1024] = {};
    mjModel* model = mj_loadXML(scene.c_str(), nullptr, load_error, sizeof(load_error));
    if (!model)
    {
        std::fprintf(stderr, "mj_loadXML failed: %s\n", load_error);
        return 1;
    }

    mjData* data = mj_makeData(model);
    if (!robot.load_config(config_path, error) || !robot.bind(model, data, error))
    {
        std::fprintf(stderr, "bind failed: %s\n", error.c_str());
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    bridge::mj_adapter adapter{robot};
    robot_msgs::LowState state{};
    adapter.read_state(state);

    if (adapter.num_motors() != 6 || state.num_motors != 6)
    {
        std::fprintf(stderr, "adapter motor count mismatch\n");
        return 1;
    }

    std::printf("test_adapter ok: num_motors=%u\n", state.num_motors);

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
