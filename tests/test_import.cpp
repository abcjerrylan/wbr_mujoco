// Import test: load application robot YAML + MJCF, bind robot::interface, read state.
// Run from wbr_mujoco repo root:
//   ./build/test_import
//   ./build/test_import config/robots/wbr.yaml

#include "mujoco_interface/robot_config.hpp"
#include "mujoco_interface/robot_interface.hpp"

#include <mujoco/mujoco.h>

#include <cstdio>
#include <cstring>
#include <string>

int main(int argc, char** argv)
{
    const std::string config_path = (argc > 1) ? argv[1] : "config/robots/wbr.yaml";
    std::string error;

    mujoco_interface::robot::config config;
    if (!mujoco_interface::robot::load_config(config_path, config, error))
    {
        std::fprintf(stderr, "load_config failed: %s\n", error.c_str());
        return 1;
    }

    const std::string scene =
        mujoco_interface::robot::resolve_path(config_path, config.scene);

    char load_error[1024] = {};
    mjModel* model = mj_loadXML(scene.c_str(), nullptr, load_error, sizeof(load_error));
    if (!model)
    {
        std::fprintf(stderr, "mj_loadXML(%s) failed: %s\n", scene.c_str(), load_error);
        return 1;
    }

    mjData* data = mj_makeData(model);
    mujoco_interface::robot::interface robot;
    if (!robot.load_config(config_path, error) || !robot.bind(model, data, error))
    {
        std::fprintf(stderr, "bind failed: %s\n", error.c_str());
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    const int home_id = mj_name2id(model, mjOBJ_KEY, "home");
    if (home_id >= 0)
    {
        mj_resetDataKeyframe(model, data, home_id);
    }
    mj_forward(model, data);

    mujoco_interface::robot::state state{};
    robot.read_state(state);

    std::printf("test_import ok\n");
    std::printf("  config: %s\n", config_path.c_str());
    std::printf("  scene:  %s\n", scene.c_str());
    std::printf("  robot:  %s\n", config.robot.c_str());
    std::printf("  motors: %u\n", state.num_motors);
    if (state.num_motors > 0)
    {
        std::printf("  motor[0]: q=%.4f dq=%.4f tau=%.4f\n",
                    state.motors[0].q, state.motors[0].dq, state.motors[0].tau_est);
    }

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
