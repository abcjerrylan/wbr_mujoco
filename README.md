# wbr_mujoco

[中文说明](README.zh-CN.md)

Application repo: WBR robot config, bridge, controller. MuJoCo stack lives in `mujoco_interface/`.

## layout

```
common/              msg + robot_msgs + robot_ipc
bridge/              Shm adapter, app_control hook
sim/                 sim entry (registers app_control)
ctrl/                external controller
config/ mjcf/        robot assets
mujoco_interface/    separate repo (mjif + sim_rt + mj_sim)
```

## build

```bash
git submodule update --init --recursive
ln -sf /opt/mujoco-3.3.6 mujoco_interface/mujoco   # local MuJoCo install
cmake -B build && cmake --build build
```

## run

```bash
./build/sim -c config/robots/wbr.yaml
./build/ctrl --ipc-prefix wbr
```

## test

```bash
./build/test_import          # yaml + mjcf bind
./build/test_adapter         # mj_adapter
./build/test_shm             # ipc
```
