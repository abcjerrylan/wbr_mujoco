# wbr_mujoco

[English](README.md)

应用仓：WBR 机器人配置、bridge、控制器。MuJoCo 栈在 `mujoco_interface/`。

## 目录

```
common/              msg + robot_msgs + robot_ipc
bridge/              Shm 适配、app_control 钩子
sim/                 sim 入口（注册 app_control）
ctrl/                外部控制器
config/ mjcf/        机器人资源
mujoco_interface/    独立仓库（mjif + sim_rt + mj_sim）
```

## 编译

```bash
git submodule update --init --recursive
ln -sf /opt/mujoco-3.3.6 mujoco_interface/mujoco   # 本地 MuJoCo 安装路径
cmake -B build && cmake --build build
```

## 运行

```bash
./build/sim -c config/robots/wbr.yaml
./build/ctrl --ipc-prefix wbr
```

## 测试

```bash
./build/test_import          # yaml + mjcf 绑定
./build/test_adapter         # mj_adapter
./build/test_shm             # ipc
```
