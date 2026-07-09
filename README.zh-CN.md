# wbr_mujoco

[English](README.md)

WBR 平衡轮腿 RoboMaster 机器人 — MuJoCo 仿真、Shm IPC、外部控制器骨架。

MuJoCo 栈与机器人绑定契约在独立子模块 **[mujoco_interface](mujoco_interface/)**（见 `INTERFACE.md` / `INTERFACE.zh-CN.md`）。

## 目录

```
common/              robot_msgs（LowState/LowCmd）+ robot_ipc（Shm）+ msg（进程内 pub/sub）
bridge/              mj_adapter、shm_bridge、app_control（sim_control 钩子）
sim/                 sim 入口 — 向 run_simulator() 注册 app_control
ctrl/                外部控制器 — 读 LowState、写 LowCmd（Shm）
config/ mjcf/        机器人 yaml + MJCF 资源
mujoco_interface/    子模块 — robot_interface、sim_rt viewer、mj_sim
tests/               test_import、test_adapter、test_shm
```

## 数据流

```
ctrl 进程                       sim 进程（physics 线程）
    │  write LowCmd ───────────►│ PullCommand（Shm）
    │                           │ write_command → mj ctrl
    │                           │ mj_step()
    │                           │ read_state ← sensordata
    │◄────── read LowState ─────│ PushState（Shm）
```

- **只观测**（调试 sensor）：单跑 `sim`，加 `--print-state`。
- **闭环控制**：`sim` + `ctrl`（或任意能读写 Shm 上 `LowState`/`LowCmd` 的进程）。

电机下标顺序须与 `config/robots/*.yaml` 的 `motors:` 及 `mujoco_interface/INTERFACE.md` 一致。

## 编译

```bash
git submodule update --init --recursive
ln -sf /opt/mujoco-3.3.6 mujoco_interface/mujoco   # 本地 MuJoCo 安装路径
cmake -B build && cmake --build build
```

## 运行

```bash
# 终端 1 — 仿真 + viewer
./build/sim -c config/robots/wbr.yaml

# 终端 2 — 外部控制器（可选）
./build/ctrl --ipc-prefix wbr
```

命令行打印 sensor（无需 ctrl）：

```bash
./build/sim -c config/robots/wbr.yaml --print-state      # 默认 5 Hz
./build/sim -c config/robots/wbr.yaml --print-state 10     # 10 Hz
```

其它参数：`--ipc-prefix NAME`（Shm 通道前缀，默认取自 yaml）。

## 测试

```bash
./build/test_import          # yaml + mjcf 绑定
./build/test_adapter         # mj_adapter 转换
./build/test_shm             # Shm 通道
```
