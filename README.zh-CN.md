# wbr_mujoco

[English](README.md)

WBR 平衡轮腿 RoboMaster 机器人 — MuJoCo 仿真、Shm IPC、外部控制器骨架。

MuJoCo 栈与机器人绑定契约在独立子模块 **[mujoco_interface](mujoco_interface/)**（见 `README.md` / `README.zh-CN.md`）。

## 目录

```
common/              robot_msgs（LowState/LowCmd）+ robot_ipc（Shm）+ msg（进程内 pub/sub）
bridge/              mj_adapter、shm_bridge、app_control（sim::control 钩子）
sim/                 sim 入口 — 注册 app_control，调用 sim::run()
ctrl/                外部控制器 — 读 LowState、写 LowCmd（Shm）
config/ mjcf/        机器人 yaml + MJCF 资源
mujoco_interface/    子模块 — robot::、input::、sim_rt（库 + sim::run API）
tests/               test_import、test_adapter、test_shm
```

## 入口分工

| 层 | 职责 |
|----|------|
| `mujoco_interface` | 通用 MuJoCo 栈：`sim::run()`、`robot::interface`、`input::snapshot` |
| `bridge/` | 应用适配：`mj_adapter`（`robot_msgs` ↔ `robot::`）、`shm_bridge`、`app_control` |
| `sim/main.cc` | **组合根**：创建 `bridge::app_control`，解析 `--ipc-prefix` 等，再调 `sim::run()` |

有外部 controller 时仍启动 **`./build/sim`**，不要单独跑子模块里的 main（已删除 `mj_sim`）。  
消息格式、Shm 前缀、打印策略等都在 `bridge/`，属于本仓库而非通用子模块。

```
ctrl (robot_ipc)  ←Shm→  sim 进程
                         sim::run(&app_control)
                         app_control → shm_bridge → mj_adapter → robot::interface → MuJoCo
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

电机下标顺序须与 `config/robots/*.yaml` 的 `motors:` 及 `mujoco_interface/README.zh-CN.md` 一致。

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
