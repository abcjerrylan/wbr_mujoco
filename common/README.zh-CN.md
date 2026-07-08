# common

WBR 仿真、PC 控制器与嵌入式固件共用的通信组件。

[English](README.md)

| 组件 | 头文件 | 作用 |
|------|--------|------|
| msg | `msg/msg.hpp` | 进程内 pub/sub（同 MCU 或同 PC 进程的多线程） |
| robot_msgs | `robot_msgs/types.hpp` | `LowState` / `LowCmd` —— **与 `mujoco_interface::RobotState` / `RobotCommand` 同布局** |
| robot_ipc | `robot_ipc/channel.hpp` | POSIX 共享内存，仿真 ↔ 外部控制器 |

电机下标顺序必须与 `config/robots/*.yaml` 的 `motors:` 列表及 mujoco_interface 的  
[INTERFACE.zh-CN.md](../mujoco_interface/INTERFACE.zh-CN.md) 一致。
