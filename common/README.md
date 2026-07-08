# common

[中文说明](README.zh-CN.md)

Shared communication primitives for WBR sim, PC controller, and embedded firmware.

| Component | Header | Role |
|-----------|--------|------|
| msg | `msg/msg.hpp` | In-process pub/sub (threads on same MCU or PC process) |
| robot_msgs | `robot_msgs/types.hpp` | `LowState` / `LowCmd` — **same layout as `mujoco_interface::RobotState` / `RobotCommand`** |
| robot_ipc | `robot_ipc/channel.hpp` | POSIX shared memory for sim ↔ external controller |

Motor index order must match `config/robots/*.yaml` `motors:` list and mujoco_interface
`INTERFACE.md`.
