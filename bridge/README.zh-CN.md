# bridge

[English](README.md)

`mujoco_interface` 与应用控制器之间的胶水层。

| 组件 | 作用 |
|------|------|
| `mj_adapter` | `RobotState`/`RobotCommand` ↔ `LowState`/`LowCmd` |
| `shm_bridge` | 基于 Shm 的降采样控制循环 |
| `app_control` | 注册到 `RunSimulator()` 的 `ISimControl` |
| `load_sim_opts` | 读取应用 yaml（`ipc_prefix`、`control.*`） |
