# bridge

[中文说明](README.zh-CN.md)

Glue between `mujoco_interface` and application controllers.

| 组件 | 作用 |
|------|------|
| `mj_adapter` | `RobotState`/`RobotCommand` ↔ `LowState`/`LowCmd` |
| `shm_bridge` | decimated control loop over Shm |
| `app_control` | `ISimControl` hook for `RunSimulator()` |
| `load_sim_opts` | reads app yaml (`ipc_prefix`, `control.*`) |
