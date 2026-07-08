# wbr_mujoco

[中文说明](README.zh-CN.md)

WBR RoboMaster balance wheel-leg robot — MuJoCo simulation, Shm IPC, and external controller skeleton.

MuJoCo stack and robot binding contract live in the separate submodule **[mujoco_interface](mujoco_interface/)** (`INTERFACE.md` / `INTERFACE.zh-CN.md`).

## Layout

```
common/              robot_msgs (LowState/LowCmd) + robot_ipc (Shm) + msg (in-proc pub/sub)
bridge/              mj_adapter, shm_bridge, app_control (ISimControl hook)
sim/                 sim entry — registers app_control with RunSimulator()
ctrl/                external controller — reads LowState, writes LowCmd over Shm
config/ mjcf/        robot yaml + MJCF assets
mujoco_interface/    submodule — RobotInterface, sim_rt viewer, mj_sim
tests/               test_import, test_adapter, test_shm
```

## Data flow

```
ctrl process                    sim process (physics thread)
    │  write LowCmd ───────────►│ PullCommand (Shm)
    │                           │ WriteCommand → mj ctrl
    │                           │ mj_step()
    │                           │ ReadState ← sensordata
    │◄────── read LowState ─────│ PushState (Shm)
```

- **Observe only** (debug sensors): run `sim` alone with `--print-state`.
- **Closed-loop control**: run `sim` + `ctrl` (or any process that speaks `LowState`/`LowCmd` on Shm).

Motor index order must match `config/robots/*.yaml` `motors:` and `mujoco_interface/INTERFACE.md`.

## Build

```bash
git submodule update --init --recursive
ln -sf /opt/mujoco-3.3.6 mujoco_interface/mujoco   # local MuJoCo install
cmake -B build && cmake --build build
```

## Run

```bash
# Terminal 1 — simulation + viewer
./build/sim -c config/robots/wbr.yaml

# Terminal 2 — external controller (optional)
./build/ctrl --ipc-prefix wbr
```

Debug sensor output on stdout (no ctrl needed):

```bash
./build/sim -c config/robots/wbr.yaml --print-state      # default 5 Hz
./build/sim -c config/robots/wbr.yaml --print-state 10     # 10 Hz
```

Other flags: `--ipc-prefix NAME` (Shm channel prefix, default from yaml).

## Test

```bash
./build/test_import          # yaml + mjcf bind
./build/test_adapter         # mj_adapter conversion
./build/test_shm             # Shm channels
```
