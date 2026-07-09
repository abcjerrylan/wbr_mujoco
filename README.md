# wbr_mujoco

[中文说明](README.zh-CN.md)

WBR RoboMaster balance wheel-leg robot — MuJoCo simulation, Shm IPC, and external controller skeleton.

MuJoCo stack and robot binding contract live in the separate submodule **[mujoco_interface](mujoco_interface/)** (see `README.md` / `README.zh-CN.md`).

## Layout

```
common/              robot_msgs (LowState/LowCmd) + robot_ipc (Shm) + msg (in-proc pub/sub)
bridge/              mj_adapter, shm_bridge, app_control (sim::control hook)
sim/                 sim entry — registers app_control, calls sim::run()
ctrl/                external controller — reads LowState, writes LowCmd over Shm
config/ mjcf/        robot yaml + MJCF assets
mujoco_interface/    submodule — robot::, input::, sim_rt (libs + sim::run API)
tests/               test_import, test_adapter, test_shm
```

## Entry points

| layer | role |
|-------|------|
| `mujoco_interface` | generic MuJoCo stack: `sim::run()`, `robot::interface`, `input::snapshot` |
| `bridge/` | app adapters: `mj_adapter` (`robot_msgs` ↔ `robot::`), `shm_bridge`, `app_control` |
| `sim/main.cc` | **composition root**: builds `bridge::app_control`, parses `--ipc-prefix`, etc., then `sim::run()` |

With an external controller, always launch **`./build/sim`** — not a submodule `main` (`mj_sim` was removed).  
Message layout, Shm prefix, and print policy live in `bridge/` (this repo), not in the generic submodule.

```
ctrl (robot_ipc)  ←Shm→  sim process
                         sim::run(&app_control)
                         app_control → shm_bridge → mj_adapter → robot::interface → MuJoCo
```

## Data flow

```
ctrl process                    sim process (physics thread)
    │  write LowCmd ───────────►│ PullCommand (Shm)
    │                           │ write_command → mj ctrl
    │                           │ mj_step()
    │                           │ read_state ← sensordata
    │◄────── read LowState ─────│ PushState (Shm)
```

- **Observe only** (debug sensors): run `sim` alone with `--print-state`.
- **Closed-loop control**: run `sim` + `ctrl` (or any process that speaks `LowState`/`LowCmd` on Shm).

Motor index order must match `config/robots/*.yaml` `motors:` and `mujoco_interface/README.md`.

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
