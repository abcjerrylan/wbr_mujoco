# wbr_mujoco

WBR RoboMaster balance wheel-leg controller, MuJoCo robot model, runtime config,
and control-tuning tools.

This repository is a downstream controller project. Simulation runtime is
provided by an installed `mujoco_interface` package; this repository owns only
the WBR-specific controller, YAML config, MJCF scene, meshes, and tuning tools.

## Layout

```text
controller/              WBR controller app, eCAL I/O, control code
common/                  small in-process message utilities
config/robots/wbr.yaml   WBR runtime config for simulator and controller
mjcf/                    WBR scene, model, and mesh assets
tools/                   mesh repair and LQR fitting utilities
```

## Requirements

- CMake 3.16+
- C++17 compiler
- `yaml-cpp`
- `glfw3`
- installed `mujoco_interface` release package

Install `mujoco_interface` first, for example:

```bash
cmake --install /path/to/mujoco_interface/build --prefix /opt/mujoco_interface
```

## Build

Build this repository against the installed interface package:

```bash
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/mujoco_interface
cmake --build build -j
```

This builds:

```text
build/ctrl
```

`wbr_mujoco` does not build or vendor the simulator. It consumes
`mujoco_interface::mujoco_interface_core` from the installed package.

## Run

Use two terminals. Start the simulator first, then the controller.

Terminal 1 — simulator:

```bash
/opt/mujoco_interface/bin/mujoco_interface \
  -c /absolute/path/to/wbr_mujoco/config/robots/wbr.yaml
```

Headless simulator:

```bash
/opt/mujoco_interface/bin/mujoco_interface \
  -c /absolute/path/to/wbr_mujoco/config/robots/wbr.yaml \
  --headless
```

Terminal 2 — controller:

```bash
./build/ctrl -c config/robots/wbr.yaml
```

The simulator and controller must use the same YAML config, or at least matching
`ipc_prefix` values. The default WBR config uses:

```yaml
ipc_prefix: wbr
timestep: 0.001
```

The default timestep is 1 kHz. `mujoco_interface` prints realtime-rate metrics by
default; a healthy headless run should report `total_real_time_rate` close to
1.0.

## `tools/` Python scripts

### `tools/fix_stl_for_mujoco.py`

Repairs STL meshes so MuJoCo can load them. It can:

- convert ASCII STL files to binary STL;
- decimate meshes above MuJoCo's 200,000-face limit;
- replace empty 0-face marker meshes with a tiny tetrahedron;
- optionally verify meshes with the Python `mujoco` package.

Dependencies:

```bash
python3 -m pip install numpy trimesh pymeshlab mujoco
```

Usage:

```bash
# Show planned changes
python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes --dry-run

# Repair meshes and keep backups in mjcf/meshes/stl_backup
python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes

# Use a custom backup directory
python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes --backup /tmp/wbr_stl_backup

# Overwrite without backups
python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes --no-backup
```

### `tools/lqr/fit_lqr.py`

Fits polynomial LQR gains for the wheel-leg balance controller and can update:

```text
controller/include/control/lqr_coeffs.hpp
```

Usage:

```bash
# Print generated C++ arrays without writing files
python3 tools/lqr/fit_lqr.py

# Refit gains and update lqr_coeffs.hpp
python3 tools/lqr/fit_lqr.py --write

# Scale the existing header without a full refit
python3 tools/lqr/fit_lqr.py --scale 0.62 --write
```

Dependencies:

- `--scale`: Python 3 + NumPy
- full refit: Python 3 + NumPy + SciPy

Tune `low`, `high`, and `spin` weights in `LQR_WEIGHTS` inside
`tools/lqr/fit_lqr.py`.

### `tools/lqr/gen_matrices.py`

Regenerates generated matrix helpers from MATLAB exports:

```bash
python3 tools/lqr/gen_matrices.py
```

Inputs:

```text
tools/lqr/vendor/Amatrix.m
tools/lqr/vendor/Bmatrix.m
```

Outputs:

```text
tools/lqr/amatrix.py
tools/lqr/bmatrix.py
```

Supporting modules used by `fit_lqr.py`:

```text
tools/lqr/dynamics.py   builds A/B matrices from generated symbolic functions
tools/lqr/leg_data.py   leg length, COM, and inertia table
tools/lqr/amatrix.py    generated symbolic A helper
tools/lqr/bmatrix.py    generated symbolic B helper
```

## Troubleshooting

- CMake cannot find `mujoco_interface`: pass
  `-DCMAKE_PREFIX_PATH=/path/to/mujoco_interface/install`.
- Simulator and controller do not connect: check that both use matching
  `ipc_prefix` values.
- Mesh loading fails: run `python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes`.
- No GUI needed or no display available: run the simulator with `--headless`.
