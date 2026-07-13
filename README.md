# wbr_mujoco

WBR RoboMaster balance wheel-leg — controller + robot assets.  
MuJoCo simulation server lives in the separate **[mujoco_interface](https://github.com/CosmosMount/mujoco_interface)** repo.

## Repositories

Clone both **side by side** (sibling directories):

```bash
git clone git@github.com:CosmosMount/wbr_mujoco.git
git clone git@github.com:CosmosMount/mujoco_interface.git
```

Expected layout:

```
code/
├── wbr_mujoco/          # this repo — controller, MJCF, wbr.yaml
└── mujoco_interface/    # sim server + core + eCAL transport
```

## Layout (wbr_mujoco)

```
controller/          control core + ecal_io
common/              in-proc msg bus
config/robots/       wbr.yaml
mjcf/                scene + meshes
tests/               test_import
```

## Build

### One-time setup (mujoco_interface)

```bash
cd ../mujoco_interface
ln -sf /opt/mujoco-3.3.6 mujoco
./scripts/fetch_ecal.sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### Controller + tests (this repo)

```bash
cd wbr_mujoco
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

`cmake` looks for a standalone `../mujoco_interface/build/libmujoco_interface_core.a`
by default. Override paths if needed:

```bash
cmake -B build \
  -DMUJOCO_INTERFACE_DIR=/path/to/mujoco_interface \
  -DMUJOCO_INTERFACE_BUILD_DIR=/path/to/mujoco_interface/build
```

This links `ctrl` against the standalone `mujoco_interface_core` build. It does
not rebuild the sim server from this repo. If you really want the old integrated
build, configure with `-DWBR_EMBED_MUJOCO_INTERFACE=ON`.

## Run

Two terminals, **sim first**:

```bash
# Terminal 1 — simulation server (from mujoco_interface build)
../mujoco_interface/build/mujoco_interface \
  -c config/robots/wbr.yaml

# Terminal 2 — controller (from wbr_mujoco build)
./build/ctrl -c config/robots/wbr.yaml
```

To use the old integrated build mode:

```bash
cmake -B build-integrated -DWBR_EMBED_MUJOCO_INTERFACE=ON
cmake --build build-integrated -j
./build-integrated/mujoco_interface/bin/mujoco_interface -c config/robots/wbr.yaml
./build-integrated/ctrl -c config/robots/wbr.yaml
```

Headless (no viewer): add `--headless` to the sim command.  
Focus the sim window for keyboard input. YAML `ipc_prefix` sets the eCAL topic namespace (default `wbr`).
