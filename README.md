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

`cmake` looks for `../mujoco_interface` by default. Override if needed:

```bash
cmake -B build -DMUJOCO_INTERFACE_DIR=/path/to/mujoco_interface
```

This links `ctrl` against `mujoco_interface_core` and also builds the sim server under `build/mujoco_interface/bin/`.  
You can use that binary or the one from a standalone `mujoco_interface/build/` — they are the same target.

## Run

Two terminals, **sim first**:

```bash
# Terminal 1 — simulation server (from mujoco_interface build)
../mujoco_interface/build/bin/mujoco_interface \
  -c config/robots/wbr.yaml

# Terminal 2 — controller (from wbr_mujoco build)
./build/ctrl -c config/robots/wbr.yaml
```

Or use the sim binary produced by wbr_mujoco’s integrated build:

```bash
./build/mujoco_interface/bin/mujoco_interface -c config/robots/wbr.yaml
./build/ctrl -c config/robots/wbr.yaml
```

Headless (no viewer): add `--headless` to the sim command.  
Focus the sim window for keyboard input. YAML `ipc_prefix` sets the eCAL topic namespace (default `wbr`).

