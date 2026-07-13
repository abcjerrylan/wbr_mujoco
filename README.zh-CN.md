# wbr_mujoco

[English](README.md)

WBR 平衡轮腿 — **控制器 + 机器人资源**。  
MuJoCo 仿真服务端在独立仓库 **[mujoco_interface](https://github.com/CosmosMount/mujoco_interface)**。

## 两个仓库

并排克隆（同级目录）：

```bash
git clone git@github.com:CosmosMount/wbr_mujoco.git
git clone git@github.com:CosmosMount/mujoco_interface.git
```

目录结构：

```
code/
├── wbr_mujoco/          # 本仓库 — 控制器、MJCF、wbr.yaml
└── mujoco_interface/    # 仿真服务端 + core + eCAL
```

## 本仓库目录

```
controller/          控制核心 + ecal_io
common/              进程内 msg bus
config/robots/       wbr.yaml
mjcf/                场景与 mesh
tests/               test_import
```

## 编译

### 一次性准备（mujoco_interface）

```bash
cd ../mujoco_interface
ln -sf /opt/mujoco-3.3.6 mujoco
./scripts/fetch_ecal.sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### 控制器（本仓库）

```bash
cd wbr_mujoco
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

默认使用独立编译好的 `../mujoco_interface/build/libmujoco_interface_core.a`；
路径可手动指定：

```bash
cmake -B build \
  -DMUJOCO_INTERFACE_DIR=/path/to/mujoco_interface \
  -DMUJOCO_INTERFACE_BUILD_DIR=/path/to/mujoco_interface/build
```

这只会把控制器链接到独立编译的 `mujoco_interface_core`，不会在本仓库里重新编译 sim。
如果确实需要旧的集成编译模式，可加 `-DWBR_EMBED_MUJOCO_INTERFACE=ON`。

## 运行

两个终端，**先开 sim**：

```bash
# 终端 1 — 仿真（mujoco_interface 独立编译）
../mujoco_interface/build/mujoco_interface \
  -c config/robots/wbr.yaml

# 终端 2 — 控制器
./build/ctrl -c config/robots/wbr.yaml
```

如需使用旧的集成编译模式：

```bash
cmake -B build-integrated -DWBR_EMBED_MUJOCO_INTERFACE=ON
cmake --build build-integrated -j
./build-integrated/mujoco_interface/bin/mujoco_interface -c config/robots/wbr.yaml
./build-integrated/ctrl -c config/robots/wbr.yaml
```

无窗口：sim 加 `--headless`。键盘需聚焦 sim 窗口。  
YAML 中 `ipc_prefix` 为 eCAL topic 命名空间（默认 `wbr`）。

## 测试

```bash
./build/test_import
./build/test_import config/robots/wbr.yaml
ctest --test-dir ../mujoco_interface/build --output-on-failure
```

## 常见问题

**找不到 mujoco_interface** — 与 `wbr_mujoco` 同级克隆，或 `-DMUJOCO_INTERFACE_DIR=...`。

**旧 build 缓存** — 换路径后：`rm -rf build && cmake -B build`。

**mujoco_interface 的 git 损坏**（子模块拆出后 `.git` 仍指向旧路径）— 在 `mujoco_interface` 目录运行：

```bash
./scripts/fix_git_remote.sh
```

**wbr_mujoco 索引里仍有 mujoco_interface gitlink** — 运行：

```bash
./scripts/fix_mujoco_interface_gitlink.sh
# 然后 git commit 记录子模块移除
```
