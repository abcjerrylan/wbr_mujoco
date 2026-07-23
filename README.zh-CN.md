# wbr_mujoco

[English](README.md)

WBR RoboMaster 平衡轮腿控制器、MuJoCo 机器人模型、运行配置和控制调参工具。

本仓库是下游控制器项目。仿真 runtime 由已安装的 `mujoco_interface` package
提供；本仓库只维护 WBR 专属的控制器、YAML config、MJCF scene、meshes 和调参工具。

## 目录

```text
controller/              WBR 控制器应用、eCAL I/O、控制代码
common/                  小型进程内消息工具
config/robots/wbr.yaml   simulator 和 controller 共用的 WBR 运行配置
mjcf/                    WBR scene、model、mesh 资源
tools/                   mesh 修复与 LQR 拟合工具
```

## 依赖

- CMake 3.16+
- C++17 编译器
- `yaml-cpp`
- `glfw3`
- 已安装的 `mujoco_interface` release package

先安装 `mujoco_interface`，例如：

```bash
cmake --install /path/to/mujoco_interface/build --prefix /opt/mujoco_interface
```

## 编译

使用安装后的 interface package 编译本仓库：

```bash
cmake -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/opt/mujoco_interface
cmake --build build -j
```

生成：

```text
build/ctrl
```

`wbr_mujoco` 不编译、不 vendored simulator；它只消费安装包中的
`mujoco_interface::mujoco_interface_core`。

## 运行

使用两个终端。先启动 simulator，再启动 controller。

终端 1 — simulator：

```bash
mujoco_interface \
  -c /absolute/path/to/wbr_mujoco/config/robots/wbr.yaml
```

Headless simulator：

```bash
mujoco_interface \
  -c /absolute/path/to/wbr_mujoco/config/robots/wbr.yaml \
  --headless
```

终端 2 — controller：

```bash
./build/ctrl -c config/robots/wbr.yaml
```

simulator 和 controller 必须使用同一份 YAML，或至少使用匹配的 `ipc_prefix`。
默认 WBR 配置为：

```yaml
ipc_prefix: wbr
timestep: 0.001
```

默认 timestep 为 1 kHz。`mujoco_interface` 默认输出 realtime-rate 指标；
正常 headless 运行时 `total_real_time_rate` 应接近 1.0。

## 键位说明

使用键盘控制前，请先让 simulator 窗口获得焦点。controller 从 `mujoco_interface`
读取键盘状态快照。

| 键位 | 功能 |
| --- | --- |
| `Space` | 切换运动使能/关闭 |
| `W` | 增大前进速度命令 |
| `S` | 增大后退速度命令 |
| `A` | 增大左转 yaw-rate 命令 |
| `D` | 增大右转 yaw-rate 命令 |
| `Q` | 将腿长目标设为低位（`lmin`） |
| `E` | 将腿长目标设为中位（`lmid`） |
| `F` | 将腿长目标设为高位（`lmax`） |

松开 `W`/`S` 后，速度命令会向零渐变。松开 `A`/`D` 后，yaw-rate 命令清零，
yaw 参考会重置为当前 yaw。

## 可视化

`build/ctrl` 启动时会同时启动一个内置 Web 可视化服务。浏览器打开：

```text
http://localhost:2000
```

页面通过 `/events` 的 Server-Sent Events 订阅 controller 进程内的 `msg_log_t`
数据流，并按组实时绘制关键观测量：

- 姿态：`pitch`、左右腿 `alpha`、pitch gyro；
- 命令与里程计：`cmd.x`、`cmd.v`、`odom.x`、`odom.v`；
- 轮力矩：LQR 轮输出和轮电机命令力矩；
- 腿部几何：左右腿 `phi` 和腿长；
- 接触与竖直运动：左右法向力估计、总法向力、竖直加速度；
- 电机实际力矩：simulator 返回的关节力矩。

默认 1 kHz 控制频率下，可视化流在 controller 内部降采样到约 50 Hz。
可视化服务跟随同一个 `build/ctrl` 进程运行；重新编译后如果页面仍是旧行为，
请重启 controller。

## `tools/` Python 脚本

### `tools/fix_stl_for_mujoco.py`

修复 STL mesh，使其能被 MuJoCo 加载。功能包括：

- ASCII STL 转 binary STL；
- 将超过 MuJoCo 200,000 面限制的 mesh 降面；
- 将 0-face 空 marker mesh 替换为很小的四面体；
- 可选用 Python `mujoco` 包验证 mesh。

依赖：

```bash
python3 -m pip install numpy trimesh pymeshlab mujoco
```

用法：

```bash
# 只查看计划修改
python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes --dry-run

# 修复 mesh，并把备份放到 mjcf/meshes/stl_backup
python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes

# 指定备份目录
python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes --backup /tmp/wbr_stl_backup

# 不备份，直接覆盖
python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes --no-backup
```

### `tools/lqr/fit_lqr.py`

拟合轮腿平衡控制器的多项式 LQR 增益，可更新：

```text
controller/include/control/lqr_coeffs.hpp
```

用法：

```bash
# 只打印生成的 C++ 数组，不写文件
python3 tools/lqr/fit_lqr.py

# 重新拟合并写入 lqr_coeffs.hpp
python3 tools/lqr/fit_lqr.py --write

# 不重新拟合，只按比例缩放当前 header
python3 tools/lqr/fit_lqr.py --scale 0.62 --write

# 在指定腿长下输出诊断范数
python3 tools/lqr/fit_lqr.py --eval-len 0.18
```

依赖：

- `--scale`：Python 3 + NumPy
- 完整重新拟合：Python 3 + NumPy + SciPy

调参入口在 `tools/lqr/fit_lqr.py` 的 `LQR_WEIGHTS`，分别对应 `low`、`high`、
`spin` 三种模式。

### `tools/lqr/gen_matrices.py`

从 MATLAB 导出文件重新生成矩阵辅助代码：

```bash
python3 tools/lqr/gen_matrices.py
```

输入：

```text
tools/lqr/vendor/Amatrix.m
tools/lqr/vendor/Bmatrix.m
```

输出：

```text
tools/lqr/amatrix.py
tools/lqr/bmatrix.py
```

`fit_lqr.py` 使用的支持模块：

```text
tools/lqr/dynamics.py   根据符号函数构造 A/B 矩阵
tools/lqr/leg_data.py   腿长、质心、惯量表
tools/lqr/amatrix.py    生成出来的 A 矩阵辅助函数
tools/lqr/bmatrix.py    生成出来的 B 矩阵辅助函数
```

## 常见问题

- CMake 找不到 `mujoco_interface`：传
  `-DCMAKE_PREFIX_PATH=/path/to/mujoco_interface/install`。
- simulator 和 controller 没连上：检查两边的 `ipc_prefix` 是否匹配。
- mesh 加载失败：运行 `python3 tools/fix_stl_for_mujoco.py --dir mjcf/meshes`。
- 不需要 GUI 或没有显示环境：simulator 加 `--headless`。
