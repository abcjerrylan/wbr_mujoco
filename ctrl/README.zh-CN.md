# ctrl

[English](README.md)

外部控制进程，通过 Shm 读写 `LowState` / `LowCmd`。

```bash
./build/sim -c config/robots/wbr.yaml          # 终端 1
./build/ctrl --ipc-prefix wbr                    # 终端 2
```
