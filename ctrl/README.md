# ctrl

[中文说明](README.zh-CN.md)

External control process. Reads `LowState`, writes `LowCmd` over Shm.

```bash
./build/sim -c config/robots/wbr.yaml          # terminal 1
./build/ctrl --ipc-prefix wbr                    # terminal 2
```
