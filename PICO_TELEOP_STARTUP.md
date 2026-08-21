# Pico 遥操作启动

型号：`615` 或 `696`。**同一终端 stdin 交互**：先选硬件/仿真，再选 URDF，最后进入 500Hz 循环（`test_rt_teleop` 内 `std::cin`）。

## 一键启动（推荐）

```bash
bash scripts/start_pico_teleop.sh
```

流程：

1. **后台** Pico PC 服务（日志：`.cache/pico_teleop/pico_service.log`）
2. **后台** `pico_udp_publisher`（日志：`.cache/pico_teleop/pico_pub.log`）
3. **前台** `test_rt_teleop` — 在本终端依次输入：

```text
0=硬件  1=仿真: 1
选择 URDF 型号… 请选择 [0/1]: 0
0=退出  1=进入500Hz循环: 1
```

常用：

```bash
bash scripts/start_pico_teleop.sh --stop     # 仅停后台服务/发布（Ctrl+C 退出遥操作时也会自动停）
bash scripts/start_pico_teleop.sh --replay   # 发布节点 CSV 回放（无头显）
PICO_SERVICE=/path/to/runService.sh bash scripts/start_pico_teleop.sh
```

环境：[`scripts/pico_teleop_env.sh`](scripts/pico_teleop_env.sh)（含 Marvin SDK、`xrobotoolkit_sdk` 路径）

**首次使用**需编译 Pico Python 绑定（只需一次）：

```bash
sudo apt install -y cmake pybind11-dev   # 若未安装
bash scripts/install_xrobot_sdk.sh       # 默认 ~/repos/XRoboToolkit-PC-Service-Pybind
```

| URDF 选择 | 型号 |
|-----------|------|
| `0` | 696 Marvin M6-S-CCS |
| `1` | 615 Marvin M3-S-CCS |

## SCHED_FIFO（500Hz）

一键脚本会在缺少 `cap_sys_nice` 时调用 [`tj_test/grant_rt_caps.sh`](tj_test/grant_rt_caps.sh)（需 sudo）。进入循环后日志应为 `fifo=ok mlock=ok`，且 `cpu=` 为避开末核的隔离核（16 核机器通常为 **14**，可用 `TJ_RT_CPU` 覆盖）。

后台 Pico 服务/发布会 `taskset` 到**除控制核外**的所有 CPU，避免与 500Hz 环争用。

```bash
bash tj_test/grant_rt_caps.sh
getcap ./build/tj_test/test_rt_teleop
# 可选：指定控制核
TJ_RT_CPU=12 bash scripts/start_pico_teleop.sh
```

验收：看 **`[period] teleop PASS`**（双臂 Teleop 段，不含 GoHome/下使能）；`overrun_max≤200µs`，`in[1800,2200]≥99.5%`。

## 手动启动（3 终端）

```bash
source scripts/pico_teleop_env.sh

# 1. Pico PC 服务
~/repos/XRoboToolkit-PC-Service/RoboticsService/bin/runService.sh

# 2. Pico 位姿发布
python3 -m python.teleop.pico_udp_publisher

# 3. 遥操作（stdin 交互；也可 --sim / --hw 跳过第一问）
./build/tj_test/test_rt_teleop
```

无头显回放：第 2 步用 `--replay-csv … --trigger 1.0 --loop`。

可选可视化：`python python/vis/rt_teleop_vis.py`（非必需）。

## 遥操作无响应排查

退出后查看 `data/test_rt_teleop/teleop_diag.txt`：

| 字段 | 含义 |
|------|------|
| `valid_samples=0` | **未收到任何 Pico UDP**（发布节点未起或端口不对） |
| `fresh_samples=0` 但 valid>0 | 收到过包但已 stale（>200ms 无新包） |
| `trigger_ge_0.99_samples=0` | 有位姿但未按扳机（阈值 0.99） |
| `servo_pico api_enter=0` | 未进入 ServoPByPico（无 UDP 或未按扳机） |

常见原因：

1. **发布节点未启动** — 看 `.cache/pico_teleop/pico_pub.log`：
   - `python: 未找到命令` → PATH 问题（脚本已自动用 python3/miniconda）
   - `No module named 'xrobotoolkit_sdk'` → 执行 `bash scripts/install_xrobot_sdk.sh`
2. **Pico 头显未连 PC 服务** — 看 `pico_service.log`，确认头显与 PC 在同一网段且 XRoboToolkit 服务正常。
3. **未按扳机** — Teleop 阶段需扳机 ≥0.99 才 InitPlan/RePlan。

独立验证 UDP（另开终端）：

```bash
source scripts/pico_teleop_env.sh
python3 -m python.teleop.pico_udp_check --port 30101 --count 10
```

遥操作循环内调试：`./build/tj_test/test_rt_teleop --pico-print`（50Hz 打印订阅到的位姿/扳机）。

**卡顿/延时归因**（退出后或另开终端）：

```bash
# 离线：分析 pico_teleop.csv（seq 缺口 = UDP/接收；timestamp 尖峰 = SDK/头显）
python3 -m python.teleop.pico_latency_diag --csv data/test_rt_teleop/pico_teleop.csv

# 实时：仅测 Pico SDK（PC Service → xrobotoolkit_sdk）
python3 -m python.teleop.pico_latency_diag --sdk --seconds 15

# 实时：仅测 UDP 到达间隔（需 pico_udp_publisher 已运行）
python3 -m python.teleop.pico_latency_diag --udp --seconds 15
```

`teleop_diag.txt` 新增字段：`udp_seq_gap`、`pico_ts_spike_gt50ms`、`frozen_during_spike`。

## 数据落盘

默认 `data/test_rt_teleop/`；控制 500Hz，Servo/记录 50Hz。详见 [`mv_control/RT_TELEOP_DESIGN.md`](mv_control/RT_TELEOP_DESIGN.md)。
