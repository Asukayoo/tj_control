# controlSDK / kinSDK 接口参考（Python）

> 封装：`TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/SDK_PYTHON/fx_robot.py`、`fx_kine.py`  
> 动态库：`libMarvinSDK.so/.dll`、`libKine.so/.dll`  
> 官方详述：[python_doc_contrl.md](TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/python_doc_contrl.md)、[python_doc_kine.md](TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/python_doc_kine.md)

相关文档：[C++ 版](SDK_API_REFERENCE_CPP.md) | [总览](SDK_API_REFERENCE.md)

---

## 架构与联用

```mermaid
flowchart LR
    PY[fx_robot / fx_kine] --> CTRL[libMarvinSDK]
    PY --> KIN[libKine]
    CTRL <-->|UDP 1kHz| ROBOT[控制器]
    KIN -.->|pset| CTRL
```

典型流程：`load_config` → `initial_kine` → `movLA` 得 `pset` → `run_pln_cart("A", pset)`。

---

## 通用约定

| 项 | 说明 |
|----|------|
| 臂 ID 字符串 | `A` 左臂，`B` 右臂，`AB` 双臂急停 |
| 订阅索引 | `outputs[0]` / `inputs[0]` 为 A，`[1]` 为 B |
| 运动学臂序号 | `load_config(arm_type)`：`0` 左，`1` 右；双臂需两个 `Marvin_Kine` 实例 |
| 角度 / 长度 | 关节 **度**；笛卡尔位置 **mm** |
| 经典控制 | `clear_set()` → 设置指令 → `send_cmd()`；API 间 `sleep ≥ 1 ms` |
| 简明控制 | `Concise_Marvin_Robot` 无需 `clear_set`/`send_cmd` |

---

## 安全注意

- UDP 无加密、无鉴权：隔离网络，用毕 `release_robot()`。
- 调试速度从 10% 起；`*.MvKDCfg` 机型（1007 SRS / 1017 CCS）必须与实物一致。

---

# 一、controlSDK（`fx_robot.py`）

## 1.1 `Marvin_Robot`（经典接口）

```python
from fx_robot import Marvin_Robot
robot = Marvin_Robot()
```

### 连接与系统

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `connect(robot_ip)` | IP 字符串 | `int` 1/0 | 先 `ping`；连接后建议清错 |
| `release_robot()` | — | `int` 1/0 | **必须**释放 |
| `SDK_version()` | — | `long` | — |
| `update_SDK(sdk_path)` | 升级包绝对路径 | — | — |
| `download_sdk_log(log_path)` | 本机目录绝对路径 | — | — |
| `log_switch(flag)` | `"0"` / `"1"` | — | 全局 1 kHz 日志 |
| `local_log_switch(flag)` | `"0"` / `"1"` | — | 主要指令日志 |
| `receive_file(local, remote)` | 绝对路径 | `bool` | 控制器 → 本机 |
| `send_file(local, remote)` | 绝对路径 | `bool` | 本机 → 控制器 |

### 订阅与参数

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `subscribe(dcss)` | `DCSS` 实例 | `dict` | 1 kHz；`states`/`outputs`/`inputs` |
| `get_param(type, paraName)` | `"int"`/`"float"`；见 `robot.ini` | 参数值 | 如 `R.A0.BASIC.Type` |
| `set_param(type, paraName, value)` | 同上 + 值 | — | 后接 `save_para_file` |
| `save_para_file()` | — | — | 写入控制器 |

**`subscribe` 常用字段**（`outputs[i]`）：`fb_joint_pos`、`fb_joint_vel`、`low_speed_flag`、`est_joint_force`、`est_cart_fn`。

### 急停与错误

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `soft_stop(arm)` | `A` / `B` / `AB` | — | 软急停 |
| `get_servo_error_code(arm, lang)` | `CN` / `EN` | 7 元列表（十六进制） | — |
| `clear_error(arm)` | `A` / `B` | — | 在 `clear_set`~`send_cmd` 间 |
| `servo_reset(arm, axis)` | `axis` 0~6 | — | 单轴软复位 |

### 数据采集

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `collect_data(targetNum, targetID, recordNum)` | 列 ≤35；ID 列表；行 1e3~1e6 | — | 右臂 ID +100 |
| `stop_collect_data()` | — | `int` | 中途停等 1 ms |
| `save_collected_data_to_path(path)` | 绝对路径 | — | 原始格式 |
| `save_collected_data_as_csv_to_path(path)` | 绝对路径 | — | CSV |

### 1 kHz 批指令（`clear_set` → … → `send_cmd`）

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `clear_set()` | — | `int` | 清缓冲 |
| `send_cmd()` | — | `int` | 下发 |
| `send_cmd_wait_response(timeout)` | ms | `long` | 带应答 |
| `set_state(arm, state)` | 0 下使能 / 1 位置 / 2 PVT / 3 扭矩 / 4 协作释放 | `int` | 先参数后状态 |
| `set_impedance_type(arm, type)` | 1 关节 / 2 笛卡尔 / 3 力控 | `int` | 需 `state=3` |
| `set_vel_acc(arm, velRatio, AccRatio)` | 0~100 | `int` | PVT、拖动不限速 |
| `set_tool(arm, kineParams, dynamicParams)` | 6 + 10 | `int` | TCP / 扭矩必需 |
| `set_joint_kd_params(arm, K, D)` | 各 7 | `int` | 关节阻抗 |
| `set_cart_kd_params(arm, K, D, type)` | 各 7 + 类型 | `int` | 笛卡尔阻抗 |
| `set_force_control_params(...)` | fcType、方向 6 维等 | `int` | 力控参数 |
| `set_force_cmd(arm, f)` | N | `int` | 力目标 |
| `set_EefCart_control_params(arm, fcType, CartCtrlPara)` | fcType 1/2 | `int` | 末端旋转 |
| `set_joint_cmd_pose(arm, joints)` | 7 角（度） | `int` | 位置/扭矩跟踪 |
| `set_drag_space(arm, dgType)` | 0 退出 / 1 关节 / 2~5 笛卡尔 | — | 切换前先退出 |
| `send_pvt_file(arm, pvt_path, id)` | id 1~99 | — | 需 `state=2` |
| `set_pvt_id(arm, id)` | 轨迹 ID | `int` | 起点对齐轨迹 |

### 规划与末端通信

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `pln_init(config_path)` | `*.MvKDCfg` | `bool` | 一次 |
| `setPln_joint(arm, start, target, vel, acc)` | 7+7 关节；比例 0~1 | `bool` | 关节规划 |
| `setPln_Cart(arm, pset)` | `movLA` 的 `void*` | `bool` | 笛卡尔规划 |
| `stopRunPln_joint(arm)` | `A`/`B` | — | 停规划 |
| `clear_485_cache` / `set_485_data` / `get_485_data` | `com` 1 CAN / 2 COM1 / 3 COM2 | `bool` / tuple | ≤256 字节/包 |

---

## 1.2 `Concise_Marvin_Robot`（简明接口）

```python
from fx_robot import Concise_Marvin_Robot
robot = Concise_Marvin_Robot()
```

无需 `clear_set`/`send_cmd`；内部自动清错发送。

| 接口 | 输入 | 输出 | 说明 |
|------|------|------|------|
| `connect(robot_ip, log_switch=0)` | IP；0/1 | `bool` | — |
| `check_arms_error()` / `check_servo_error()` | — | `bool` | 检查并清错 |
| `set_position_state(arm, vel, acc)` | 0~100 | `bool` | 位置模式 |
| `set_imp_joint_state(arm, vel, acc, K, D)` | D∈[0,1] | `bool` | 关节阻抗 |
| `set_imp_cart_state(arm, vel, acc, K, D, rot_type, cart_ctrl_para)` | rot 0/1/2 | `bool` | 笛卡尔阻抗 |
| `set_imp_force_state(arm, fx_dir, fc_adj_lmt)` | 方向 6 维 | `bool` | 力控模式 |
| `set_joint_position_cmd(arm, joint)` | 7 角 | `bool` | 关节指令 |
| `set_force_cmd(arm, force)` | N | `bool` | 力指令 |
| `pln_init` / `run_pln_joint` / `run_pln_cart` / `stop_pln` | 同经典规划 | `bool` | — |
| `send_pvt` / `run_pvt` | 路径；id 0~99 | `bool` | PVT |
| `set_joint_drag` / `set_cart_drag(arm, type_)` / `exit_drag` | `X`/`Y`/`Z`/`R` | `bool` | 拖动 |
| `disable(arm)` | — | `bool` | 下使能 |
| `clear_ch_data` / `get_ch_data` / `set_ch_data` | 同 485 | — | 末端透传 |
| `start_collect_data` / `stop_collect_data` / `save_gather_data*` | 同经典采集 | `bool` | — |

其余：`release_robot`、`subscribe`、`get_param`、`set_param`、`soft_stop`、`servo_reset`、`set_tool` 等同 `Marvin_Robot`。

---

## 1.3 控制 SDK 示例

**简明接口**

```python
from fx_robot import Concise_Marvin_Robot
import time

robot = Concise_Marvin_Robot()
assert robot.connect("192.168.1.190")
robot.check_arms_error()
robot.set_tool("A", [0] * 6, [0] * 10)
robot.set_position_state("A", velRatio=10, AccRatio=10)
robot.set_joint_position_cmd("A", [0] * 7)
time.sleep(2)
robot.release_robot()
```

**经典 1 kHz 批**

```python
from fx_robot import Marvin_Robot

robot = Marvin_Robot()
robot.connect("192.168.1.190")
robot.clear_set()
robot.set_state("A", 1)
robot.set_vel_acc("A", 10, 10)
robot.set_joint_cmd_pose("A", [10, 20, 30, 40, 50, 60, 70])
robot.send_cmd()
```

---

# 二、kinSDK（`fx_kine.py`）

## 2.1 `Marvin_Kine`

```python
from fx_kine import Marvin_Kine, FX_InvKineSolvePara
kk = Marvin_Kine()
```

### 初始化

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `log_switch(switch)` | 0 / 1 | — | 库日志 |
| `load_config(arm_type, config_path)` | 0 左 / 1 右；`*.MvKDCfg` | `dict` | TYPE/DH/PNVA/BD… |
| `initial_kine(robot_type, dh, pnva, j67)` | 8×4 DH；7×4 限位；4×3 BD | `bool` | 在 `load_config` 后 |
| `set_tool_kine(tool_mat)` | 4×4（相对法兰） | `bool` | 解算到 TCP |
| `remove_tool_kine()` | — | `bool` | 回法兰 |

### 运动学

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `fk(joints)` | 7 角（度） | `list[4][4]` | mm + 度 |
| `fk_nsp(joints)` | 7 角 | 位姿 + 3×3 nsp | nsp 列可作 IK 参数 |
| `ik(structure_data)` | `FX_InvKineSolvePara` | `bool` | J4≠0 |
| `ik_nsp(structure_data)` | 同上 | `bool` | 臂角/零空间 |
| `joints2JacobMatrix(joints)` | 7 角 | 6×7 | 雅可比 |

**`FX_InvKineSolvePara` 主要字段**

| 字段 | 入/出 | 含义 |
|------|-------|------|
| `m_Input_IK_TargetTCP` | 入 | 4×4 目标 TCP |
| `m_Input_IK_RefJoint` | 入 | 7 参考关节 |
| `m_Input_IK_ZSPType` | 入 | 0 欧式最近；1 臂角平面 |
| `m_Input_IK_ZSPPara` | 入 | 6 维（type=1） |
| `m_Input_ZSP_Angle` | 入 | 臂角（度） |
| `m_Output_RetJoint` | 出 | 解算关节 |
| `m_OutPut_AllJoint` | 出 | 全部解（≤8 组） |
| `m_OutPut_Result_Num` | 出 | 解的组数 |
| `m_Output_IsOutRange` / `IsDeg` / `IsJntExd` | 出 | 可达/奇异/超限 |

辅助方法：`set_input_ik_*`、`get_output_*`（见类定义）。

### 规划与变换

| 接口 | 输入 | 输出 | 用法要点 |
|------|------|------|----------|
| `movL(..., save_path)` | XYZABC 起终点；参考关节；vel/acc；freq | `bool` | 离线 ~500Hz |
| `movLA(...)` | 同上 | `(点列表, pset)` | 供 `run_pln_cart` |
| `movL_KeepJ` / `movL_KeepJA` | 关节起终点 | 文件 / 点集 | 保持构型 |
| `create_point_set` / `destroy_point_set` | 点类型 | `void*` | 生命周期 |
| `get_point_set_data(pset, dim)` | 6 或 7 维 | `List[List[float]]` | — |
| `calculate_end_xyzabc(...)` | 位姿 + 偏移 + 旋转类型 | 6 维 XYZABC | — |
| `xyzabc_to_mat4x4` / `mat4x4_to_xyzabc` | 6 ↔ 4×4 | 矩阵 / XYZABC | — |
| `mat4x4_to_mat1x16` | 4×4 | 16 元列表 | — |
| `identify_tool_dyn(robot_type, ipath)` | 1 CCS / 2 SRS | 10 元动力学或错误码 1~4 | — |

---

## 2.2 运动学 + 控制联用示例

```python
import os
from fx_kine import Marvin_Kine, FX_InvKineSolvePara
from fx_robot import Concise_Marvin_Robot

kk = Marvin_Kine()
ini = kk.load_config(0, "config/srs.MvKDCfg")
kk.initial_kine(ini["TYPE"][0], ini["DH"][0], ini["PNVA"][0], ini["BD"][0])

joints = [0, 30, 60, 90, 0, 0, 0]
_, pset = kk.movLA(
    [300, 0, 400, 0, 0, 0], [400, 0, 400, 0, 0, 0],
    joints, vel=50, acc=100, freq_hz=500,
)

robot = Concise_Marvin_Robot()
robot.connect("192.168.1.190")
robot.pln_init("config/srs.MvKDCfg")
robot.set_position_state("A", 10, 10)
robot.run_pln_cart("A", pset)
kk.destroy_point_set(pset)
robot.release_robot()
```

---

## 延伸阅读

| 文档 | 路径 |
|------|------|
| 控制详述 | `TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/python_doc_contrl.md` |
| 运动学详述 | `TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/python_doc_kine.md` |
| DEMO | `TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/DEMO_PYTHON/` |
