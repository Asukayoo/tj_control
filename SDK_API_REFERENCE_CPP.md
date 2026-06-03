# controlSDK / kinSDK 接口参考（C++）

> 头文件：`contrlSDK/MarvinSDK.h`、`kinematicsSDK/FxRobot.h`  
> 库：`libMarvinSDK.so/.dll`、`libKine.so/.dll`  
> 官方详述：[c++_doc_contrl.md](TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/c++_doc_contrl.md)、[c++_doc_kine.md](TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/c++_doc_kine.md)

相关文档：[Python 版](SDK_API_REFERENCE_PYTHON.md) | [总览](SDK_API_REFERENCE.md)

---

## 编译与链接

```bash
# Linux 控制库（contrlSDK 目录）
g++ *.cpp -Wall -O2 -fPIC -shared -o libMarvinSDK.so -lpthread -lrt -DCMPL_LIN

# Linux 运动学库（kinematicsSDK 目录）
g++ *.cpp -Wall -O2 -fPIC -shared -o libKine.so -lpthread -lrt

# 应用链接示例（workspace/main.cpp）
g++ -Wall main.cpp -I../contrlSDK -L../contrlSDK -lMarvinSDK -lpthread -lrt -DCMPL_LIN
```

Windows 见 `marvinSDK_windows.bat` 或 README 中 MinGW 命令。

---

## 架构与联用

```mermaid
flowchart LR
    APP[C++ 应用] --> CTRL[libMarvinSDK]
    APP --> KIN[libKine]
    CTRL <-->|UDP 1kHz| ROBOT[控制器]
    KIN -.->|FX_Robot_PLN_MOVLA_C → pset| CTRL
```

典型流程：`LOADMvCfg` / `FX_Robot_Init_*` → `FX_Robot_PLN_MOVLA_C` → `RunPlnCart("A", pset)`。

---

## 通用约定

| 项 | 说明 |
|----|------|
| 臂字符 | `FX_CHAR arm`：`'A'`、`'B'`；急停 `"AB"`（`EStop`） |
| `RobotSerial` | 运动学实例序号，左 0 / 右 1（与配置双臂对应） |
| 角度 / 长度 | 关节 **度**；笛卡尔 **mm** |
| 经典控制 | `OnClearSet()` → 设置 → `OnSetSend()`；间隔 ≥ 1 ms |
| 简明控制 | `Connect`、`SetJointPostionCmd` 等封装清错与发送 |

---

## 安全注意

- UDP 无加密、无鉴权：隔离网络；`OnRelease()` 释放连接。
- `*.MvKDCfg` 机型错误会导致运动学静默错误；TYPE 1007（SRS）/ 1017（CCS）。

---

# 一、controlSDK（`MarvinSDK.h`）

`extern "C"`，导出宏 `FX_DLL_EXPORT`。

## 1.1 连接 / 系统 / 订阅

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `OnLinkTo(ip1..ip4)` | 4×`FX_UCHAR` | `bool` | UDP 连接 |
| `OnRelease()` | — | `bool` | **必须**释放 |
| `OnGetSDKVersion()` | — | `long` | SDK 版本 |
| `OnGetBuf(DCSS *ret)` | 结构体指针 | `bool` | 1 kHz 订阅 |
| `OnUpdateSystem(local_path)` | 升级包路径 | `bool` | 升级控制器 |
| `OnDownloadLog(local_path)` | 本机路径 | `bool` | 下载日志 |
| `OnSendFile` / `OnRecvFile` | 本地/远程绝对路径 | `bool` | 文件传输 |
| `OnSetIntPara` / `OnSetFloatPara` | `char[30]` 名 + 值 | `long` | 写参数 |
| `OnGetIntPara` / `OnGetFloatPara` | 名 + 出参 | `long` | 读参数 |
| `OnSavePara()` | — | `long` | 持久化 |
| `OnLogOn` / `OnLogOff` | — | `void` | 全局日志 |
| `OnLocalLogOn` / `OnLocalLogOff` | — | `void` | 本地日志 |

## 1.2 急停 / 伺服错误

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `OnEMG_A` / `OnEMG_B` / `OnEMG_AB` | — | `void` | 软急停 |
| `OnServoReset_A/B(int axis)` | 0~6 | `void` | 单轴复位 |
| `OnGetServoErr_A/B(long ErrCode[7])` | 数组 | `void` | 错误码 |
| `OnClearErr_A` / `OnClearErr_B` | — | `void` | 清臂错误 |

## 1.3 批指令（`OnClearSet` ~ `OnSetSend`）

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `OnClearSet()` | — | `bool` | 清待发 |
| `OnSetSend()` | — | `bool` | 下发 |
| `OnSetSendWaitResponse(time_out)` | ms | `long` | 带超时 |
| `OnSetTargetState_A/B(state)` | 0~4 | `bool` | 控制模式 |
| `OnSetImpType_A/B(type)` | 1/2/3 | `bool` | 阻抗类型（扭矩下） |
| `OnSetJointLmt_A/B(vel, acc)` | 0~100 | `bool` | 速度加速度比 |
| `OnSetJointKD_A/B(K[7], D[7])` | 关节阻抗 | `bool` | — |
| `OnSetCartKD_A/B(K, D, type)` | 笛卡尔阻抗 | `bool` | — |
| `OnSetEefRot_A/B(fcType, CartCtrlPara[7])` | 1 自定义 / 2 自动 | `bool` | 末端旋转 |
| `OnSetForceCtrPara_A/B(...)` | 力控参数 | `bool` | 扭矩 + 力控类型 3 |
| `OnSetForceCmd_A/B(force)` | N | `bool` | 力目标 |
| `OnSetTool_A/B(kine[6], dyn[10])` | 工具参数 | `bool` | TCP / 动力学 |
| `OnSetJointCmdPos_A/B(joint[7])` | 度 | `bool` | 关节指令 |
| `OnSetDragSpace_A/B(dgType)` | 0~5 | `bool` | 拖动 |
| `OnSetPVT_A/B(id)` | 0~99 | `bool` | 运行 PVT |
| `OnSendPVT_A/B(file, serial)` | 路径 + ID | `bool` | 上传轨迹 |
| `OnStartGather` / `OnStopGather` | 采集配置 | `bool` | 数据采集 |
| `OnSaveGatherData` / `OnSaveGatherDataCSV` | 路径 | `bool` | 保存；后 sleep ≥1s |

**`state`（`OnSetTargetState`）**：0 下使能；1 位置；2 PVT；3 扭矩；4 协作释放。

## 1.4 规划（控制侧执行）

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `OnInitPlnLmt(char *path)` | `*.MvKDCfg` | `bool` | 初始化一次 |
| `OnSetPlnJoint_A/B(start[7], stop[7], vel, acc)` | 关节；速度比 | `bool` | 关节规划 |
| `OnSetPlnCart_A/B(void *pset)` | kinSDK 点集 | `bool` | 笛卡尔规划 |
| `OnStopPlnJoint_A/B` | — | `bool` | 停止 |
| `FX_CPointSet_Create/Destroy` | — | `void*` | 点集句柄 |

## 1.5 末端通道

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `OnClearChDataA/B` | — | `bool` | 清缓存 |
| `OnGetChDataA/B(data[256], *ret_ch)` | 缓冲 | `long` 长度 | ch：1 CAN 2 COM1 3 COM2 |
| `OnSetChDataA/B(data, size, set_ch)` | ≤256 字节 | `bool` | 发送 |

## 1.6 简明 API（`Connect` 系列，推荐）

| API | 输入 | 输出 | 说明 |
|-----|------|------|------|
| `Connect(ip1..ip4, log_switch=0)` | IP；日志 | `bool` | 连接 + 可选日志 |
| `LogSwitch(signal)` | 0/1 | `void` | 日志开关 |
| `EStop(const FX_CHAR *arm)` | `"A"`/`"B"`/`"AB"` | `void` | 急停 |
| `ServoReset(arm, axis)` | 臂 + 轴 | `void` | 伺服复位 |
| `CheckArmError` / `CheckServoError` | — | `bool` | 检查并清错 |
| `ClearErr()` | — | `void` | 清双臂错误 |
| `SetTool(arm, kine[6], dyn[10])` | 工具 | `bool` | — |
| `SetJointMode(arm, vel, acc)` | 0~100 | `bool` | 位置模式 |
| `SetImpJointMode(arm, vel, acc, K, D)` | 关节阻抗 | `bool` | 扭矩 |
| `SetImpCartMode(arm, vel, acc, K, D, RotType, CartCtrlPara)` | 笛卡尔阻抗 | `bool` | RotType 0/1/2 |
| `SetImpForceMode(arm, fxDir[6], fcAdjLmt)` | 力控 | `bool` | — |
| `SetForceCmd(arm, force)` | N | `bool` | — |
| `SetJointPostionCmd(arm, joint[7])` | 度 | `bool` | 关节指令 |
| `PlnInit(path)` | 配置 | `bool` | 规划初始化 |
| `RunPlnJoint(arm, start, stop, vel, acc)` | 7+7 | `bool` | 关节规划运行 |
| `RunPlnCart(arm, pset)` | 点集 | `bool` | 笛卡尔规划运行 |
| `StopPln(arm)` | — | `bool` | 停规划 |
| `SendPVT(arm, file, serial)` | 0~99 | `bool` | 上传 PVT |
| `RunPVT(arm, id)` | id | `bool` | 运行 PVT |
| `SetJointDrag` / `SetCartDrag(arm, type)` / `ExitDrag` | `"X"`~`"R"` | `bool` | 拖动 |
| `Disable(arm)` | — | `bool` | 下使能 |
| `ClearChData` / `GetChData` / `SetChData` | 同 A/B 通道 | — | 末端通讯 |
| `StartCollectData` / `StopCollectData` | 同 `OnStartGather` | `bool` | 采集 |

参数读写仍用 `OnSetIntPara` / `OnGetFloatPara` 等；订阅仍用 `OnGetBuf`。

---

## 1.7 控制 SDK 示例

**简明接口**

```cpp
#include "MarvinSDK.h"

int main() {
  Connect(192, 168, 1, 190, 0);
  CheckArmError();
  double kine[6] = {0}, dyn[10] = {0};
  SetTool('A', kine, dyn);
  SetJointMode('A', 10, 10);
  double joint[7] = {0};
  SetJointPostionCmd('A', joint);
  OnRelease();
  return 0;
}
```

**经典批指令**

```cpp
OnLinkTo(192, 168, 1, 190);
OnClearSet();
OnSetTargetState_A(1);
OnSetJointLmt_A(10, 10);
double j[7] = {10, 20, 30, 40, 50, 60, 70};
OnSetJointCmdPos_A(j);
OnSetSend();
OnRelease();
```

---

# 二、kinSDK（`FxRobot.h`）

`extern "C"`；机型枚举：`FX_ROBOT_TYPE_PILOT_SRS` (1007)、`FX_ROBOT_TYPE_PILOT_CCS` (1017)。

## 2.1 配置与初始化

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `LOADMvCfg(path, TYPE, GRV, DH, PNVA, BD, Mass, MCP, I)` | `*.MvKDCfg` 及出参数组 | `FX_BOOL` | 一次读配置 |
| `FX_LOG_SWITCH(log_tag)` | 0/1 | `void` | 日志 |
| `FX_Robot_Init_Type(serial, RobotType)` | 序号 + 机型 | `FX_BOOL` | — |
| `FX_Robot_Init_Kine(serial, DH[8][4])` | MDH | `FX_BOOL` | 度 + mm |
| `FX_Robot_Init_Lmt(serial, PNVA[7][4], J67[4][3])` | 限位；CCS 干涉 | `FX_BOOL` | — |
| `FX_Robot_Tool_Set(serial, Matrix4 tool)` | 4×4 相对法兰 | `FX_BOOL` | TCP |
| `FX_Robot_Tool_Rmv(serial)` | — | `FX_BOOL` | 移除工具 |

## 2.2 运动学

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `FX_Robot_Kine_FK(serial, joints[7], Matrix4 pgos)` | 关节角 | 4×4 位姿 | 正解 |
| `FX_Robot_Kine_FK_NSP(..., Matrix3 nspg)` | 关节角 | 位姿 + nsp | 零空间矩阵 |
| `FX_Robot_Kine_IK(serial, FX_InvKineSolvePara *)` | 逆解结构体 | `FX_BOOL` | 参考角 J4≠0 |
| `FX_Robot_Kine_IK_NSP(serial, solve_para)` | + 臂角参数 | `FX_BOOL` | 零空间 IK |
| `FX_Robot_Kine_Jacb(serial, joints, FX_Jacobi *)` | — | 6×7 雅可比 | — |

**`FX_InvKineSolvePara` 主要成员**（见 `FxRobot.h`）

| 成员 | 入/出 | 含义 |
|------|-------|------|
| `m_Input_IK_TargetTCP` | 入 | `Matrix4` 目标 |
| `m_Input_IK_RefJoint` | 入 | `Vect7` 参考关节 |
| `m_Input_IK_ZSPType` | 入 | 0 / 1 |
| `m_Input_IK_ZSPPara[6]` | 入 | 零空间平面 |
| `m_Input_ZSP_Angle` | 入 | 臂角（度） |
| `m_DGR1` / `m_DGR2` | 入 | 奇异判定（仅 NSP） |
| `m_Output_RetJoint` | 出 | `Vect7` |
| `m_OutPut_AllJoint` | 出 | `Matrix8` 多解 |
| `m_OutPut_Result_Num` | 出 | 解组数 |
| `m_Output_IsOutRange` 等 | 出 | 标志位 |

## 2.3 规划

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `FX_Robot_CalEndXYZABC(Start, offset, RotType, Angle, End)` | `Vect6` XYZABC | `End` | 位姿叠加 |
| `FX_Robot_PLN_MOVL(..., OutPutPath)` | 起终点；参考关节；vel/acc；Freq | `FX_BOOL` | 离线文件 |
| `FX_Robot_PLN_MOVLA_C(..., void *ret_pset)` | 同上 | `FX_BOOL` | 在线点集 → `RunPlnCart` |
| `FX_Robot_PLN_MOVL_KeepJ` / `KeepJA_C` | 关节起终点 | `FX_BOOL` | 保持构型 |
| `FX_CPointSet_Create/Destroy/OnInit/OnGetPoint/OnSetPoint` | 点集 C 包装 | — | 点集操作 |

C++ 专用（非 C）：`FX_Robot_PLN_MOVLA(..., CPointSet *ret_pset)`。

## 2.4 动力学与其它

| API | 输入 | 输出 | 用法 |
|-----|------|------|------|
| `FX_Robot_JntTau2EETau(serial, q, Joint_Torque, EE_Torque)` | 7 关节力矩 | 6 维末端力 | 映射 |
| `FX_Robot_Iden_LoadDyn(Type, path, mass, mr, I)` | 1 CCS / 2 SRS | `FX_INT32` | 负载辨识 |
| `FX_XYZABC2Matrix4DEG` / `FX_Matrix42XYZABCDEG` | 6 ↔ 4×4 | — | 位姿转换 |

---

## 2.5 运动学 + 控制联用示例

```cpp
#include "FxRobot.h"
#include "MarvinSDK.h"

void demo() {
  FX_DOUBLE dh[8][4], pnva[7][4], j67[4][3];
  FX_INT32L type[2];
  LOADMvCfg("config/srs.MvKDCfg", type, /* GRV, DH, ... */);

  FX_INT32L serial = 0;
  FX_Robot_Init_Type(serial, FX_ROBOT_TYPE_PILOT_SRS);
  FX_Robot_Init_Kine(serial, dh);
  FX_Robot_Init_Lmt(serial, pnva, j67);

  FX_DOUBLE joints[7] = {0, 30, 60, 90, 0, 0, 0};
  void *pset = FX_CPointSet_Create();
  Vect6 start = {300, 0, 400, 0, 0, 0};
  Vect6 end   = {400, 0, 400, 0, 0, 0};
  Vect7 ref;
  memcpy(ref, joints, sizeof(ref));
  FX_Robot_PLN_MOVLA_C(serial, start, end, ref, 50, 100, 500, pset);

  Connect(192, 168, 1, 190);
  PlnInit("config/srs.MvKDCfg");
  SetJointMode('A', 10, 10);
  RunPlnCart('A', pset);
  FX_CPointSet_Destroy(pset);
  OnRelease();
}
```

---

## 延伸阅读

| 文档 | 路径 |
|------|------|
| 控制详述 | `TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/c++_doc_contrl.md` |
| 运动学详述 | `TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/c++_doc_kine.md` |
| DEMO | `TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++/` |
