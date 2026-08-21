#pragma once

#include <array>

#include <Eigen/Dense>
#include <Eigen/Geometry>

constexpr int DOF = 7;
constexpr double PI = 3.14159265358979323846;
constexpr double D2R = PI / 180.0;
constexpr double R2D = 180.0 / PI;
constexpr double kControlDt = 0.002;
constexpr int kControlHz = static_cast<int>(1.0 / kControlDt + 0.5);
constexpr int kControlPeriodUs = static_cast<int>(kControlDt * 1e6 + 0.5);
constexpr int64_t kControlPeriodUsLo = (kControlPeriodUs * 9) / 10;
constexpr int64_t kControlPeriodUsHi = (kControlPeriodUs * 11) / 10;
constexpr int kControlCatchUpPeriodUs = kControlPeriodUs / 2;
constexpr int kControlWorkStallUs =
    static_cast<int>(static_cast<double>(kControlPeriodUs) * 1.5 + 0.5);
constexpr int kControlCyclesPerSecond = kControlHz;
constexpr int kControlMaxCycles5Min = kControlHz * 300;

// 墙钟时长 [s] → 控制周期数（≥1）
constexpr int ControlCyclesFromSeconds(double sec) {
    const int n = static_cast<int>(sec / kControlDt + 0.5);
    return n < 1 ? 1 : n;
}

constexpr int ControlMsFromSeconds(double sec) {
    const int ms = static_cast<int>(sec * 1000.0 + 0.5);
    return ms < 1 ? 1 : ms;
}

// 墙钟语义（秒）→ 周期/ms（随 Ts 缩放；不含 Servo 样条插值窗口）
constexpr double kWallTransitionTimeoutS = 1.0;
constexpr double kWallSdkFrameStaleS = 0.020;
constexpr double kWallSdkInitPollWindowS = 0.005;
constexpr double kWallOpenClearRetryS = 0.010;
constexpr double kWallSdkSendSettleS = 0.010;
constexpr double kWallVelEstSendWaitS = 0.050;
constexpr double kWallVelEstRetryBudgetS = 0.100;
constexpr double kWallMotionStopS = 0.200;
constexpr double kWallServoErrPollS = 1.0;

constexpr int kTransitionTimeoutCycles =
    ControlCyclesFromSeconds(kWallTransitionTimeoutS);
constexpr int kSdkFrameStaleRunCycles =
    ControlCyclesFromSeconds(kWallSdkFrameStaleS);
constexpr int kSdkFramePollTries =
    ControlCyclesFromSeconds(kWallSdkInitPollWindowS);
constexpr int kOpenClearRetries = ControlCyclesFromSeconds(kWallOpenClearRetryS);
constexpr int kVelEstSlotRetries =
    ControlCyclesFromSeconds(kWallVelEstRetryBudgetS);
constexpr int kMotionStopCycles = ControlCyclesFromSeconds(kWallMotionStopS);
constexpr double kMotionStopDurationS = kMotionStopCycles * kControlDt;
constexpr int kServoErrPollCyclesDefault =
    ControlCyclesFromSeconds(kWallServoErrPollS);

// SDK FX_OnSetVelEstStep：轨迹/指令发送周期(ms)；false=Init 不发（屏蔽 1 周期速度前瞻）
constexpr bool kEnableVelEstStep = true;
constexpr int kVelEstStepMs = static_cast<int>(kControlDt * 1000.0 + 0.5);

constexpr int kControlPeriodMs = kVelEstStepMs;
constexpr int kInitPollSleepMs = kControlPeriodMs;
constexpr int kSdkSendSleepMs = ControlMsFromSeconds(kWallSdkSendSettleS);
constexpr int kVelEstSlotSleepMs = kControlPeriodMs;
constexpr long kVelEstSendWaitMs = ControlMsFromSeconds(kWallVelEstSendWaitS);

// Servo 内部三次样条窗口（固定 40ms）；与外部指令频率无关
constexpr double kStreamServoWindowS = 0.040;
constexpr int kStreamServoCycles =
    static_cast<int>(kStreamServoWindowS / kControlDt + 0.5);
constexpr double kStreamServoPeriod = kStreamServoCycles * kControlDt;

using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;
using V6d = Eigen::Matrix<double, 6, 1>;
using V7d = Eigen::Matrix<double, 7, 1>;

using M3d = Eigen::Matrix<double, 3, 3>;
using M4d = Eigen::Matrix<double, 4, 4>;
using M6d = Eigen::Matrix<double, 6, 6>;
using M7d = Eigen::Matrix<double, 7, 7>;
using Jacob = Eigen::Matrix<double, 6, 7>;

using Quat = Eigen::Quaternion<double, Eigen::DontAlign>;

struct JointState {
    V7d q;
    V7d v;
    V7d a;
    V7d j;
    V7d tau;
};

struct alignas(16) Pose {
    V3d pos = V3d::Zero();
    Quat quat = Quat::Identity();
};

struct alignas(16) CartState {
    Pose pose;
    V6d vel;
    Jacob jacob;
};

struct alignas(16) RobotState {
    JointState joint_state;
    CartState cart_state;
};

enum class ControlMode {
    Position = 0,
    JointImp = 1,
    CartImp = 2,
    Force = 3
};

enum class ControlModeStatus {
    Position = 0,
    JointImp = 1,
    CartImp = 2,
    Force = 3,
    Translating = 4,
};

// SetEnable 唯一入参
enum class EnableMode {
    Disable = 0,
    Enable = 1
};

enum class StatusCode {
    Disabled = 0,
    Ready = 1,
    Running = 2,
    Stopping = 3,
    Fault = 4,
};

enum class EnableState {
    Disabled = 0,
    Enabling = 1,
    Enabled = 2,
    Disabling = 3,
};

enum class ErrorCode {
    Normal = 0,
    ConnectError = 1,
    InitError = 2,
    HardwareError = 3,
    ModeError = 4,
    EnableError = 5,
    ConfigError = 6,
    MotionError = 7,
    PlanErr = 8,  // 轨迹规划失败（Ruckig InitPlan 等）
};

struct JointLimit {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V7d max_v;
    V7d max_a;
    V7d max_j;
};

struct CartLimit {
    double max_line_v;   // mm/s
    double max_line_a;   // mm/s²
    double max_line_j;   // mm/s³
    double max_angle_v;  // rad/s（YAML 为 deg/s）
    double max_angle_a;  // rad/s²（YAML 为 deg/s²）
    double max_angle_j;  // rad/s³（YAML 为 deg/s³）
};
