#pragma once

#include <array>

#include <Eigen/Dense>
#include <Eigen/Geometry>

constexpr int DOF = 7;
constexpr double PI = 3.14159265358979323846;
constexpr double D2R = PI / 180.0;
constexpr double R2D = 180.0 / PI;
constexpr double kControlDt = 0.001;
// Servo 内部三次样条窗口（1kHz 下 40 周期 = 40ms）；与外部指令频率无关
constexpr int kStreamServoCycles = 40;
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
