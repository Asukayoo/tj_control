#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

constexpr int DOF = 7;
constexpr double PI = 3.14159265358979323846;
constexpr double D2R = PI / 180.0;
constexpr double R2D = 180.0 / PI;

using V3d = Eigen::Vector3d;
using V4d = Eigen::Vector4d;
using V6d = Eigen::Matrix<double, 6, 1>;
using V7d = Eigen::Matrix<double, 7, 1>;

using M3d = Eigen::Matrix<double, 3, 3>;
using M4d = Eigen::Matrix<double, 4, 4>;
using M6d = Eigen::Matrix<double, 6, 6>;
using M7d = Eigen::Matrix<double, 7, 7>;
using Jacob = Eigen::Matrix<double, 6, 7>;

using Quat = Eigen::Quaterniond;

struct JointState {
    V7d q;
    V7d v;
    V7d a;
    V7d j;
    V7d tau;
};

struct Pose {
    V3d pos;
    Quat quat;
};

struct CartState {
    Pose pose;
    V6d vel;
    Jacob jacob;
};

struct RobotState {
    JointState joint_state;
    CartState cart_state;
};

enum class ControlMode {
    Position = 0,
    JointImp = 1,
    CartImp = 2,
    Force = 3
};

enum class EnableMode {
    Disable = 0,
    Enable = 1
};

enum class StatusCode {
    Error = 0,
    Ready = 1,
    Running = 2,
    Stop = 3,
};

enum class ErrorCode {
    InitError = 0,
    Normal = 1,
    ServoError = 2,
    SizeError = 3,
    VelError = 4,
    IKError = 5
};

// rad/s rad/s^2 rad/s^3
struct JointLimit {
    V7d max_v;
    V7d max_a;
    V7d max_j;
};

// mm/s, mm/s^2, mm/s^3, rad/s, rad/s^2, rad/s^3
struct CartLimit {
    double max_line_v;
    double max_line_a;
    double max_line_j;
    double max_angle_v;
    double max_angle_a;
    double max_angle_j;
};
