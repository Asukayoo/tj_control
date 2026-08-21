#pragma once

#include <array>
#include <string>

#include "common.hpp"

#ifndef MV_CONTROL_CONFIG_DEFAULT
#define MV_CONTROL_CONFIG_DEFAULT "mv_control/config/config.yaml"
#endif

struct ArmConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    int arm_serial = 0;
    V7d home_q = V7d::Zero();
    V7d work_q = V7d::Zero();
    JointLimit joint_limit{};
    CartLimit cart_limit{};
};

struct ServoPdGain {
    double p_gain = 0.0;
    double d_gain = 0.0;
};

struct ServoConfig {
    ServoPdGain servoj{500.0, 50.0};
    ServoPdGain servop{200.0, 50.0};
};

struct ConnectConfig {
    std::array<uint8_t, 4> ip{{192, 168, 1, 190}};
    int log_switch = 0;
    int vel_ratio = 10;
    int acc_ratio = 10;
    int mode_transition_timeout_ms = kTransitionTimeoutCycles;
    bool strict_init_state = true;
    int servo_err_poll_cycles = kServoErrPollCyclesDefault;  // 默认约 1Hz；0=Run 内不轮询
};

struct ImpJointConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // DEMO: showcase_offline_movl_keepj_execution.cpp（关节阻抗预设）
    V7d K = (V7d() << 2, 2, 2, 1.6, 1, 1, 1).finished();
    V7d D = V7d::Constant(0.4);
};

struct ImpCartConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // DEMO: showcase_offline_movl_execution.cpp
    // K[0..2] 平移 N/m；K[3..5] 旋转 N·m/rad；K[6] 零空间；D 阻尼比
    V7d K = (V7d() << 8000, 8000, 8000, 600, 600, 600, 20).finished();
    V7d D = (V7d() << 0.8, 0.8, 0.8, 0.4, 0.4, 0.4, 1).finished();
    int rot_type = 2;
    std::array<double, 7> cart_ctrl_para{};
};

struct ImpForceConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V6d fx_dir = (V6d() << 0, 0, 1, 0, 0, 0).finished();
    double fc_adj_lmt = 10.0;
};

struct ImpConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImpJointConfig joint{};
    ImpCartConfig cart{};
    ImpForceConfig force{};
};

struct MvConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ConnectConfig connect{};
    std::string urdf_path;
    ServoConfig servo{};
    ImpConfig imp{};
    ArmConfig left{};
    ArmConfig right{};
};

bool LoadMvConfig(const char* yaml_path, MvConfig& out);

// model: "615" 或 "696"；由 config_path 推导仓库根目录（启动交互选择时调用）
bool ResolveRobotModelUrdf(const char* config_path, const char* model,
                           std::string& out);
