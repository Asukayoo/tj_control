#pragma once

#include <array>
#include <string>

#include "common.hpp"

#ifndef MV_CONTROL_CONFIG_DEFAULT
#define MV_CONTROL_CONFIG_DEFAULT "mv_control/config/config.yaml"
#endif

struct ArmConfig {
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
    int mode_transition_timeout_ms = 1000;
    bool strict_init_state = true;
    int servo_err_poll_cycles = 1000;  // 1kHz 下默认约 1Hz；0=Run 内不轮询
};

struct ImpJointConfig {
    V7d K = (V7d() << 100, 100, 100, 100, 50, 50, 50).finished();
    V7d D = V7d::Constant(0.5);
};

struct ImpCartConfig {
    V7d K = (V7d() << 500, 500, 500, 50, 50, 50, 20).finished();
    V7d D = V7d::Constant(0.5);
    int rot_type = 2;
    std::array<double, 7> cart_ctrl_para{};
};

struct ImpForceConfig {
    V6d fx_dir = (V6d() << 0, 0, 1, 0, 0, 0).finished();
    double fc_adj_lmt = 10.0;
};

struct ImpConfig {
    ImpJointConfig joint{};
    ImpCartConfig cart{};
    ImpForceConfig force{};
};

struct MvConfig {
    ConnectConfig connect{};
    std::string urdf_path;
    ServoConfig servo{};
    ImpConfig imp{};
    ArmConfig left{};
    ArmConfig right{};
};

bool LoadMvConfig(const char* yaml_path, MvConfig& out);
