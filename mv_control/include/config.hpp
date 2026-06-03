#pragma once

#include <array>
#include <string>

#include "common.hpp"

struct ArmConfig {
    int arm_serial = 0;
    V7d home_q = V7d::Zero();
    V7d work_q = V7d::Zero();
    JointLimit joint_limit{};
    CartLimit cart_limit{};
};

struct MvConfig {
    std::array<uint8_t, 4> connect_ip{{192, 168, 1, 190}};
    int log_switch = 0;
    std::string kine_cfg_path;
    ArmConfig left{};
    ArmConfig right{};
};

// 从 yaml 加载配置，path 可为相对路径
bool LoadMvConfig(const char* yaml_path, MvConfig& out);
