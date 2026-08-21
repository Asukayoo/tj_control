#pragma once

#include "common.hpp"

#include <array>

// 控制周期见 common.hpp；SDK 帧/超时常量见 common.hpp（kSdkFrame* / kTransition*）
// SDK m_OutFrameSerial：Init 轮询次数 / Run 无刷新阈值

// SDK 原始状态，仅供 Detect / 清错 / 慢速轮询
struct SdkErrorDetail {
    int arm_state = 0;
    int arm_err_code = 0;
    int imp_type = 0;
    std::array<long, 7> servo_err{};
    int frame_stale_cycles = 0;
    bool servo_err_fresh = false;
};

enum class MotionType {
    None = 0,
    Stop = 1,
    ServoJ = 2,
    ServoP = 3,
    MovJ = 4,
    MovL = 5,
    ServoPByPico = 6,
};

enum class StateCmdType {
    Enable = 0,
    Disable = 1,
    ClearError = 2,
    EStop = 3,
    SetPositionMode = 4,
    SetJointImp = 5,
    SetCartImp = 6,
    SetForce = 7,
};

inline bool IsStreamMotion(MotionType type) {
    return type == MotionType::ServoJ || type == MotionType::ServoP ||
           type == MotionType::ServoPByPico;
}

// 运动指令包：用户指令与硬件输出共用
struct CmdPackage {
    MotionType type = MotionType::Stop;
    V7d q = V7d::Zero();
    Pose pose{};

    void AssignFrom(const CmdPackage& other) {
        type = other.type;
        q = other.q;
        pose = other.pose;
    }
};

// 状态指令包：使能/模式/清错等统一表示
struct StateCmdPackage {
    StateCmdType type = StateCmdType::Disable;
    int vel_percent = 10;
    int acc_percent = 10;
    bool set_target_state = true;
    double k[DOF]{};
    double d[DOF]{};
    int rot_type = 0;
    double cart_ctrl_para[DOF]{};
    double fx_dir[6]{};
    double fc_adj_lmt = 10.0;

    void AssignFrom(const StateCmdPackage& other) {
        type = other.type;
        vel_percent = other.vel_percent;
        acc_percent = other.acc_percent;
        set_target_state = other.set_target_state;
        rot_type = other.rot_type;
        fc_adj_lmt = other.fc_adj_lmt;
        for (int i = 0; i < DOF; ++i) {
            k[i] = other.k[i];
            d[i] = other.d[i];
            cart_ctrl_para[i] = other.cart_ctrl_para[i];
        }
        for (int i = 0; i < 6; ++i) {
            fx_dir[i] = other.fx_dir[i];
        }
    }
};
