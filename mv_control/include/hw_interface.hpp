#pragma once

#include "common.hpp"
#include "config.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>

// 单臂硬件快照
struct HwArmSnapshot {
    int arm_state = 0;
    int arm_err_code = 0;
    int imp_type = 0;
    int out_frame_serial = 0;
    bool low_spd_flag = false;
    bool is_fault = false;
    bool is_transition = false;
    double joint_pos_deg[DOF]{};
    double joint_vel_deg[DOF]{};
    double joint_tau_nm[DOF]{};  // m_Out.m_FB_Joint_SToq，Nm
    std::array<long, DOF> servo_err{};
    bool servo_err_fresh = false;
};

struct HwSnapshot {
    HwArmSnapshot left;
    HwArmSnapshot right;
    bool read_ok = false;
};

// 状态写命令（Deferred 入 Robot 队列，Run 内非阻塞；仅 EStop 走 Immediate）
struct HwStateCommand {
    enum class Op {
        Enable,
        Disable,
        ClearError,
        EStop,
        SetPositionMode,
        SetJointImp,
        SetCartImp,
        SetForce,
    };

    Op op = Op::Disable;
    int arm = 0;
    int vel_percent = 10;
    int acc_percent = 10;
    bool set_target_state = true;
    double k[DOF]{};
    double d[DOF]{};
    int rot_type = 0;
    double cart_ctrl_para[DOF]{};
    double fx_dir[6]{};
    double fc_adj_lmt = 10.0;
};

// 阻抗模式每周期 K/D（与关节指令同批发送）
struct HwImpKd {
    double k[DOF]{};
    double d[DOF]{};
    bool is_cart = false;
};

struct HwArmWrite {
    bool has_state = false;
    HwStateCommand state{};
    // 使能后每周期流：关节目标 + 阻抗 K/D（按模式）
    bool has_stream = false;
    bool send_joint = false;
    bool send_imp_kd = false;
    double q_deg[DOF]{};
    HwImpKd imp_kd{};
};

struct HwWriteRequest {
    HwArmWrite left;
    HwArmWrite right;
};

struct HwWriteResult {
    bool ok = true;
    bool udp_sent = false;
    bool had_state = false;
    bool had_stream = false;
};

struct HwRunSlotStats {
    uint64_t wait_total_us = 0;
    uint64_t wait_max_us = 0;
    uint64_t clear_fail = 0;
    uint64_t send_fail = 0;
};

class HwInterface {
public:
    static std::shared_ptr<HwInterface> Create(const ConnectConfig& cfg, bool sim);

    bool Open();
    void Close();

    bool Read(HwSnapshot& snap);
    bool Write(const HwWriteRequest& req, HwWriteResult& out);

    // EStop：Run 内立即下发（无 sleep）
    bool ExecuteImmediate(const HwStateCommand& cmd);

    // 状态命令入队前校验；未就绪则返回 false，下周期重试（不阻塞）
    bool PrepareDeferredState(HwStateCommand& cmd, bool is_stationary);

    bool WasSentLastCycle() const;
    bool WasSentThisCycle() const;
    const HwRunSlotStats& SlotStats() const;
    void ResetSlotStats();

    // Sim：Write 后镜像 ref 关节到 Read
    void SetSimRefJoints(int arm, const double q_deg[DOF]);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    explicit HwInterface(std::unique_ptr<Impl> impl);
};

inline bool HwStateOpIsImmediate(HwStateCommand::Op op) {
    return op == HwStateCommand::Op::EStop;
}

inline bool HwStateOpIsDeferred(HwStateCommand::Op op) {
    return !HwStateOpIsImmediate(op);
}
