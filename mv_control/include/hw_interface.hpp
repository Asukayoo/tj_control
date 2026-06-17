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
    std::array<long, DOF> servo_err{};
    bool servo_err_fresh = false;
};

struct HwSnapshot {
    HwArmSnapshot left;
    HwArmSnapshot right;
    bool read_ok = false;
};

// 状态写命令（Deferred 入 Robot 队列；Immediate 直接 ExecuteImmediate）
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

struct HwMotionCommand {
    double q_deg[DOF]{};
};

struct HwArmWrite {
    bool has_state = false;
    HwStateCommand state{};
    bool has_motion = false;
    HwMotionCommand motion{};
};

struct HwWriteRequest {
    HwArmWrite left;
    HwArmWrite right;
};

struct HwWriteResult {
    bool ok = true;
    bool udp_sent = false;
    bool had_state = false;
    bool had_motion = false;
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

    // ClearError / EStop：同步阻塞，不经 Deferred 队列
    bool ExecuteImmediate(const HwStateCommand& cmd);

    // SetEnable / SetControlMode 入队前校验（静止、读 CurState 等）
    bool PrepareDeferredState(HwStateCommand& cmd);

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
    return op == HwStateCommand::Op::ClearError || op == HwStateCommand::Op::EStop;
}

inline bool HwStateOpIsDeferred(HwStateCommand::Op op) {
    return !HwStateOpIsImmediate(op);
}
