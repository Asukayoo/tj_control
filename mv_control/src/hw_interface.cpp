#include "hw_interface.hpp"

#include "internal/diag.hpp"

#include "FxRtCSDef.h"
#include "MarvinSDK.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <unistd.h>

namespace {

constexpr int kFramePollTries = 5;
constexpr int kStationaryPollTries = 5;
constexpr int kSdkSendSleepMs = 10;
constexpr int kSendSlotStepUs = 20;
constexpr int kSendSlotTimeoutUs = 500;

void SleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void ClampVelAcc(int& vel, int& acc) {
    vel = std::clamp(vel, 1, 100);
    acc = std::clamp(acc, 1, 100);
}

bool IsValidIp(const ConnectConfig& cfg) {
    const auto& ip = cfg.ip;
    if (ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0) {
        return false;
    }
    if (ip[0] == 255 && ip[1] == 255 && ip[2] == 255 && ip[3] == 255) {
        return false;
    }
    if (ip[0] == 127) {
        return false;
    }
    return true;
}

void DecodeArm(HwArmSnapshot& arm, const DCSS& dcss, int idx) {
    arm.arm_state = dcss.m_State[idx].m_CurState;
    arm.arm_err_code = dcss.m_State[idx].m_ERRCode;
    arm.imp_type = dcss.m_In[idx].m_ImpType;
    arm.out_frame_serial = dcss.m_Out[idx].m_OutFrameSerial;
    arm.low_spd_flag = dcss.m_Out[idx].m_LowSpdFlag != 0;
    arm.is_fault = arm.arm_state == ARM_STATE_ERROR;
    arm.is_transition = arm.arm_state >= ARM_STATE_TRANS_TO_POSITION &&
                        arm.arm_state <= ARM_STATE_TRANS_TO_IDLE;
    for (int i = 0; i < DOF; ++i) {
        arm.joint_pos_deg[i] = dcss.m_Out[idx].m_FB_Joint_Pos[i];
        arm.joint_vel_deg[i] = dcss.m_Out[idx].m_FB_Joint_Vel[i];
    }
}

bool ServoHasError(int arm) {
    long err[DOF]{};
    if (arm == 0) {
        OnGetServoErr_A(err);
    } else {
        OnGetServoErr_B(err);
    }
    for (int i = 0; i < DOF; ++i) {
        if (err[i] != 0) {
            return true;
        }
    }
    return false;
}

bool CurrentModeMatches(const DCSS& dcss, int arm, int cur_state, int imp_type) {
    if (dcss.m_State[arm].m_CurState != cur_state) {
        return false;
    }
    if (cur_state == ARM_STATE_TORQ && dcss.m_In[arm].m_ImpType != imp_type) {
        return false;
    }
    return true;
}

using SetJointLmtFn = bool (*)(int, int);
using SetJointKdFn = bool (*)(double*, double*);
using SetCartKdFn = bool (*)(double*, double*, int);
using SetTargetStateFn = bool (*)(int);
using SetImpTypeFn = bool (*)(int);
using SetForceParaFn = bool (*)(int, double*, double*, double);
using SetEefRotFn = bool (*)(int, double*);

struct ArmOps {
    SetJointLmtFn set_joint_lmt;
    SetJointKdFn set_joint_kd;
    SetCartKdFn set_cart_kd;
    SetTargetStateFn set_target_state;
    SetImpTypeFn set_imp_type;
    SetForceParaFn set_force_para;
    SetEefRotFn set_eef_rot;
};

const ArmOps kArmOps[2] = {
    {OnSetJointLmt_A, OnSetJointKD_A, OnSetCartKD_A, OnSetTargetState_A,
     OnSetImpType_A, OnSetForceCtrPara_A, OnSetEefRot_A},
    {OnSetJointLmt_B, OnSetJointKD_B, OnSetCartKD_B, OnSetTargetState_B,
     OnSetImpType_B, OnSetForceCtrPara_B, OnSetEefRot_B},
};

const ArmOps& Ops(int arm) { return kArmOps[arm]; }

void ApplyLogSwitch(int log_switch) {
    if (log_switch == 0) {
        OnLogOff();
        OnLocalLogOff();
    } else {
        OnLogOn();
        OnLocalLogOn();
    }
}

}  // namespace

struct IHwBackend {
    virtual ~IHwBackend() = default;
    virtual bool Open() = 0;
    virtual void Close() = 0;
    virtual bool Read(HwSnapshot& snap) = 0;
    virtual bool Write(const HwWriteRequest& req, HwWriteResult& out) = 0;
    virtual bool ExecuteImmediate(const HwStateCommand& cmd) = 0;
    virtual bool PrepareDeferredState(HwStateCommand& cmd) = 0;
    virtual void SetSimRefJoints(int arm, const double q_deg[DOF]) = 0;

    HwRunSlotStats slot_stats{};
    ConnectConfig cfg{};
};

namespace {

class RealBackend : public IHwBackend {
public:
    bool Open() override {
        if (!IsValidIp(cfg)) {
            return false;
        }
        if (!OnLinkTo(cfg.ip[0], cfg.ip[1], cfg.ip[2], cfg.ip[3])) {
            return false;
        }
        for (int arm = 0; arm < 2; ++arm) {
            HwStateCommand clr{};
            clr.op = HwStateCommand::Op::ClearError;
            clr.arm = arm;
            if (!ExecuteImmediate(clr)) {
                return false;
            }
        }
        DCSS dcss{};
        int frame_update = 0;
        for (int i = 0; i < kFramePollTries; ++i) {
            if (!OnGetBuf(&dcss)) {
                SleepMs(1);
                continue;
            }
            if (dcss.m_Out[0].m_OutFrameSerial != 0 &&
                dcss.m_Out[0].m_OutFrameSerial != frame_update) {
                ApplyLogSwitch(cfg.log_switch);
                SleepMs(kSdkSendSleepMs);
                return true;
            }
            SleepMs(1);
        }
        return false;
    }

    void Close() override { OnRelease(); }

    bool Read(HwSnapshot& snap) override {
        DCSS dcss{};
        snap.read_ok = OnGetBuf(&dcss);
        if (!snap.read_ok) {
            return false;
        }
        DecodeArm(snap.left, dcss, 0);
        DecodeArm(snap.right, dcss, 1);
        if (cfg.servo_err_poll_cycles > 0) {
            ++servo_poll_cnt_;
            if (servo_poll_cnt_ >= cfg.servo_err_poll_cycles) {
                servo_poll_cnt_ = 0;
                OnGetServoErr_A(snap.left.servo_err.data());
                OnGetServoErr_B(snap.right.servo_err.data());
                snap.left.servo_err_fresh = true;
                snap.right.servo_err_fresh = true;
            }
        }
        return true;
    }

    bool Write(const HwWriteRequest& req, HwWriteResult& out) override {
        out = {};
        const bool want_state = req.left.has_state || req.right.has_state;
        const bool want_motion = req.left.has_motion || req.right.has_motion;
        if (!want_state && !want_motion) {
            return true;
        }

        if (!WaitAndClearSet()) {
            out.ok = false;
            return false;
        }

        if (req.left.has_state && !ApplyStateToBatch(req.left.state)) {
            out.ok = false;
            return false;
        }
        if (req.right.has_state && !ApplyStateToBatch(req.right.state)) {
            out.ok = false;
            return false;
        }
        out.had_state = want_state;

        if (req.left.has_motion) {
            double q[DOF];
            for (int i = 0; i < DOF; ++i) {
                q[i] = req.left.motion.q_deg[i];
            }
            if (!OnSetJointCmdPos_A(q)) {
                out.ok = false;
                return false;
            }
            out.had_motion = true;
        }
        if (req.right.has_motion) {
            double q[DOF];
            for (int i = 0; i < DOF; ++i) {
                q[i] = req.right.motion.q_deg[i];
            }
            if (!OnSetJointCmdPos_B(q)) {
                out.ok = false;
                return false;
            }
            out.had_motion = true;
        }

        out.udp_sent = OnSetSend();
        if (!out.udp_sent) {
            ++slot_stats.send_fail;
            out.ok = false;
        }
        return out.ok;
    }

    bool ExecuteImmediate(const HwStateCommand& cmd) override {
        if (cmd.op == HwStateCommand::Op::ClearError) {
            return ClearErrorImmediate(cmd.arm);
        }
        if (cmd.op == HwStateCommand::Op::EStop) {
            if (cmd.arm == 2) {
                OnEMG_AB();
            } else if (cmd.arm == 1) {
                OnEMG_B();
            } else {
                OnEMG_A();
            }
            return true;
        }
        return false;
    }

    bool PrepareDeferredState(HwStateCommand& cmd) override {
        if (cmd.arm < 0 || cmd.arm > 1) {
            return false;
        }
        if (cmd.op == HwStateCommand::Op::Disable) {
            return true;
        }
        ClampVelAcc(cmd.vel_percent, cmd.acc_percent);
        if (!IsStationary(cmd.arm)) {
            return false;
        }
        DCSS dcss{};
        if (!OnGetBuf(&dcss)) {
            return false;
        }
        switch (cmd.op) {
            case HwStateCommand::Op::Enable:
            case HwStateCommand::Op::SetPositionMode:
                cmd.set_target_state =
                    !CurrentModeMatches(dcss, cmd.arm, ARM_STATE_POSITION, 0);
                return true;
            case HwStateCommand::Op::SetJointImp:
                cmd.set_target_state = !CurrentModeMatches(dcss, cmd.arm,
                                                           ARM_STATE_TORQ, ARM_IMP_JOINT);
                return true;
            case HwStateCommand::Op::SetCartImp:
                cmd.set_target_state = !CurrentModeMatches(dcss, cmd.arm,
                                                           ARM_STATE_TORQ, ARM_IMP_CART);
                return true;
            case HwStateCommand::Op::SetForce:
                cmd.set_target_state = !CurrentModeMatches(dcss, cmd.arm,
                                                           ARM_STATE_TORQ, ARM_IMP_FORCE);
                return true;
            default:
                return true;
        }
    }

    void SetSimRefJoints(int, const double[DOF]) override {}

private:
    int servo_poll_cnt_ = 0;

    bool WaitAndClearSet() {
        const auto t0 = std::chrono::steady_clock::now();
        for (;;) {
            if (OnClearSet()) {
                const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                                         std::chrono::steady_clock::now() - t0)
                                         .count();
                slot_stats.wait_total_us += static_cast<uint64_t>(elapsed);
                if (static_cast<uint64_t>(elapsed) > slot_stats.wait_max_us) {
                    slot_stats.wait_max_us = static_cast<uint64_t>(elapsed);
                }
                return true;
            }
            const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                                     std::chrono::steady_clock::now() - t0)
                                     .count();
            if (elapsed >= kSendSlotTimeoutUs) {
                ++slot_stats.clear_fail;
                return false;
            }
            usleep(static_cast<useconds_t>(kSendSlotStepUs));
        }
    }

    bool IsStationary(int arm) {
        DCSS dcss{};
        for (int i = 0; i < kStationaryPollTries; ++i) {
            if (!OnGetBuf(&dcss)) {
                SleepMs(1);
                continue;
            }
            if (dcss.m_Out[arm].m_LowSpdFlag == 1) {
                return true;
            }
            SleepMs(1);
        }
        return false;
    }

    bool ClearErrorImmediate(int arm) {
        if (arm < 0 || arm > 1) {
            return false;
        }
        OnClearSet();
        if (arm == 0) {
            OnClearErr_A();
        } else {
            OnClearErr_B();
        }
        if (!OnSetSend()) {
            return false;
        }
        SleepMs(kSdkSendSleepMs);
        return !ServoHasError(arm);
    }

    bool ApplyStateToBatch(const HwStateCommand& cmd) {
        if (cmd.arm < 0 || cmd.arm > 1) {
            return false;
        }
        const ArmOps& ops = Ops(cmd.arm);
        switch (cmd.op) {
            case HwStateCommand::Op::Enable:
            case HwStateCommand::Op::SetPositionMode:
                if (!ops.set_joint_lmt(cmd.vel_percent, cmd.acc_percent)) {
                    return false;
                }
                if (cmd.set_target_state &&
                    !ops.set_target_state(ARM_STATE_POSITION)) {
                    return false;
                }
                return true;
            case HwStateCommand::Op::Disable:
                return ops.set_target_state(ARM_STATE_IDLE);
            case HwStateCommand::Op::SetJointImp: {
                if (!ops.set_joint_lmt(cmd.vel_percent, cmd.acc_percent)) {
                    return false;
                }
                double k[DOF];
                double d[DOF];
                for (int i = 0; i < DOF; ++i) {
                    k[i] = cmd.k[i];
                    d[i] = cmd.d[i];
                }
                if (!ops.set_joint_kd(k, d)) {
                    return false;
                }
                if (cmd.set_target_state) {
                    if (!ops.set_target_state(ARM_STATE_TORQ) ||
                        !ops.set_imp_type(ARM_IMP_JOINT)) {
                        return false;
                    }
                }
                return true;
            }
            case HwStateCommand::Op::SetCartImp: {
                if (!ops.set_joint_lmt(cmd.vel_percent, cmd.acc_percent)) {
                    return false;
                }
                double k[DOF];
                double d[DOF];
                for (int i = 0; i < DOF; ++i) {
                    k[i] = cmd.k[i];
                    d[i] = cmd.d[i];
                }
                if (!ops.set_cart_kd(k, d, 2)) {
                    return false;
                }
                if (cmd.rot_type != 0) {
                    double para[DOF];
                    for (int i = 0; i < DOF; ++i) {
                        para[i] = cmd.cart_ctrl_para[i];
                    }
                    if (!ops.set_eef_rot(cmd.rot_type, para)) {
                        return false;
                    }
                }
                if (cmd.set_target_state) {
                    if (!ops.set_target_state(ARM_STATE_TORQ) ||
                        !ops.set_imp_type(ARM_IMP_CART)) {
                        return false;
                    }
                }
                return true;
            }
            case HwStateCommand::Op::SetForce: {
                double fc_ctrl[7]{};
                double fx[6];
                for (int i = 0; i < 6; ++i) {
                    fx[i] = cmd.fx_dir[i];
                }
                if (!ops.set_force_para(0, fx, fc_ctrl, cmd.fc_adj_lmt)) {
                    return false;
                }
                if (cmd.set_target_state) {
                    if (!ops.set_target_state(ARM_STATE_TORQ) ||
                        !ops.set_imp_type(ARM_IMP_FORCE)) {
                        return false;
                    }
                }
                return true;
            }
            default:
                return false;
        }
    }
};

class SimBackend : public IHwBackend {
public:
    bool Open() override { return true; }
    void Close() override {}

    bool Read(HwSnapshot& snap) override {
        snap.read_ok = true;
        snap.left = arms_[0];
        snap.right = arms_[1];
        for (int i = 0; i < DOF; ++i) {
            if (has_ref_[0]) {
                snap.left.joint_pos_deg[i] = ref_q_deg_[0][i];
            }
            if (has_ref_[1]) {
                snap.right.joint_pos_deg[i] = ref_q_deg_[1][i];
            }
        }
        return true;
    }

    bool Write(const HwWriteRequest& req, HwWriteResult& out) override {
        out = {};

        auto apply_state = [this](const HwArmWrite& slot, int arm) {
            if (!slot.has_state) {
                return;
            }
            switch (slot.state.op) {
                case HwStateCommand::Op::Enable:
                case HwStateCommand::Op::SetPositionMode:
                case HwStateCommand::Op::SetJointImp:
                case HwStateCommand::Op::SetCartImp:
                case HwStateCommand::Op::SetForce:
                    arms_[arm].arm_state = 1;
                    break;
                case HwStateCommand::Op::Disable:
                    arms_[arm].arm_state = 0;
                    break;
                default:
                    break;
            }
        };

        if (req.left.has_state) {
            apply_state(req.left, 0);
            out.had_state = true;
        }
        if (req.right.has_state) {
            apply_state(req.right, 1);
            out.had_state = true;
        }
        if (req.left.has_motion || req.right.has_motion) {
            out.had_motion = true;
        }
        out.ok = true;
        return true;
    }

    bool ExecuteImmediate(const HwStateCommand& cmd) override {
        if (cmd.op == HwStateCommand::Op::ClearError) {
            if (cmd.arm == 0 || cmd.arm == 1) {
                arms_[cmd.arm].arm_err_code = 0;
                if (arms_[cmd.arm].arm_state == ARM_STATE_ERROR) {
                    arms_[cmd.arm].arm_state = 0;
                }
                arms_[cmd.arm].servo_err.fill(0);
            }
            return true;
        }
        if (cmd.op == HwStateCommand::Op::EStop) {
            if (cmd.arm == 2) {
                arms_[0].arm_state = 0;
                arms_[1].arm_state = 0;
            } else if (cmd.arm == 0 || cmd.arm == 1) {
                arms_[cmd.arm].arm_state = 0;
            }
            return true;
        }
        return false;
    }

    bool PrepareDeferredState(HwStateCommand& cmd) override {
        (void)cmd;
        return true;
    }

    void SetSimRefJoints(int arm, const double q_deg[DOF]) override {
        if (arm < 0 || arm > 1) {
            return;
        }
        for (int i = 0; i < DOF; ++i) {
            ref_q_deg_[arm][i] = q_deg[i];
        }
        has_ref_[arm] = true;
    }

private:
    HwArmSnapshot arms_[2]{};
    double ref_q_deg_[2][DOF]{};
    bool has_ref_[2]{false, false};
};

}  // namespace

struct HwInterface::Impl {
    std::unique_ptr<IHwBackend> backend;
    bool sent_last_cycle = false;
    bool sent_this_cycle = false;

    bool Open() { return backend->Open(); }
    void Close() { backend->Close(); }
    bool Read(HwSnapshot& snap) { return backend->Read(snap); }
    bool Write(const HwWriteRequest& req, HwWriteResult& out) {
        sent_last_cycle = sent_this_cycle;
        sent_this_cycle = false;
        const bool ok = backend->Write(req, out);
        if (out.udp_sent) {
            sent_this_cycle = true;
        }
        return ok;
    }
    bool ExecuteImmediate(const HwStateCommand& cmd) {
        return backend->ExecuteImmediate(cmd);
    }
    bool PrepareDeferredState(HwStateCommand& cmd) {
        return backend->PrepareDeferredState(cmd);
    }
    void SetSimRefJoints(int arm, const double q_deg[DOF]) {
        backend->SetSimRefJoints(arm, q_deg);
    }
    const HwRunSlotStats& SlotStats() const { return backend->slot_stats; }
    void ResetSlotStats() { backend->slot_stats = {}; }
};

HwInterface::HwInterface(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

std::shared_ptr<HwInterface> HwInterface::Create(const ConnectConfig& cfg, bool sim) {
    std::unique_ptr<IHwBackend> backend;
    if (sim) {
        backend = std::make_unique<SimBackend>();
    } else {
        backend = std::make_unique<RealBackend>();
    }
    backend->cfg = cfg;
    auto impl = std::make_unique<Impl>();
    impl->backend = std::move(backend);
    return std::shared_ptr<HwInterface>(new HwInterface(std::move(impl)));
}

bool HwInterface::Open() { return impl_->Open(); }

void HwInterface::Close() { impl_->Close(); }

bool HwInterface::Read(HwSnapshot& snap) { return impl_->Read(snap); }

bool HwInterface::Write(const HwWriteRequest& req, HwWriteResult& out) {
    return impl_->Write(req, out);
}

bool HwInterface::ExecuteImmediate(const HwStateCommand& cmd) {
    return impl_->ExecuteImmediate(cmd);
}

bool HwInterface::PrepareDeferredState(HwStateCommand& cmd) {
    return impl_->PrepareDeferredState(cmd);
}

void HwInterface::SetSimRefJoints(int arm, const double q_deg[DOF]) {
    impl_->SetSimRefJoints(arm, q_deg);
}

bool HwInterface::WasSentLastCycle() const { return impl_->sent_last_cycle; }

bool HwInterface::WasSentThisCycle() const { return impl_->sent_this_cycle; }

const HwRunSlotStats& HwInterface::SlotStats() const { return impl_->SlotStats(); }

void HwInterface::ResetSlotStats() { impl_->ResetSlotStats(); }
