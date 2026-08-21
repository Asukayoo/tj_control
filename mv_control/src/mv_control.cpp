#include "mv_control.hpp"

#include "internal/ik.hpp"
#include "internal/math.hpp"
#include "internal/robot_impl.hpp"
#include "internal/sdk_map.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>

namespace {

constexpr int kArmCount = 2;

RobotState MakeZeroRobotState() {
    RobotState rs;
    rs.joint_state.q.setZero();
    rs.joint_state.v.setZero();
    rs.joint_state.a.setZero();
    rs.joint_state.j.setZero();
    rs.joint_state.tau.setZero();
    rs.cart_state.pose.pos.setZero();
    rs.cart_state.pose.quat = Quat::Identity();
    rs.cart_state.vel.setZero();
    rs.cart_state.jacob.setZero();
    return rs;
}

}  // namespace

// #region agent log
namespace {
void AgentDbgLog(const char* hypothesis_id, const char* location, const char* message,
                 const std::string& data_json = "{}") {
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
    std::ofstream out("/home/yxc/tj_control/.cursor/debug-c463d8.log", std::ios::app);
    if (!out) {
        return;
    }
    out << "{\"sessionId\":\"c463d8\",\"runId\":\"pre-fix\",\"hypothesisId\":\""
        << hypothesis_id << "\",\"location\":\"" << location << "\",\"message\":\""
        << message << "\",\"data\":" << data_json << ",\"timestamp\":" << ms << "}\n";
}
bool AgentMallocOk(std::size_t n) {
    void* p = std::malloc(n);
    if (p == nullptr) {
        return false;
    }
    std::free(p);
    return true;
}
}  // namespace
// #endregion

MVControl::MVControl() : left_(0), right_(1) {
    // #region agent log
    AgentDbgLog("B", "mv_control.cpp:MVControl()", "ctor_done",
                std::string("{\"malloc64\":") + (AgentMallocOk(64) ? "1" : "0") +
                    ",\"malloc1112\":" + (AgentMallocOk(1112) ? "1" : "0") + "}");
    // #endregion
}

MVControl::~MVControl() {
    if (hw_) {
        hw_->Close();
    }
    connected_ = false;
}

void MVControl::_FillArmWrite(Robot& arm, HwArmWrite& slot) {
    if (!arm.impl_->pending_state_queue_.empty()) {
        const StateCmdPackage pending = arm.impl_->pending_state_queue_.front();
        HwStateCommand hw_cmd{};
        hw_cmd.arm = arm.impl_->arm_serial_;
        hw_cmd.op = kStateCmdHwOpMap[static_cast<int>(pending.type)];
        hw_cmd.vel_percent = pending.vel_percent;
        hw_cmd.acc_percent = pending.acc_percent;
        hw_cmd.set_target_state = pending.set_target_state;
        hw_cmd.rot_type = pending.rot_type;
        hw_cmd.fc_adj_lmt = pending.fc_adj_lmt;
        for (int i = 0; i < DOF; ++i) {
            hw_cmd.k[i] = pending.k[i];
            hw_cmd.d[i] = pending.d[i];
            hw_cmd.cart_ctrl_para[i] = pending.cart_ctrl_para[i];
        }
        for (int i = 0; i < 6; ++i) {
            hw_cmd.fx_dir[i] = pending.fx_dir[i];
        }
        const bool is_stationary = arm.impl_->low_spd_flag_ == 1;
        if (!is_sim_ && !hw_->PrepareDeferredState(hw_cmd, is_stationary)) {
            return;  // 未就绪，保留队列，下周期再试
        }
        // 等 Write 成功（udp_sent）后再 pop；ClearSet 失败时否则会丢 Disable/Enable
        slot.has_state = true;
        slot.state = hw_cmd;
    }

    const Robot::Impl& impl = *arm.impl_;
    if (impl.enable_state_ != EnableState::Enabled ||
        impl.error_code_ != ErrorCode::Normal) {
        return;
    }
    const ControlMode mode = impl.control_mode_actual_;
    if (mode != ControlMode::Position && mode != ControlMode::JointImp &&
        mode != ControlMode::CartImp) {
        return;
    }
    if (impl.status_code_ != StatusCode::Ready &&
        impl.status_code_ != StatusCode::Running &&
        impl.status_code_ != StatusCode::Stopping) {
        return;
    }

    slot.has_stream = true;
    slot.send_joint = true;
    V7dToSdkDeg(impl.ref_rs_.joint_state.q, slot.q_deg);

    if (mode == ControlMode::JointImp || mode == ControlMode::CartImp) {
        slot.send_imp_kd = true;
        slot.imp_kd.is_cart = (mode == ControlMode::CartImp);
        if (mode == ControlMode::JointImp) {
            V7dToArray(impl.imp_config_.joint.K, slot.imp_kd.k);
            V7dToArray(impl.imp_config_.joint.D, slot.imp_kd.d);
        } else {
            V7dToArray(impl.imp_config_.cart.K, slot.imp_kd.k);
            V7dToArray(impl.imp_config_.cart.D, slot.imp_kd.d);
        }
    }
}

bool MVControl::Init(const char* config_path, bool is_sim,
                     std::shared_ptr<HwInterface> hw, const char* urdf_override) {
    if (connected_) {
        return is_sim_ == is_sim;
    }

    // #region agent log
    AgentDbgLog("A", "mv_control.cpp:Init", "before_make_unique_MvConfig",
                std::string("{\"malloc64\":") + (AgentMallocOk(64) ? "1" : "0") +
                    ",\"malloc4k\":" + (AgentMallocOk(4096) ? "1" : "0") +
                    ",\"sizeof_MvConfig\":" + std::to_string(sizeof(MvConfig)) +
                    ",\"sizeof_ImpConfig\":" + std::to_string(sizeof(ImpConfig)) + "}");
    // #endregion
    auto cfg = std::make_unique<MvConfig>();
    // #region agent log
    AgentDbgLog("A", "mv_control.cpp:Init", "after_make_unique_MvConfig",
                std::string("{\"malloc64\":") + (AgentMallocOk(64) ? "1" : "0") +
                    ",\"imp_joint_K0\":" + std::to_string(cfg->imp.joint.K(0)) +
                    ",\"imp_cart_K0\":" + std::to_string(cfg->imp.cart.K(0)) + "}");
    // #endregion
    if (!LoadMvConfig(config_path, *cfg)) {
        // #region agent log
        AgentDbgLog("A", "mv_control.cpp:Init", "LoadMvConfig_failed", "{}");
        // #endregion
        return false;
    }
    // #region agent log
    AgentDbgLog("A", "mv_control.cpp:Init", "after_LoadMvConfig",
                std::string("{\"malloc64\":") + (AgentMallocOk(64) ? "1" : "0") +
                    ",\"imp_joint_K0\":" + std::to_string(cfg->imp.joint.K(0)) +
                    ",\"runId\":\"post-fix\"}");
    // #endregion

    is_sim_ = is_sim;
    connect_cfg_ = cfg->connect;

    left_._ApplyConfig(cfg->left, cfg->servo, cfg->connect, cfg->imp);
    // #region agent log
    AgentDbgLog("C", "mv_control.cpp:Init", "after_ApplyConfig_left",
                std::string("{\"malloc64\":") + (AgentMallocOk(64) ? "1" : "0") + "}");
    // #endregion
    right_._ApplyConfig(cfg->right, cfg->servo, cfg->connect, cfg->imp);
    // #region agent log
    AgentDbgLog("C", "mv_control.cpp:Init", "after_ApplyConfig_right",
                std::string("{\"malloc64\":") + (AgentMallocOk(64) ? "1" : "0") + "}");
    // #endregion

    const char* urdf = nullptr;
    if (urdf_override != nullptr && urdf_override[0] != '\0') {
        urdf = urdf_override;
    } else if (!cfg->urdf_path.empty()) {
        urdf = cfg->urdf_path.c_str();
    }
    // #region agent log
    AgentDbgLog("E", "mv_control.cpp:Init", "before_IkSolver",
                std::string("{\"urdf_null\":") + (urdf == nullptr ? "1" : "0") + "}");
    // #endregion
    if (urdf != nullptr && !IkSolver::InitFromUrdf(urdf)) {
        left_.impl_->error_code_ = ErrorCode::InitError;
        right_.impl_->error_code_ = ErrorCode::InitError;
        return false;
    }
    // #region agent log
    AgentDbgLog("E", "mv_control.cpp:Init", "after_IkSolver",
                std::string("{\"malloc64\":") + (AgentMallocOk(64) ? "1" : "0") + "}");
    // #endregion
    if (!left_._Init(is_sim_) || !right_._Init(is_sim_)) {
        return false;
    }

    hw_ = hw ? std::move(hw) : HwInterface::Create(cfg->connect, is_sim_);

    if (!hw_->Open()) {
        left_.impl_->error_code_ = ErrorCode::InitError;
        right_.impl_->error_code_ = ErrorCode::InitError;
        return false;
    }

    connected_ = true;

    bool resp_ok = false;
    if (is_sim_) {
        const RobotState zero = MakeZeroRobotState();
        resp_ok = left_._SetRespState(zero) && right_._SetRespState(zero);
        if (resp_ok) {
            left_.impl_->sdk_detail_.arm_state = 0;
            left_.impl_->sdk_detail_.imp_type = 0;
            right_.impl_->sdk_detail_.arm_state = 0;
            right_.impl_->sdk_detail_.imp_type = 0;
            left_._UpdateEnableState();
            right_._UpdateEnableState();
        }
    } else {
        HwSnapshot snap{};
        resp_ok = hw_->Read(snap);
        if (resp_ok) {
            _ApplySnapshot(snap, false);
            std::array<Robot*, kArmCount> arms{{&left_, &right_}};
            for (int i = 0; i < kArmCount; ++i) {
                arms[i]->_UpdateEnableState();
                const int st = arms[i]->impl_->sdk_detail_.arm_state;
                if (!IsInitArmStateAllowed(st, arms[i]->impl_->strict_init_state_)) {
                    std::fprintf(stderr,
                                 "[InitError] arm=%d sdk CurState=%d (strict_init_state=%d)\n",
                                 i, st, arms[i]->impl_->strict_init_state_ ? 1 : 0);
                    arms[i]->impl_->error_code_ = ErrorCode::InitError;
                    resp_ok = false;
                    break;
                }
            }
        }
    }

    if (!resp_ok) {
        hw_->Close();
        connected_ = false;
        left_.impl_->error_code_ = ErrorCode::InitError;
        right_.impl_->error_code_ = ErrorCode::InitError;
        return false;
    }

    left_._SetRefState(left_.GetRespState());
    right_._SetRefState(right_.GetRespState());
    return true;
}

void MVControl::Run() {
    if (!connected_ || !hw_) {
        return;
    }

    HwSnapshot snap{};
    hw_->Read(snap);
    _ApplySnapshot(snap, true);

    std::array<Robot*, kArmCount> arms{{&left_, &right_}};
    for (Robot* arm : arms) {
        if (!arm->impl_->immediate_state_cmd_.has_value()) {
            continue;
        }
        const StateCmdPackage cmd = *arm->impl_->immediate_state_cmd_;
        arm->impl_->immediate_state_cmd_.reset();
        if (is_sim_ && cmd.type == StateCmdType::EStop) {
            arm->impl_->error_code_ = ErrorCode::HardwareError;
            arm->_UpdateEnableState();
            arm->_UpdateStatus();
            continue;
        }
        if (is_sim_ || cmd.type != StateCmdType::EStop) {
            continue;
        }
        HwStateCommand hw_cmd{};
        hw_cmd.arm = arm->impl_->arm_serial_;
        hw_cmd.op = HwStateCommand::Op::EStop;
        hw_->ExecuteImmediate(hw_cmd);
    }

    left_.Detect();
    right_.Detect();
    left_.RunLogic();
    right_.RunLogic();

    HwWriteRequest wr{};
    _FillArmWrite(left_, wr.left);
    _FillArmWrite(right_, wr.right);

    if (is_sim_) {
        if (wr.left.has_stream && wr.left.send_joint) {
            hw_->SetSimRefJoints(0, wr.left.q_deg);
        }
        if (wr.right.has_stream && wr.right.send_joint) {
            hw_->SetSimRefJoints(1, wr.right.q_deg);
        }
    }

    HwWriteResult wout{};
    hw_->Write(wr, wout);

    const bool commit_state = is_sim_ ? wout.ok : wout.udp_sent;
    if (commit_state) {
        if (wr.left.has_state && !left_.impl_->pending_state_queue_.empty()) {
            left_.impl_->pending_state_queue_.pop_front();
        }
        if (wr.right.has_state && !right_.impl_->pending_state_queue_.empty()) {
            right_.impl_->pending_state_queue_.pop_front();
        }
    }

    if (is_sim_ && (wout.had_state || wout.had_stream)) {
        hw_->Read(snap);
        _ApplySnapshot(snap, false);
        left_._UpdateEnableState();
        right_._UpdateEnableState();
    }

    last_hw_stats_.sent_this_cycle = wout.udp_sent;
    const auto& slot = hw_->SlotStats();
    last_hw_stats_.send_clear_fail_total = slot.clear_fail;
    last_hw_stats_.send_slot_wait_max_us = slot.wait_max_us;

    last_transition_diag_.left.enable_trans_cycles =
        static_cast<uint16_t>(left_.impl_->enable_transition_cycles_);
    last_transition_diag_.left.enable_trans_limit =
        static_cast<uint16_t>(left_.impl_->mode_transition_timeout_cycles_);
    last_transition_diag_.left.sdk_cur_state = left_.impl_->sdk_detail_.arm_state;
    last_transition_diag_.right.enable_trans_cycles =
        static_cast<uint16_t>(right_.impl_->enable_transition_cycles_);
    last_transition_diag_.right.enable_trans_limit =
        static_cast<uint16_t>(right_.impl_->mode_transition_timeout_cycles_);
    last_transition_diag_.right.sdk_cur_state = right_.impl_->sdk_detail_.arm_state;
}

void MVControl::ResetHwRunStats() {
    if (hw_) {
        hw_->ResetSlotStats();
    }
    last_hw_stats_ = {};
    last_transition_diag_ = {};
}

Robot& MVControl::Left() { return left_; }

Robot& MVControl::Right() { return right_; }

bool MVControl::BothArmsStationary() const {
    if (is_sim_) {
        return true;
    }
    return left_.impl_->low_spd_flag_ == 1 && right_.impl_->low_spd_flag_ == 1;
}

void MVControl::_ApplySnapshot(const HwSnapshot& snap, bool track_frame_serial) {
    std::array<Robot*, kArmCount> arms{{&left_, &right_}};
    const HwArmSnapshot* hw_arms[kArmCount] = {&snap.left, &snap.right};

    for (int i = 0; i < kArmCount; ++i) {
        arms[i]->impl_->read_buf_ok_ = snap.read_ok;
        if (!snap.read_ok) {
            continue;
        }
        RobotState rs;
        rs.joint_state.q.setZero();
        rs.joint_state.v.setZero();
        rs.joint_state.a.setZero();
        rs.joint_state.j.setZero();
        rs.joint_state.tau.setZero();
        for (int j = 0; j < DOF; ++j) {
            rs.joint_state.q(j) = hw_arms[i]->joint_pos_deg[j] * D2R;
            rs.joint_state.v(j) = hw_arms[i]->joint_vel_deg[j] * D2R;
            rs.joint_state.tau(j) = hw_arms[i]->joint_tau_nm[j];
        }
        arms[i]->_SetRespState(rs);
        arms[i]->impl_->sdk_detail_.arm_state = hw_arms[i]->arm_state;
        arms[i]->impl_->sdk_detail_.arm_err_code = hw_arms[i]->arm_err_code;
        arms[i]->impl_->sdk_detail_.imp_type = hw_arms[i]->imp_type;
        if (hw_arms[i]->servo_err_fresh) {
            arms[i]->impl_->sdk_detail_.servo_err = hw_arms[i]->servo_err;
            arms[i]->impl_->sdk_detail_.servo_err_fresh = true;
        }
        arms[i]->impl_->low_spd_flag_ = hw_arms[i]->low_spd_flag ? 1 : 0;

        if (!track_frame_serial || !connected_) {
            continue;
        }
        const int serial = hw_arms[i]->out_frame_serial;
        if (serial == 0) {
            continue;
        }
        if (arms[i]->impl_->sdk_last_frame_serial_ == 0 ||
            serial != arms[i]->impl_->sdk_last_frame_serial_) {
            arms[i]->impl_->sdk_last_frame_serial_ = serial;
            arms[i]->impl_->sdk_detail_.frame_stale_cycles = 0;
        } else if (hw_->WasSentLastCycle()) {
            arms[i]->impl_->sdk_detail_.frame_stale_cycles++;
        }
    }
}
