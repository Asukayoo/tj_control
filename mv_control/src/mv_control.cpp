#include "mv_control.hpp"

#include "internal/ik.hpp"
#include "internal/math.hpp"
#include "internal/robot_impl.hpp"
#include "internal/sdk_map.hpp"

#include <array>
#include <cstdio>
#include <memory>

namespace {

constexpr int kArmCount = 2;

// SIM Init 须显式清零；RobotState 默认构造不保证 Eigen 成员为 0
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

MVControl::MVControl() : left_(0), right_(1) {}

MVControl::~MVControl() {
    if (hw_) {
        hw_->Close();
    }
    connected_ = false;
}

void MVControl::_QueueMotionIfNeeded(Robot& arm) {
    const Robot::Impl& impl = *arm.impl_;
    if (impl.enable_state_ != EnableState::Enabled ||
        impl.control_mode_actual_ != ControlMode::Position ||
        impl.error_code_ != ErrorCode::Normal ||
        (impl.status_code_ != StatusCode::Ready &&
         impl.status_code_ != StatusCode::Running &&
         impl.status_code_ != StatusCode::Stopping)) {
        return;
    }
    CmdPackage pkg;
    pkg.type = MotionType::PositionServo;
    pkg.q = impl.ref_rs_.joint_state.q;
    arm.impl_->pending_motion_queue_.push_back(pkg);
}

void MVControl::_DrainStateForWrite(Robot& arm, HwArmWrite& slot) {
    if (arm.impl_->pending_state_queue_.empty()) {
        return;
    }
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
    if (!is_sim_ && !hw_->PrepareDeferredState(hw_cmd)) {
        arm.impl_->pending_state_queue_.pop_front();
        if (pending.type == StateCmdType::Enable ||
            pending.type == StateCmdType::Disable) {
            arm.impl_->error_code_ = ErrorCode::EnableError;
        } else {
            arm.impl_->error_code_ = ErrorCode::ModeError;
        }
        return;
    }
    arm.impl_->pending_state_queue_.pop_front();
    slot.has_state = true;
    slot.state = hw_cmd;
}

void MVControl::_DrainMotionForWrite(Robot& arm, HwArmWrite& slot) {
    if (arm.impl_->pending_motion_queue_.empty()) {
        return;
    }
    slot.has_motion = true;
    const CmdPackage& pkg = arm.impl_->pending_motion_queue_.front();
    for (int i = 0; i < DOF; ++i) {
        slot.motion.q_deg[i] = pkg.q(i) * R2D;
    }
    arm.impl_->pending_motion_queue_.pop_front();
}

bool MVControl::Init(const char* config_path, bool is_sim,
                     std::shared_ptr<HwInterface> hw) {
    if (connected_) {
        return is_sim_ == is_sim;
    }

    auto cfg = std::make_unique<MvConfig>();
    if (!LoadMvConfig(config_path, *cfg)) {
        return false;
    }

    is_sim_ = is_sim;
    connect_cfg_ = cfg->connect;

    left_._ApplyConfig(cfg->left, cfg->servo, cfg->connect, cfg->imp);
    right_._ApplyConfig(cfg->right, cfg->servo, cfg->connect, cfg->imp);

    const char* urdf =
        cfg->urdf_path.empty() ? nullptr : cfg->urdf_path.c_str();
    if (urdf != nullptr && !IkSolver::InitFromUrdf(urdf)) {
        left_.impl_->error_code_ = ErrorCode::InitError;
        right_.impl_->error_code_ = ErrorCode::InitError;
        return false;
    }
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
        if (is_sim_) {
            if (cmd.type == StateCmdType::ClearError) {
                arm->impl_->error_code_ = ErrorCode::Normal;
                arm->_UpdateEnableState();
                arm->_UpdateStatus();
            }
            continue;
        }
        HwStateCommand hw_cmd{};
        hw_cmd.arm = arm->impl_->arm_serial_;
        hw_cmd.op = kStateCmdHwOpMap[static_cast<int>(cmd.type)];
        const bool ok = hw_->ExecuteImmediate(hw_cmd);
        if (cmd.type == StateCmdType::ClearError && ok) {
            arm->impl_->error_code_ = ErrorCode::Normal;
            arm->_UpdateEnableState();
            arm->_UpdateStatus();
        }
    }

    left_.Detect();
    right_.Detect();
    left_.RunLogic();
    right_.RunLogic();

    _QueueMotionIfNeeded(left_);
    _QueueMotionIfNeeded(right_);

    HwWriteRequest wr{};
    _BuildWriteRequest(wr);

    if (is_sim_) {
        double q_left[DOF]{};
        double q_right[DOF]{};
        if (wr.left.has_motion) {
            for (int i = 0; i < DOF; ++i) {
                q_left[i] = wr.left.motion.q_deg[i];
            }
            hw_->SetSimRefJoints(0, q_left);
        }
        if (wr.right.has_motion) {
            for (int i = 0; i < DOF; ++i) {
                q_right[i] = wr.right.motion.q_deg[i];
            }
            hw_->SetSimRefJoints(1, q_right);
        }
    }

    HwWriteResult wout{};
    hw_->Write(wr, wout);

    if (is_sim_ && (wr.left.has_motion || wr.right.has_motion || wr.left.has_state ||
                    wr.right.has_state)) {
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
        for (int j = 0; j < DOF; ++j) {
            rs.joint_state.q(j) = hw_arms[i]->joint_pos_deg[j] * D2R;
            rs.joint_state.v(j) = hw_arms[i]->joint_vel_deg[j] * D2R;
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

void MVControl::_BuildWriteRequest(HwWriteRequest& req) {
    _DrainStateForWrite(left_, req.left);
    _DrainMotionForWrite(left_, req.left);
    _DrainStateForWrite(right_, req.right);
    _DrainMotionForWrite(right_, req.right);
}
