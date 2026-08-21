#include "robot.hpp"

#include "internal/diag.hpp"
#include "internal/robot_impl.hpp"
#include "internal/sdk_map.hpp"
#include "internal/ik.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace {

bool UpdateCartFromJoint(int arm_serial, RobotState& rs) {
    if (!IkSolver::IsReady()) {
        return false;
    }
    if (!IkSolver::Forward(arm_serial, rs.joint_state.q, rs.cart_state.pose)) {
        return false;
    }
    Jacob j_arm;
    if (IkSolver::Jacobian(arm_serial, rs.joint_state.q, j_arm)) {
        rs.cart_state.jacob = j_arm;
        rs.cart_state.vel = j_arm * rs.joint_state.v;
    } else {
        rs.cart_state.jacob.setZero();
        rs.cart_state.vel.setZero();
    }
    return true;
}

JointLimit DefaultJointLimit() {
    JointLimit lim;
    lim.max_v = V7d::Constant(1.0);
    lim.max_a = V7d::Constant(3.0);
    lim.max_j = V7d::Constant(30.0);
    return lim;
}

CartLimit DefaultCartLimit() {
    CartLimit lim;
    lim.max_line_v = 200.0;
    lim.max_line_a = 1000.0;
    lim.max_line_j = 5000.0;
    lim.max_angle_v = 1.0;
    lim.max_angle_a = 3.0;
    lim.max_angle_j = 30.0;
    return lim;
}

bool IsHardwareRelatedError(ErrorCode code) {
    switch (code) {
        case ErrorCode::ConnectError:
        case ErrorCode::HardwareError:
        case ErrorCode::ModeError:
        case ErrorCode::EnableError:
        case ErrorCode::ConfigError:
            return true;
        default:
            return false;
    }
}

void V7dToArray(const V7d& src, double dst[DOF]) {
    for (int i = 0; i < DOF; ++i) {
        dst[i] = src(i);
    }
}

const char* EnableStateName(EnableState s) {
    switch (s) {
        case EnableState::Disabled:
            return "Disabled";
        case EnableState::Enabling:
            return "Enabling";
        case EnableState::Enabled:
            return "Enabled";
        case EnableState::Disabling:
            return "Disabling";
    }
    return "?";
}

const char* ErrorCodeName(ErrorCode e) {
    switch (e) {
        case ErrorCode::Normal:
            return "Normal";
        case ErrorCode::ConnectError:
            return "ConnectError";
        case ErrorCode::InitError:
            return "InitError";
        case ErrorCode::HardwareError:
            return "HardwareError";
        case ErrorCode::ModeError:
            return "ModeError";
        case ErrorCode::EnableError:
            return "EnableError";
        case ErrorCode::ConfigError:
            return "ConfigError";
        case ErrorCode::MotionError:
            return "MotionError";
        case ErrorCode::PlanErr:
            return "PlanErr";
    }
    return "?";
}

}  // namespace

void Robot::_LogErrorChange(ErrorCode prev, const char* source) {
    if (impl_->error_code_ == prev) {
        return;
    }
    MvDiag::LogEnable(impl_->arm_serial_, "err %s→%s (%s)",
                      ErrorCodeName(prev), ErrorCodeName(impl_->error_code_), source);
    MvDiag::LogVerbose(
        impl_->arm_serial_,
        "err detail: en=%s sdk=%d sdk_err=%d LowSpd=%d via %s",
        EnableStateName(impl_->enable_state_), impl_->sdk_detail_.arm_state,
        impl_->sdk_detail_.arm_err_code, impl_->low_spd_flag_, source);
    impl_->diag_last_error_ = impl_->error_code_;
    if (impl_->error_code_ == ErrorCode::Normal) {
        impl_->diag_logged_enable_mismatch_ = false;
    }
}

void Robot::_PostEnableDiag(EnableState prev_enable) {
    if (impl_->enable_state_ != prev_enable) {
        MvDiag::LogEnable(impl_->arm_serial_, "%s→%s",
                          EnableStateName(prev_enable),
                          EnableStateName(impl_->enable_state_));
        MvDiag::LogVerbose(
            impl_->arm_serial_,
            "enable detail: mode=%s sdk=%d sdk_err=%d err=%s",
            impl_->enable_mode_ == EnableMode::Enable ? "Enable" : "Disable",
            impl_->sdk_detail_.arm_state, impl_->sdk_detail_.arm_err_code,
            ErrorCodeName(impl_->error_code_));
        impl_->diag_last_enable_ = impl_->enable_state_;
    }
    if (impl_->enable_state_ == EnableState::Enabled &&
        impl_->error_code_ != ErrorCode::Normal && !impl_->diag_logged_enable_mismatch_) {
        impl_->diag_logged_enable_mismatch_ = true;
        MvDiag::LogEnable(impl_->arm_serial_,
                          "MISMATCH: Enabled 但 err=%s，_CanAcceptCmd=false",
                          ErrorCodeName(impl_->error_code_));
    }
}

Robot::Impl::Impl(int arm_serial, const JointLimit& joint_lim,
                  const CartLimit& cart_lim)
    : arm_serial_(arm_serial),
      motion_stop_(joint_lim),
      motion_movj_(joint_lim),
      motion_movl_(cart_lim, arm_serial),
      motion_servoj_(joint_lim),
      motion_servop_(cart_lim, arm_serial),
      motion_servop_pico_(cart_lim, arm_serial) {
    // #region agent log
    {
        std::ofstream out("/home/yxc/tj_control/.cursor/debug-c463d8.log", std::ios::app);
        if (out) {
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
            out << "{\"sessionId\":\"c463d8\",\"runId\":\"pre-fix\",\"hypothesisId\":\"B\","
                   "\"location\":\"robot.cpp:Impl::Impl\",\"message\":\"impl_ctor_done\","
                   "\"data\":{\"arm\":"
                << arm_serial << ",\"sizeof_Impl\":" << sizeof(Impl)
                << ",\"imp_K0\":" << imp_config_.joint.K(0)
                << "},\"timestamp\":" << ms << "}\n";
        }
    }
    // #endregion
}

Robot::Robot(int arm_serial)
    : impl_(std::make_unique<Impl>(
          arm_serial, DefaultJointLimit(), DefaultCartLimit())) {}

Robot::~Robot() = default;

Robot::Robot(Robot&&) noexcept = default;
Robot& Robot::operator=(Robot&&) noexcept = default;

bool Robot::_Init(bool is_sim) {
    impl_->is_sim_ = is_sim;
    impl_->status_code_ = StatusCode::Disabled;
    impl_->error_code_ = ErrorCode::Normal;
    impl_->enable_mode_ = EnableMode::Disable;
    impl_->enable_state_ = EnableState::Disabled;
    impl_->control_mode_target_ = ControlMode::Position;
    impl_->control_mode_actual_ = ControlMode::Position;
    impl_->pending_state_queue_.clear();
    impl_->immediate_state_cmd_.reset();
    return true;
}

void Robot::_ApplyConfig(const ArmConfig& arm, const ServoConfig& servo,
                         const ConnectConfig& connect, const ImpConfig& imp) {
    impl_->arm_serial_ = arm.arm_serial;
    impl_->home_q_ = arm.home_q;
    impl_->work_q_ = arm.work_q;
    impl_->joint_limit_ = arm.joint_limit;
    impl_->cart_limit_ = arm.cart_limit;
    impl_->motion_movj_.SetLimit(impl_->joint_limit_);
    impl_->motion_movl_.SetLimit(impl_->cart_limit_);
    impl_->motion_servoj_.SetLimit(impl_->joint_limit_);
    impl_->motion_servop_.SetLimit(impl_->cart_limit_);
    impl_->motion_servop_pico_.SetLimit(impl_->cart_limit_);
    impl_->motion_stop_.SetLimit(impl_->joint_limit_);

    impl_->motion_servoj_.SetPdGain(servo.servoj.p_gain, servo.servoj.d_gain);
    impl_->motion_servop_.SetPdGain(servo.servop.p_gain, servo.servop.d_gain);
    impl_->motion_servop_pico_.SetPdGain(servo.servop.p_gain, servo.servop.d_gain);

    impl_->vel_ratio_ = connect.vel_ratio;
    impl_->acc_ratio_ = connect.acc_ratio;
    impl_->strict_init_state_ = connect.strict_init_state;
    impl_->mode_transition_timeout_cycles_ = connect.mode_transition_timeout_ms;
    if (impl_->mode_transition_timeout_cycles_ < 1) {
        impl_->mode_transition_timeout_cycles_ = 1;
    }

    impl_->imp_config_ = imp;
}

void Robot::_ClearMotionCmds() {
    impl_->cmd_queue_.clear();
    impl_->stream_cmd_.reset();
    impl_->stream_dirty_ = false;
    if (impl_->active_motion_ == MotionType::ServoPByPico) {
        impl_->motion_servop_pico_.ResetSession();
    }
    impl_->stop_pending_ = false;
    impl_->active_motion_ = MotionType::None;
    impl_->motion_inited_ = false;
    impl_->active_cmd_.reset();
}

bool Robot::SetEnable(EnableMode mode) {
    impl_->enable_mode_ = mode;

    if (impl_->is_sim_) {
        if (mode == EnableMode::Enable) {
            if (!_CallSdkControlMode(impl_->control_mode_target_)) {
                impl_->error_code_ = ErrorCode::ModeError;
                return false;
            }
            impl_->enable_state_ = EnableState::Enabling;
            impl_->enable_transition_cycles_ = 0;
            return true;
        }
        _ClearMotionCmds();
        impl_->control_mode_target_ = ControlMode::Position;
        impl_->control_mode_actual_ = ControlMode::Position;
        StateCmdPackage cmd{};
        cmd.type = StateCmdType::Disable;
        impl_->pending_state_queue_.push_back(cmd);
        impl_->enable_state_ = EnableState::Disabling;
        impl_->enable_transition_cycles_ = 0;
        return true;
    }

    if (mode == EnableMode::Enable && impl_->enable_state_ == EnableState::Enabled) {
        return true;
    }
    if (mode == EnableMode::Disable && impl_->enable_state_ == EnableState::Disabled) {
        return true;
    }

    if (mode == EnableMode::Enable) {
        MvDiag::LogVerbose(
            impl_->arm_serial_,
            "SetEnable(Enable) mode=%d enable_state=%s err=%s sdk_CurState=%d LowSpdFlag=%d",
            static_cast<int>(impl_->control_mode_target_),
            EnableStateName(impl_->enable_state_), ErrorCodeName(impl_->error_code_),
            impl_->sdk_detail_.arm_state, impl_->low_spd_flag_);
        if (!_CallSdkControlMode(impl_->control_mode_target_)) {
            impl_->error_code_ = ErrorCode::ModeError;
            return false;
        }
        const EnableState prev_en = impl_->enable_state_;
        impl_->enable_state_ = EnableState::Enabling;
        impl_->enable_transition_cycles_ = 0;
        _PostEnableDiag(prev_en);
    } else {
        _ClearMotionCmds();
        impl_->control_mode_target_ = ControlMode::Position;
        impl_->control_mode_actual_ = ControlMode::Position;
        StateCmdPackage cmd{};
        cmd.type = StateCmdType::Disable;
        impl_->pending_state_queue_.push_back(cmd);
        const EnableState prev_en = impl_->enable_state_;
        impl_->enable_state_ = EnableState::Disabling;
        impl_->enable_transition_cycles_ = 0;
        _PostEnableDiag(prev_en);
    }
    return true;
}

EnableState Robot::GetEnableState() const { return impl_->enable_state_; }

void Robot::EStop() {
    _ClearMotionCmds();
    if (!impl_->is_sim_) {
        StateCmdPackage cmd{};
        cmd.type = StateCmdType::EStop;
        impl_->immediate_state_cmd_ = cmd;
    }
    impl_->error_code_ = ErrorCode::HardwareError;
    _EnterStopOnFault();
}

void Robot::_UpdateEnableState() {
    const int st = impl_->sdk_detail_.arm_state;
    if (st == 0 || st == 1 || st == 2 || st == 3 || st == 4 || st == 100) {
        // 下使能且 CurState==0：保留 SetControlMode 预设，不从 SDK 覆盖
        if (!(st == 0 && impl_->enable_state_ == EnableState::Disabled)) {
            impl_->control_mode_actual_ =
                MapSdkToControlMode(st, impl_->sdk_detail_.imp_type);
        }
    }

    if (impl_->is_sim_) {
        if (st == 0) {
            impl_->enable_state_ = impl_->enable_mode_ == EnableMode::Enable
                                       ? EnableState::Enabling
                                       : EnableState::Disabled;
            return;
        }
        if (IsModeTransitionState(st)) {
            impl_->enable_state_ = impl_->enable_mode_ == EnableMode::Enable
                                       ? EnableState::Enabling
                                       : EnableState::Disabling;
            return;
        }
        if (impl_->enable_mode_ == EnableMode::Enable && (st == 1 || st == 3)) {
            impl_->enable_state_ = EnableState::Enabled;
            return;
        }
        if (impl_->enable_mode_ == EnableMode::Disable) {
            impl_->enable_state_ = EnableState::Disabling;
            return;
        }
        impl_->enable_state_ = EnableState::Disabled;
        return;
    }

    if (st == 1 && impl_->control_mode_actual_ == ControlMode::Position) {
        impl_->enable_state_ = (impl_->enable_mode_ == EnableMode::Disable)
                                   ? EnableState::Disabling
                                   : EnableState::Enabled;
        return;
    }
    if (st == 3 && impl_->enable_mode_ == EnableMode::Enable) {
        impl_->enable_state_ = EnableState::Enabled;
        return;
    }
    if (st == 0) {
        if (impl_->enable_mode_ == EnableMode::Enable) {
            impl_->enable_state_ = EnableState::Enabling;
        } else {
            impl_->enable_state_ = EnableState::Disabled;
        }
        return;
    }
    if (IsModeTransitionState(st)) {
        impl_->enable_state_ = (impl_->enable_mode_ == EnableMode::Enable) ? EnableState::Enabling
                                                             : EnableState::Disabling;
        return;
    }
    if (st == 100) {
        return;
    }
    if (impl_->enable_mode_ == EnableMode::Disable) {
        impl_->enable_state_ = EnableState::Disabling;
        return;
    }
    if (impl_->enable_mode_ == EnableMode::Enable) {
        impl_->enable_state_ = EnableState::Enabling;
        return;
    }
    impl_->enable_state_ = EnableState::Disabled;
}

bool Robot::_CallSdkControlMode(ControlMode mode) {
    StateCmdPackage cmd{};
    cmd.vel_percent = impl_->vel_ratio_;
    cmd.acc_percent = impl_->acc_ratio_;

    double k[DOF];
    double d[DOF];
    switch (mode) {
        case ControlMode::Position:
            cmd.type = StateCmdType::SetPositionMode;
            impl_->pending_state_queue_.push_back(cmd);
            return true;
        case ControlMode::JointImp:
            cmd.type = StateCmdType::SetJointImp;
            V7dToArray(impl_->imp_config_.joint.K, k);
            V7dToArray(impl_->imp_config_.joint.D, d);
            for (int i = 0; i < DOF; ++i) {
                cmd.k[i] = k[i];
                cmd.d[i] = d[i];
            }
            impl_->pending_state_queue_.push_back(cmd);
            return true;
        case ControlMode::CartImp:
            cmd.type = StateCmdType::SetCartImp;
            V7dToArray(impl_->imp_config_.cart.K, k);
            V7dToArray(impl_->imp_config_.cart.D, d);
            for (int i = 0; i < DOF; ++i) {
                cmd.k[i] = k[i];
                cmd.d[i] = d[i];
            }
            cmd.rot_type = impl_->imp_config_.cart.rot_type;
            for (int i = 0; i < DOF; ++i) {
                cmd.cart_ctrl_para[i] = impl_->imp_config_.cart.cart_ctrl_para[i];
            }
            impl_->pending_state_queue_.push_back(cmd);
            return true;
        case ControlMode::Force: {
            cmd.type = StateCmdType::SetForce;
            for (int i = 0; i < 6; ++i) {
                cmd.fx_dir[i] = impl_->imp_config_.force.fx_dir(i);
            }
            cmd.fc_adj_lmt = impl_->imp_config_.force.fc_adj_lmt;
            impl_->pending_state_queue_.push_back(cmd);
            return true;
        }
    }
    return false;
}

bool Robot::SetControlMode(ControlMode mode) {
    // 安全策略：仅下使能（CurState==0）时允许改模式；实际上切模式在 SetEnable(Enable) 时发 SDK
    if (impl_->enable_state_ != EnableState::Disabled) {
        impl_->error_code_ = ErrorCode::ModeError;
        return false;
    }
    if (!impl_->is_sim_ && impl_->sdk_detail_.arm_state != 0) {
        impl_->error_code_ = ErrorCode::ModeError;
        return false;
    }
    impl_->control_mode_target_ = mode;
    impl_->control_mode_actual_ = mode;
    return true;
}

ControlModeStatus Robot::GetControlModeStatus() const {
    if (impl_->is_sim_) {
        if (impl_->control_mode_target_ != impl_->control_mode_actual_) {
            return ControlModeStatus::Translating;
        }
    } else {
        if (IsModeTransitionState(impl_->sdk_detail_.arm_state)) {
            return ControlModeStatus::Translating;
        }
        if (impl_->enable_mode_ == EnableMode::Enable &&
            impl_->control_mode_target_ != impl_->control_mode_actual_) {
            return ControlModeStatus::Translating;
        }
    }
    switch (impl_->control_mode_actual_) {
        case ControlMode::Position:
            return ControlModeStatus::Position;
        case ControlMode::JointImp:
            return ControlModeStatus::JointImp;
        case ControlMode::CartImp:
            return ControlModeStatus::CartImp;
        case ControlMode::Force:
            return ControlModeStatus::Force;
    }
    return ControlModeStatus::Position;
}

void Robot::_SubmitStream(MotionType type, const CmdPackage& pkg) {
    if (!_CanAcceptCmd()) {
        return;
    }
    if (!impl_->stream_cmd_.has_value()) {
        impl_->stream_cmd_.emplace();
    }
    impl_->stream_cmd_->AssignFrom(pkg);
    impl_->stream_dirty_ = true;

    if (impl_->active_motion_ == type && impl_->motion_inited_) {
        return;
    }
    if (impl_->active_motion_ == type && !impl_->motion_inited_) {
        if (!impl_->active_cmd_.has_value()) {
            impl_->active_cmd_.emplace();
        }
        impl_->active_cmd_->AssignFrom(pkg);
        return;
    }
    const bool can_start =
        impl_->active_motion_ == MotionType::None || _MotionDoneForSwitch();
    if (can_start) {
        if (impl_->active_motion_ != type &&
            impl_->active_motion_ == MotionType::ServoPByPico) {
            impl_->motion_servop_pico_.ResetSession();
        }
        impl_->active_motion_ = type;
        impl_->motion_inited_ = false;
        if (!impl_->active_cmd_.has_value()) {
            impl_->active_cmd_.emplace();
        }
        impl_->active_cmd_->AssignFrom(pkg);
        return;
    }
    if (IsStreamMotion(impl_->active_motion_) && impl_->active_motion_ != type) {
        if (impl_->active_motion_ == MotionType::ServoPByPico) {
            impl_->motion_servop_pico_.ResetSession();
        }
        impl_->active_motion_ = type;
        impl_->motion_inited_ = false;
        if (!impl_->active_cmd_.has_value()) {
            impl_->active_cmd_.emplace();
        }
        impl_->active_cmd_->AssignFrom(pkg);
    }
}

void Robot::_ApplyStreamCmd() {
    if (!impl_->stream_dirty_ || !impl_->stream_cmd_.has_value()) {
        return;
    }
    if (!IsStreamMotion(impl_->active_motion_) || !impl_->motion_inited_) {
        return;
    }
    if (impl_->stream_cmd_->type != impl_->active_motion_) {
        return;
    }
    switch (impl_->active_motion_) {
        case MotionType::ServoJ:
            impl_->motion_servoj_.RePlan(impl_->stream_cmd_->q, impl_->ref_rs_);
            break;
        case MotionType::ServoP:
            impl_->motion_servop_.RePlan(impl_->stream_cmd_->pose, impl_->ref_rs_, impl_->ref_rs_.joint_state.q);
            break;
        case MotionType::ServoPByPico:
            impl_->motion_servop_pico_.RePlan(impl_->stream_cmd_->pose, impl_->ref_rs_,
                                       impl_->ref_rs_.joint_state.q);
            break;
        default:
            break;
    }
    impl_->stream_dirty_ = false;
}

void Robot::_EnterStop() {
    impl_->cmd_queue_.clear();
    impl_->stream_cmd_.reset();
    impl_->stream_dirty_ = false;
    if (impl_->active_motion_ == MotionType::ServoPByPico) {
        impl_->motion_servop_pico_.ResetSession();
    }
    impl_->stop_pending_ = true;
    impl_->active_motion_ = MotionType::Stop;
    impl_->motion_inited_ = false;
    impl_->active_cmd_.reset();
}

void Robot::_EnterStopOnFault() {
    impl_->cmd_queue_.clear();
    impl_->stream_cmd_.reset();
    impl_->stream_dirty_ = false;
    if (impl_->active_motion_ == MotionType::ServoPByPico) {
        impl_->motion_servop_pico_.ResetSession();
    }
    impl_->stop_pending_ = true;
    if (impl_->active_motion_ != MotionType::Stop) {
        impl_->active_motion_ = MotionType::Stop;
        impl_->motion_inited_ = false;
        impl_->active_cmd_.reset();
    }
}

bool Robot::_CanAcceptCmd() const {
    return impl_->error_code_ == ErrorCode::Normal &&
           impl_->enable_state_ == EnableState::Enabled &&
           impl_->status_code_ != StatusCode::Fault &&
           impl_->status_code_ != StatusCode::Stopping;
}

void Robot::_ProcessCmdQueue() {
    if (impl_->stop_pending_) {
        if (impl_->active_motion_ != MotionType::Stop) {
            impl_->active_motion_ = MotionType::Stop;
            impl_->active_cmd_.reset();
            impl_->motion_inited_ = false;
        }
        return;
    }
    if (impl_->cmd_queue_.empty()) {
        return;
    }

    const bool can_start =
        impl_->active_motion_ == MotionType::None || _MotionDoneForSwitch();
    if (!can_start) {
        return;
    }

    const CmdPackage& pkg = impl_->cmd_queue_.front();
    impl_->cmd_queue_.pop_front();
    if (!impl_->active_cmd_.has_value()) {
        impl_->active_cmd_.emplace();
    }
    impl_->active_cmd_->AssignFrom(pkg);
    impl_->motion_inited_ = false;
    impl_->active_motion_ = pkg.type;
}

bool Robot::_MotionDoneForSwitch() {
    if (IsStreamMotion(impl_->active_motion_)) {
        return false;
    }
    switch (impl_->active_motion_) {
        case MotionType::Stop:
            return impl_->motion_stop_.IsDone();
        case MotionType::MovJ:
            return impl_->motion_movj_.IsDone();
        case MotionType::MovL:
            return impl_->motion_movl_.IsDone();
        default:
            return true;
    }
}

void Robot::_RunActiveMotion() {
    if (impl_->active_motion_ == MotionType::None) {
        return;
    }

    if (!impl_->motion_inited_) {
        bool motion_ready = true;
        switch (impl_->active_motion_) {
            case MotionType::Stop:
                impl_->motion_stop_.InitPlan(impl_->ref_rs_);
                break;
            case MotionType::ServoJ: {
                const V7d q =
                    impl_->active_cmd_.has_value() ? impl_->active_cmd_->q : impl_->ref_rs_.joint_state.q;
                impl_->motion_servoj_.InitPlan(q, impl_->ref_rs_);
                break;
            }
            case MotionType::ServoP: {
                const Pose& p = impl_->active_cmd_.has_value() ? impl_->active_cmd_->pose
                                                        : impl_->ref_rs_.cart_state.pose;
                impl_->motion_servop_.InitPlan(p, impl_->ref_rs_, impl_->ref_rs_.joint_state.q);
                break;
            }
            case MotionType::ServoPByPico:
                ++MvDiag::ServoPicoTraceGet().arm[impl_->arm_serial_].motion_init;
                impl_->motion_servop_pico_.InitPlan(impl_->active_cmd_->pose, impl_->ref_rs_,
                                             impl_->ref_rs_.joint_state.q);
                motion_ready = impl_->motion_servop_pico_.IsSessionActive();
                break;
            case MotionType::MovJ:
                impl_->motion_movj_.InitPlan(impl_->active_cmd_->q, impl_->ref_rs_);
                if (!impl_->motion_movj_.TrajValid()) {
                    impl_->error_code_ = ErrorCode::PlanErr;
                }
                break;
            case MotionType::MovL:
                impl_->motion_movl_.InitPlan(impl_->active_cmd_->pose, impl_->ref_rs_);
                break;
            default:
                break;
        }
        impl_->motion_inited_ = motion_ready;
    }

    switch (impl_->active_motion_) {
        case MotionType::Stop:
            impl_->motion_stop_.RunPlan(impl_->ref_rs_);
            break;
        case MotionType::ServoJ:
            impl_->motion_servoj_.RunPlan(impl_->ref_rs_);
            break;
        case MotionType::ServoP:
            impl_->motion_servop_.RunPlan(impl_->ref_rs_);
            break;
        case MotionType::ServoPByPico:
            impl_->motion_servop_pico_.RunPlan(impl_->ref_rs_);
            break;
        case MotionType::MovJ:
            impl_->motion_movj_.RunPlan(impl_->ref_rs_);
            break;
        case MotionType::MovL:
            impl_->motion_movl_.RunPlan(impl_->ref_rs_);
            break;
        default:
            break;
    }

    UpdateCartFromJoint(impl_->arm_serial_, impl_->ref_rs_);

    if (!IsStreamMotion(impl_->active_motion_) && _MotionDoneForSwitch()) {
        impl_->active_motion_ = MotionType::None;
        impl_->active_cmd_.reset();
        impl_->motion_inited_ = false;
        if (impl_->stop_pending_) {
            impl_->stop_pending_ = false;
        }
    }
}

void Robot::_RunActiveMotionIfStopping() {
    if (impl_->active_motion_ != MotionType::Stop) {
        return;
    }
    if (!impl_->motion_inited_) {
        impl_->motion_stop_.InitPlan(impl_->ref_rs_);
        impl_->motion_inited_ = true;
    }
    impl_->motion_stop_.RunPlan(impl_->ref_rs_);
    UpdateCartFromJoint(impl_->arm_serial_, impl_->ref_rs_);
    if (impl_->motion_stop_.IsDone()) {
        impl_->active_motion_ = MotionType::None;
        impl_->motion_inited_ = false;
        impl_->stop_pending_ = false;
    }
}

void Robot::_UpdateStatus() {
    if (impl_->error_code_ != ErrorCode::Normal) {
        impl_->status_code_ = StatusCode::Fault;
        return;
    }
    if (impl_->enable_state_ != EnableState::Enabled) {
        impl_->status_code_ = StatusCode::Disabled;
        return;
    }
    if (impl_->stop_pending_ || impl_->active_motion_ == MotionType::Stop) {
        impl_->status_code_ = StatusCode::Stopping;
        return;
    }
    if (impl_->active_motion_ != MotionType::None) {
        impl_->status_code_ = StatusCode::Running;
        return;
    }
    impl_->status_code_ = StatusCode::Ready;
}

void Robot::_TickTransitionTimeouts() {
    if (impl_->error_code_ != ErrorCode::Normal) {
        return;
    }

    const int limit = impl_->mode_transition_timeout_cycles_;
    const bool enable_switching =
        impl_->enable_state_ == EnableState::Enabling ||
        impl_->enable_state_ == EnableState::Disabling;
    if (enable_switching) {
        if (++impl_->enable_transition_cycles_ >= limit) {
            const ErrorCode prev_err = impl_->error_code_;
            impl_->error_code_ = ErrorCode::EnableError;
            MvDiag::LogEnable(impl_->arm_serial_, "enable TIMEOUT %d/%d → EnableError",
                              impl_->enable_transition_cycles_, limit);
            MvDiag::LogVerbose(impl_->arm_serial_, "enable TIMEOUT sdk=%d sdk_err=%d en=%s",
                               impl_->sdk_detail_.arm_state, impl_->sdk_detail_.arm_err_code,
                               EnableStateName(impl_->enable_state_));
            _LogErrorChange(prev_err, "TickTransitionTimeouts/enable");
            _EnterStopOnFault();
            return;
        }
    } else {
        impl_->enable_transition_cycles_ = 0;
    }

    bool mode_switching = false;
    if (impl_->is_sim_) {
        mode_switching =
            impl_->control_mode_target_ != impl_->control_mode_actual_;
    } else {
        mode_switching = IsModeTransitionState(impl_->sdk_detail_.arm_state);
        if (!mode_switching && impl_->enable_mode_ == EnableMode::Enable &&
            impl_->control_mode_target_ != impl_->control_mode_actual_) {
            mode_switching = true;
        }
    }
    if (mode_switching) {
        if (++impl_->mode_transition_cycles_ >= limit) {
            const ErrorCode prev_err = impl_->error_code_;
            impl_->error_code_ = ErrorCode::ModeError;
            MvDiag::LogEnable(impl_->arm_serial_, "mode TIMEOUT %d/%d → ModeError",
                              impl_->mode_transition_cycles_, limit);
            MvDiag::LogVerbose(impl_->arm_serial_, "mode TIMEOUT sdk=%d imp=%d",
                               impl_->sdk_detail_.arm_state, impl_->sdk_detail_.imp_type);
            _LogErrorChange(prev_err, "TickTransitionTimeouts/mode");
            _EnterStopOnFault();
        }
    } else {
        impl_->mode_transition_cycles_ = 0;
    }
}

void Robot::RunLogic() {
    const EnableState prev_en = impl_->enable_state_;
    _UpdateEnableState();
    _PostEnableDiag(prev_en);
    _TickTransitionTimeouts();

    if (impl_->error_code_ != ErrorCode::Normal) {
        _EnterStopOnFault();
        _RunActiveMotionIfStopping();
        _UpdateStatus();
        return;
    }

    if (impl_->enable_state_ != EnableState::Enabled) {
        _UpdateStatus();
        return;
    }

    if (!_CanAcceptCmd()) {
        _UpdateStatus();
        return;
    }

    _ProcessCmdQueue();
    _ApplyStreamCmd();
    _RunActiveMotion();
    if (impl_->error_code_ != ErrorCode::Normal) {
        _EnterStopOnFault();
        _RunActiveMotionIfStopping();
    }
    _UpdateStatus();
}

bool Robot::Detect() {
    const EnableState prev_en = impl_->enable_state_;
    _UpdateEnableState();
    _PostEnableDiag(prev_en);

    ErrorCode detected = ErrorCode::Normal;

    if (!impl_->is_sim_) {
        if (!impl_->read_buf_ok_) {
            detected = ErrorCode::ConnectError;
        } else if (impl_->sdk_detail_.frame_stale_cycles >= kSdkFrameStaleRunCycles) {
            detected = ErrorCode::ConnectError;
        } else {
            detected = MapSdkToError(impl_->sdk_detail_, ErrorCode::Normal);
            if (ShouldReportSdkModeMismatch(impl_->sdk_detail_.arm_state, impl_->sdk_detail_.imp_type,
                                            impl_->enable_mode_, impl_->control_mode_target_)) {
                detected = PickHigherPriorityError(detected, ErrorCode::ModeError);
            }
        }
    }

    if (detected != ErrorCode::Normal) {
        const ErrorCode prev_err = impl_->error_code_;
        impl_->error_code_ = detected;
        _LogErrorChange(prev_err, "Detect/MapSdkToError");
        _EnterStopOnFault();
        return false;
    }
    // 通信恢复后清除瞬态 ConnectError，避免锁死写关节与运动规划
    if (impl_->error_code_ == ErrorCode::ConnectError && impl_->read_buf_ok_ &&
        impl_->sdk_detail_.frame_stale_cycles == 0) {
        const ErrorCode prev_err = impl_->error_code_;
        impl_->error_code_ = ErrorCode::Normal;
        impl_->stop_pending_ = false;
        if (impl_->active_motion_ == MotionType::Stop) {
            impl_->active_motion_ = MotionType::None;
            impl_->motion_inited_ = false;
            impl_->active_cmd_.reset();
        }
        _LogErrorChange(prev_err, "Detect/ConnectRecovered");
        _UpdateStatus();
        return true;
    }
    if (impl_->error_code_ != ErrorCode::Normal) {
        _EnterStopOnFault();
        MvDiag::LogVerbose(
            impl_->arm_serial_,
            "Detect: SDK 无新故障但 internal error_code=%s 仍保留 (latched)",
            ErrorCodeName(impl_->error_code_));
    }
    return true;
}

void Robot::Stop() {
    if (impl_->enable_state_ != EnableState::Enabled ||
        impl_->error_code_ != ErrorCode::Normal) {
        return;
    }
    _EnterStop();
}

void Robot::ServoJ(const V7d& q) {
    if (!_CanAcceptCmd()) {
        return;
    }
    CmdPackage pkg;
    pkg.type = MotionType::ServoJ;
    pkg.q = q;
    _SubmitStream(MotionType::ServoJ, pkg);
}

void Robot::ServoP(const Pose& pose) {
    if (!_CanAcceptCmd()) {
        return;
    }
    CmdPackage pkg;
    pkg.type = MotionType::ServoP;
    pkg.pose = pose;
    _SubmitStream(MotionType::ServoP, pkg);
}

void Robot::ServoPByPico(const Pose& pose, bool is_run) {
    const int arm = impl_->arm_serial_;
    if (!is_run) {
        impl_->stream_cmd_.reset();
        impl_->stream_dirty_ = false;
        if (impl_->active_motion_ == MotionType::ServoPByPico) {
            impl_->motion_servop_pico_.ResetSession();
            impl_->active_motion_ = MotionType::None;
            impl_->motion_inited_ = false;
            impl_->active_cmd_.reset();
        }
        return;
    }
    ++MvDiag::ServoPicoTraceGet().arm[arm].api_enter;
    if (!_CanAcceptCmd()) {
        ++MvDiag::ServoPicoTraceGet().arm[arm].api_reject;
        return;
    }
    CmdPackage pkg;
    pkg.type = MotionType::ServoPByPico;
    pkg.pose = pose;
    _SubmitStream(MotionType::ServoPByPico, pkg);
    ++MvDiag::ServoPicoTraceGet().arm[arm].stream_submit;
}

void Robot::GoWork() { MovJ(impl_->work_q_); }

void Robot::GoHome() { MovJ(impl_->home_q_); }

void Robot::MovJ(const V7d& q) {
    if (!_CanAcceptCmd()) {
        return;
    }
    CmdPackage pkg;
    pkg.type = MotionType::MovJ;
    pkg.q = q;
    impl_->cmd_queue_.emplace_back();
    impl_->cmd_queue_.back().AssignFrom(pkg);
}

void Robot::MovL(const Pose& pose) {
    if (!_CanAcceptCmd()) {
        return;
    }
    CmdPackage pkg;
    pkg.type = MotionType::MovL;
    pkg.pose = pose;
    impl_->cmd_queue_.emplace_back();
    impl_->cmd_queue_.back().AssignFrom(pkg);
}

const RobotState& Robot::GetRefState() const { return impl_->ref_rs_; }

const RobotState& Robot::GetRespState() const { return impl_->resp_rs_; }

bool Robot::ClearError() {
    _ClearMotionCmds();

    if (impl_->error_code_ == ErrorCode::MotionError ||
        impl_->error_code_ == ErrorCode::PlanErr) {
        impl_->error_code_ = ErrorCode::Normal;
        _UpdateStatus();
        return true;
    }

    // 使能/模式过渡超时为软件侧判定，清错后允许重新下发 Disable/Enable
    if (impl_->error_code_ == ErrorCode::EnableError ||
        impl_->error_code_ == ErrorCode::ModeError) {
        impl_->error_code_ = ErrorCode::Normal;
        impl_->enable_transition_cycles_ = 0;
        impl_->mode_transition_cycles_ = 0;
        impl_->stop_pending_ = false;
        if (impl_->active_motion_ == MotionType::Stop) {
            impl_->active_motion_ = MotionType::None;
            impl_->motion_inited_ = false;
            impl_->active_cmd_.reset();
        }
    }

    if (impl_->is_sim_) {
        impl_->error_code_ = ErrorCode::Normal;
    } else if (IsHardwareRelatedError(impl_->error_code_) ||
               impl_->error_code_ == ErrorCode::InitError) {
        if (impl_->sdk_detail_.frame_stale_cycles >= kSdkFrameStaleRunCycles) {
            return false;
        }
        StateCmdPackage cmd{};
        cmd.type = StateCmdType::ClearError;
        if (impl_->pending_state_queue_.empty() ||
            impl_->pending_state_queue_.front().type != StateCmdType::ClearError) {
            impl_->pending_state_queue_.push_front(cmd);
        }
        _UpdateEnableState();
        _UpdateStatus();
        return true;
    } else {
        impl_->error_code_ = ErrorCode::Normal;
    }

    _UpdateEnableState();
    _UpdateStatus();
    return impl_->error_code_ == ErrorCode::Normal;
}

StatusCode Robot::GetStatusCode() const { return impl_->status_code_; }

ErrorCode Robot::GetErrorCode() const { return impl_->error_code_; }

bool Robot::IsStationary() const { return impl_->low_spd_flag_ == 1; }

bool Robot::_SetRespState(const RobotState& rs) {
    impl_->resp_rs_.joint_state = rs.joint_state;
    if (IkSolver::IsReady() && !UpdateCartFromJoint(impl_->arm_serial_, impl_->resp_rs_)) {
        impl_->error_code_ = ErrorCode::InitError;
        return false;
    }
    if (impl_->work_q_.isZero(1e-9)) {
        impl_->work_q_ = rs.joint_state.q;
    }
    return true;
}

void Robot::_SetRefState(const RobotState& rs) {
    impl_->ref_rs_.joint_state = rs.joint_state;
    UpdateCartFromJoint(impl_->arm_serial_, impl_->ref_rs_);
}
