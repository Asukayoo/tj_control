#include "mv_control.hpp"

#include "internal/ik.hpp"

namespace {

constexpr int kLeftArmIdx = 0;
constexpr int kRightArmIdx = 1;

void V7dToArray(const V7d& q, double joints[DOF]) {
    for (int i = 0; i < DOF; ++i) {
        joints[i] = q(i);
    }
}

void ApplyDiffToRef(RobotState& ref, const JointState& js) {
    const V7d dq = js.q - ref.joint_state.q;
    const V7d dv = js.v - ref.joint_state.v;
    const V7d da = js.a - ref.joint_state.a;
    ref.joint_state.j = (da) / kControlDt;
    ref.joint_state.a = dv / kControlDt;
    ref.joint_state.v = dq / kControlDt;
    ref.joint_state.q = js.q;
}

void UpdateRefCart(int arm_serial, RobotState& ref) {
    // 运动学未初始化时跳过，避免 SDK 未定义行为破坏堆
    if (!IkSolver::IsReady()) {
        return;
    }
    if (!IkSolver::Forward(arm_serial, ref.joint_state.q, ref.cart_state.pose)) {
        return;
    }
    if (IkSolver::Jacobian(arm_serial, ref.joint_state.q, ref.cart_state.jacob)) {
        ref.cart_state.vel = ref.cart_state.jacob * ref.joint_state.v;
    }
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

}  // namespace

Robot::Robot(int arm_serial)
    : arm_serial_(arm_serial),
      motion_stop_(DefaultJointLimit()),
      motion_movj_(DefaultJointLimit()),
      motion_movl_(DefaultCartLimit(), arm_serial),
      motion_servoj_(DefaultJointLimit()),
      motion_servop_(DefaultCartLimit(), arm_serial) {}

Robot::~Robot() = default;

bool Robot::_Init() {
    status_code_ = StatusCode::Ready;
    error_code_ = ErrorCode::Normal;
#ifdef MV_CONTROL_SIM
    ref_rs_.joint_state.q = V7d::Zero();
    ref_rs_.joint_state.v = V7d::Zero();
    ref_rs_.joint_state.a = V7d::Zero();
    ref_rs_.joint_state.j = V7d::Zero();
    ref_rs_.joint_state.tau = V7d::Zero();
    ref_rs_.cart_state.vel = V6d::Zero();
    ref_rs_.cart_state.jacob = Jacob::Zero();
    if (IkSolver::IsReady()) {
        if (!IkSolver::Forward(arm_serial_, ref_rs_.joint_state.q, ref_rs_.cart_state.pose)) {
            error_code_ = ErrorCode::InitError;
            return false;
        }
    }
    resp_rs_ = ref_rs_;
#endif
    return true;
}

void Robot::_ApplyArmConfig(const ArmConfig& cfg) {
    arm_serial_ = cfg.arm_serial;
    home_q_ = cfg.home_q;
    work_q_ = cfg.work_q;
    joint_limit_ = cfg.joint_limit;
    cart_limit_ = cfg.cart_limit;
    motion_movj_.SetLimit(joint_limit_);
    motion_movl_.SetLimit(cart_limit_);
    motion_servoj_.SetLimit(joint_limit_);
    motion_servop_.SetLimit(cart_limit_);
    motion_stop_.SetLimit(joint_limit_);
}

void Robot::_PushCmd(CmdPackage pkg) {
    cmd_queue_.push_back(pkg);
}

bool Robot::_IsServoCmd(CmdType type) const {
    return type == CmdType::ServoJ || type == CmdType::ServoP;
}

bool Robot::_IsServoMotion(MotionKind kind) const {
    return kind == MotionKind::ServoJ || kind == MotionKind::ServoP;
}

void Robot::_ProcessCmdQueue() {
    if (stop_pending_) {
        if (active_motion_ != MotionKind::Stop) {
            active_motion_ = MotionKind::Stop;
            active_cmd_.reset();
            motion_inited_ = false;
        }
        return;
    }
    if (cmd_queue_.empty()) {
        return;
    }

    const CmdPackage& front = cmd_queue_.front();
    const bool servo_replan =
        _IsServoCmd(front.type) && _IsServoMotion(active_motion_) && motion_inited_;

    if (servo_replan) {
        bool motion_done = true;
        switch (active_motion_) {
            case MotionKind::ServoJ:
                motion_done = motion_servoj_.IsDone();
                break;
            case MotionKind::ServoP:
                motion_done = motion_servop_.IsDone();
                break;
            default:
                break;
        }
        if (!motion_done) {
            CmdPackage pkg = cmd_queue_.front();
            cmd_queue_.pop_front();
            active_cmd_ = pkg;
            if (pkg.type == CmdType::ServoJ) {
                motion_servoj_.RePlan(pkg.q, resp_rs_);
            } else {
                motion_servop_.RePlan(pkg.pose, resp_rs_, work_q_);
            }
            motion_inited_ = true;
            return;
        }
    }

    const bool can_start = active_motion_ == MotionKind::None ||
                           _MotionDoneForSwitch();
    if (!can_start) {
        return;
    }

    CmdPackage pkg = cmd_queue_.front();
    cmd_queue_.pop_front();
    active_cmd_ = pkg;
    motion_inited_ = false;

    switch (pkg.type) {
        case CmdType::Stop:
            active_motion_ = MotionKind::Stop;
            break;
        case CmdType::ServoJ:
            active_motion_ = MotionKind::ServoJ;
            break;
        case CmdType::ServoP:
            active_motion_ = MotionKind::ServoP;
            break;
        case CmdType::MovJ:
        case CmdType::GoWork:
        case CmdType::GoHome:
            active_motion_ = MotionKind::MovJ;
            break;
        case CmdType::MovL:
            active_motion_ = MotionKind::MovL;
            break;
    }
}

bool Robot::_MotionDoneForSwitch() {
    switch (active_motion_) {
        case MotionKind::Stop:
            return motion_stop_.IsDone();
        case MotionKind::ServoJ:
            return motion_servoj_.IsDone();
        case MotionKind::ServoP:
            return motion_servop_.IsDone();
        case MotionKind::MovJ:
            return motion_movj_.IsDone();
        case MotionKind::MovL:
            return motion_movl_.IsDone();
        default:
            return true;
    }
}

void Robot::_RunActiveMotion() {
    if (active_motion_ == MotionKind::None) {
        return;
    }

    if (!motion_inited_) {
        switch (active_motion_) {
            case MotionKind::Stop:
                motion_stop_.InitPlan(resp_rs_);
                break;
            case MotionKind::ServoJ: {
                const V7d q = active_cmd_.has_value() ? active_cmd_->q : resp_rs_.joint_state.q;
                motion_servoj_.InitPlan(q, resp_rs_);
                break;
            }
            case MotionKind::ServoP: {
                const Pose p = active_cmd_.has_value() ? active_cmd_->pose : resp_rs_.cart_state.pose;
                motion_servop_.InitPlan(p, resp_rs_, work_q_);
                break;
            }
            case MotionKind::MovJ: {
                V7d target = active_cmd_->q;
                if (active_cmd_->type == CmdType::GoWork) {
                    target = work_q_;
                } else if (active_cmd_->type == CmdType::GoHome) {
                    target = home_q_;
                }
                motion_movj_.InitPlan(target, resp_rs_);
                break;
            }
            case MotionKind::MovL:
                motion_movl_.InitPlan(active_cmd_->pose, resp_rs_, work_q_);
                break;
            default:
                break;
        }
        motion_inited_ = true;
    }

    switch (active_motion_) {
        case MotionKind::Stop:
            motion_stop_.RunPlan(resp_rs_, predeal_queue_);
            break;
        case MotionKind::ServoJ:
            motion_servoj_.RunPlan(predeal_queue_);
            break;
        case MotionKind::ServoP:
            motion_servop_.RunPlan(predeal_queue_);
            break;
        case MotionKind::MovJ:
            motion_movj_.RunPlan(predeal_queue_);
            break;
        case MotionKind::MovL:
            motion_movl_.RunPlan(predeal_queue_);
            break;
        default:
            break;
    }

    if (_MotionDoneForSwitch()) {
        active_motion_ = MotionKind::None;
        active_cmd_.reset();
        motion_inited_ = false;
        if (stop_pending_) {
            stop_pending_ = false;
        }
    }
}

void Robot::_ApplyPredeal() {
    if (predeal_queue_.empty()) {
        return;
    }
    const JointState js = predeal_queue_.front();
    predeal_queue_.pop_front();
    ApplyDiffToRef(ref_rs_, js);
    UpdateRefCart(arm_serial_, ref_rs_);
}

void Robot::_UpdateStatus() {
    if (stop_pending_ || active_motion_ == MotionKind::Stop) {
        status_code_ = StatusCode::Stop;
        return;
    }
    if (active_motion_ != MotionKind::None) {
        status_code_ = StatusCode::Running;
        return;
    }
    status_code_ = StatusCode::Ready;
}

void Robot::_Run() {
    _ProcessCmdQueue();
    _RunActiveMotion();
    _ApplyPredeal();
    _UpdateStatus();
}

bool Robot::_Detect() { return true; }

void Robot::Stop() {
    cmd_queue_.clear();
    stop_pending_ = true;
    active_motion_ = MotionKind::Stop;
    motion_inited_ = false;
    active_cmd_.reset();
    status_code_ = StatusCode::Stop;
}

void Robot::ServoJ(const V7d& q) {
    CmdPackage pkg;
    pkg.type = CmdType::ServoJ;
    pkg.q = q;
    _PushCmd(pkg);
}

void Robot::ServoP(const Pose& pose) {
    CmdPackage pkg;
    pkg.type = CmdType::ServoP;
    pkg.pose = pose;
    _PushCmd(pkg);
}

void Robot::GoWork() {
    CmdPackage pkg;
    pkg.type = CmdType::GoWork;
    pkg.q = work_q_;
    _PushCmd(pkg);
}

void Robot::GoHome() {
    CmdPackage pkg;
    pkg.type = CmdType::GoHome;
    pkg.q = home_q_;
    _PushCmd(pkg);
}

void Robot::MovJ(const V7d& q) {
    CmdPackage pkg;
    pkg.type = CmdType::MovJ;
    pkg.q = q;
    _PushCmd(pkg);
}

void Robot::MovL(const Pose& pose) {
    CmdPackage pkg;
    pkg.type = CmdType::MovL;
    pkg.pose = pose;
    _PushCmd(pkg);
}

void Robot::SetEnableMode(EnableMode enable_mode) { enable_mode_ = enable_mode; }

void Robot::SetControlMode(ControlMode control_mode) { control_mode_ = control_mode; }

RobotState Robot::GetRefState() const { return ref_rs_; }

RobotState Robot::GetRespState() const { return resp_rs_; }

void Robot::ClearError() { error_code_ = ErrorCode::Normal; }

StatusCode Robot::GetStatusCode() const { return status_code_; }

ErrorCode Robot::GetErrorCode() const { return error_code_; }

void Robot::_SetRefState(const RobotState& rs) { ref_rs_ = rs; }

void Robot::_SetRespState(const RobotState& rs) {
    resp_rs_ = rs;
    if (work_q_.isZero(1e-9)) {
        work_q_ = rs.joint_state.q;
    }
}
