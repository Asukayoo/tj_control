#include "motion.hpp"

#include "diag.hpp"
#include "ik.hpp"
#include "math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace {

constexpr double kSmall = 1e-6;
constexpr double kLittle = 0.001;
constexpr double kStopTime = 0.2;

CubicCoef CalCubic(double ps, double vs, double pe, double ve, double t) {
    CubicCoef c;
    c.d = ps;
    c.c = vs;
    c.b = (3.0 * (pe - ps) / t - 2.0 * vs - ve) / t;
    c.a = (2.0 * (ps - pe) / t + vs + ve) / (t * t);
    return c;
}

void EvalProfile(const CubicCoef& c, double t, double& s, double& v, double& a) {
    s = c.a * t * t * t + c.b * t * t + c.c * t + c.d;
    v = 3.0 * c.a * t * t + 2.0 * c.b * t + c.c;
    a = 6.0 * c.a * t + 2.0 * c.b;
}

Quat SlerpQuat(const Quat& a, const Quat& b, double t) {
    return a.slerp(std::clamp(t, 0.0, 1.0), b);
}

void WriteJointQ(RobotState& ref_rs, const V7d& q, const V7d& v = V7d::Zero(),
                 const V7d& a = V7d::Zero()) {
    ref_rs.joint_state.q = q;
    ref_rs.joint_state.v = v;
    ref_rs.joint_state.a = a;
    ref_rs.joint_state.j.setZero();
}

void ClampCartVel(V6d& v, const CartLimit& limit) {
    if (v.head<3>().norm() > limit.max_line_v) {
        v.head<3>() = v.head<3>().normalized() * limit.max_line_v;
    }
    if (v.tail<3>().norm() > limit.max_angle_v) {
        v.tail<3>() = v.tail<3>().normalized() * limit.max_angle_v;
    }
}

}  // namespace

const char* RuckigResultName(ruckig::Result r) {
    switch (r) {
        case ruckig::Result::Working:
            return "Working";
        case ruckig::Result::Finished:
            return "Finished";
        case ruckig::Result::Error:
            return "Error";
        case ruckig::Result::ErrorInvalidInput:
            return "InvalidInput";
        case ruckig::Result::ErrorTrajectoryDuration:
            return "TrajDuration";
        case ruckig::Result::ErrorPositionalLimits:
            return "PosLimits";
        case ruckig::Result::ErrorZeroLimits:
            return "ZeroLimits";
        case ruckig::Result::ErrorExecutionTimeCalculation:
            return "ExecTime";
        case ruckig::Result::ErrorSynchronizationCalculation:
            return "Sync";
        default:
            return "?";
    }
}

MotionStop::MotionStop(const JointLimit& limit) : limit_(limit) {}

bool MotionStop::IsDone() const { return done_; }

void MotionStop::SetLimit(const JointLimit& limit) { limit_ = limit; }

void MotionStop::InitPlan(const RobotState& ref_rs) {
    const V7d& v = ref_rs.joint_state.v;
    const V7d& a = ref_rs.joint_state.a;
    speed_norm_ = v.norm();
    if (speed_norm_ <= kSmall) {
        done_ = true;
        init_ = false;
        return;
    }
    axis_t_ = v / speed_norm_;
    acc_norm_ = a.norm();
    int sign = 1;
    for (int i = 0; i < DOF; ++i) {
        if (std::abs(v(i)) > kSmall) {
            sign = (a(i) / v(i) > 0.0) ? 1 : -1;
        }
    }
    acc_norm_ = sign * acc_norm_;
    stopacc_ = std::max(speed_norm_ / kStopTime, std::abs(acc_norm_));
    for (int i = 0; i < DOF; ++i) {
        if (std::abs(axis_t_(i)) > kSmall) {
            stopacc_ = std::min(stopacc_, limit_.max_a(i) / std::abs(axis_t_(i)));
        }
    }
    cycle_count_ = 1;
    done_ = false;
    init_ = true;
}

void MotionStop::RunPlan(RobotState& ref_rs) {
    if (!init_) {
        done_ = true;
        return;
    }
    V7d q = ref_rs.joint_state.q;
    V7d v_out = V7d::Zero();
    if (ref_rs.joint_state.v.norm() > kSmall) {
        const double t0 = cycle_count_ * kControlDt;
        double vel = speed_norm_ + acc_norm_ * t0 - stopacc_ * t0;
        if (vel <= kSmall) {
            vel = 0.0;
        }
        q = ref_rs.joint_state.q + axis_t_ * vel * kControlDt;
        v_out = axis_t_ * vel;
        cycle_count_++;
        if (vel <= kSmall) {
            done_ = true;
            init_ = false;
        }
    } else {
        done_ = true;
        init_ = false;
    }
    WriteJointQ(ref_rs, q, v_out, -axis_t_ * stopacc_);
}

MotionMovJ::MotionMovJ(const JointLimit& limit) : limit_(limit) {}

bool MotionMovJ::IsDone() const { return done_; }

void MotionMovJ::SetLimit(const JointLimit& limit) { limit_ = limit; }

void MotionMovJ::InitPlan(const V7d& target_q, const RobotState& ref_rs) {
    V7d target = target_q;
    for (int i = 0; i < DOF; ++i) {
        while (target(i) - ref_rs.joint_state.q(i) > PI) {
            target(i) -= 2.0 * PI;
        }
        while (target(i) - ref_rs.joint_state.q(i) < -PI) {
            target(i) += 2.0 * PI;
        }
    }

    ruckig::InputParameter<DOF> input;
    input.current_position = {ref_rs.joint_state.q(0), ref_rs.joint_state.q(1),
                              ref_rs.joint_state.q(2), ref_rs.joint_state.q(3),
                              ref_rs.joint_state.q(4), ref_rs.joint_state.q(5),
                              ref_rs.joint_state.q(6)};
    // PositionServo 保持阶段 ref 仅 q 可信；v/a 可能未更新，规划从静止起步
    input.current_velocity.fill(0.0);
    input.current_acceleration.fill(0.0);
    input.target_position = {target(0), target(1), target(2), target(3),
                             target(4), target(5), target(6)};
    input.target_velocity.fill(0.0);
    input.target_acceleration.fill(0.0);
    input.max_velocity = {limit_.max_v(0), limit_.max_v(1), limit_.max_v(2),
                          limit_.max_v(3), limit_.max_v(4), limit_.max_v(5),
                          limit_.max_v(6)};
    input.max_acceleration = {limit_.max_a(0), limit_.max_a(1), limit_.max_a(2),
                              limit_.max_a(3), limit_.max_a(4), limit_.max_a(5),
                              limit_.max_a(6)};
    input.max_jerk = {limit_.max_j(0), limit_.max_j(1), limit_.max_j(2), limit_.max_j(3),
                      limit_.max_j(4), limit_.max_j(5), limit_.max_j(6)};
    input.min_velocity = {-limit_.max_v(0), -limit_.max_v(1), -limit_.max_v(2),
                          -limit_.max_v(3), -limit_.max_v(4), -limit_.max_v(5),
                          -limit_.max_v(6)};
    input.min_acceleration = {-limit_.max_a(0), -limit_.max_a(1), -limit_.max_a(2),
                              -limit_.max_a(3), -limit_.max_a(4), -limit_.max_a(5),
                              -limit_.max_a(6)};

    const ruckig::Result result = ruckig_.calculate(input, trajectory_);
    traj_ok_ = (result == ruckig::Result::Working || result == ruckig::Result::Finished);
    plan_time_ = 0.0;
    done_ = !traj_ok_;
    init_ = traj_ok_;

    double max_dq = 0.0;
    for (int i = 0; i < DOF; ++i) {
        max_dq = std::max(max_dq, std::abs(target(i) - ref_rs.joint_state.q(i)));
    }
    MvDiag::LogMotion(
        "InitPlan ruckig=%s ok=%d dur=%.3fs max_dq=%.4frad(%.2fdeg) q0=%.3f v0=%.3f a0=%.3f",
        RuckigResultName(result), traj_ok_ ? 1 : 0,
        traj_ok_ ? trajectory_.get_duration() : 0.0, max_dq, max_dq * R2D,
        ref_rs.joint_state.q(3) * R2D, ref_rs.joint_state.v(3) * R2D,
        ref_rs.joint_state.a(3) * R2D);
}

void MotionMovJ::RunPlan(RobotState& ref_rs) {
    if (!init_ || !traj_ok_) {
        done_ = true;
        return;
    }
    std::array<double, DOF> pos{};
    std::array<double, DOF> vel{};
    std::array<double, DOF> acc{};
    trajectory_.at_time(plan_time_, pos, vel, acc);

    for (int i = 0; i < DOF; ++i) {
        ref_rs.joint_state.q(i) = pos[i];
        ref_rs.joint_state.v(i) = vel[i];
        ref_rs.joint_state.a(i) = acc[i];
    }
    ref_rs.joint_state.j.setZero();

    plan_time_ += kControlDt;
    if (plan_time_ > trajectory_.get_duration()) {
        done_ = true;
        init_ = false;
    }
}

MotionServoJ::MotionServoJ(const JointLimit& limit) : limit_(limit) {}

void MotionServoJ::SetLimit(const JointLimit& limit) { limit_ = limit; }

void MotionServoJ::SetPdGain(double /*p_gain*/, double /*d_gain*/) {}

void MotionServoJ::InitPlan(const V7d& target_q, const RobotState& ref_rs) {
    const V7d ps = ref_rs.joint_state.q;
    const V7d vs = ref_rs.joint_state.v;
    p_cmd_ = target_q;
    p_target_ = ps;
    v_target_ = vs;
    last_out_ = ps;
    for (int i = 0; i < DOF; ++i) {
        for (int j = 0; j < kFilterSize; ++j) {
            p_flt_[i][j] = ps(i);
        }
    }
    flt_i_ = 0;
    t_ = kStreamServoPeriod;
    t_cur_ = kControlDt;

    for (int i = 0; i < DOF; ++i) {
        const double ve = std::clamp((p_cmd_(i) - ps(i)) / t_, -limit_.max_v(i),
                                     limit_.max_v(i));
        coef_[i] = CalCubic(ps(i), vs(i), p_cmd_(i), ve, t_);
        v_cmd_(i) = ve;
    }
    init_ = true;
}

void MotionServoJ::RePlan(const V7d& target_q, const RobotState& ref_rs) {
    p_cmd_ = target_q;
    if (!init_) {
        InitPlan(target_q, ref_rs);
        return;
    }
    // 段 B 段首 = 段 A 末 p_target_/v_target_，保证 C¹ 连续
    const V7d ps = p_target_;
    const V7d vs = v_target_;
    t_ = kStreamServoPeriod;
    t_cur_ = kControlDt;

    for (int i = 0; i < DOF; ++i) {
        const double ve =
            std::clamp((p_cmd_(i) - ps(i)) / t_, -limit_.max_v(i), limit_.max_v(i));
        coef_[i] = CalCubic(ps(i), vs(i), p_cmd_(i), ve, t_);
        v_cmd_(i) = ve;
    }
    init_ = true;
}

void MotionServoJ::RunPlan(RobotState& ref_rs) {
    if (!init_) {
        return;
    }

    if (t_cur_ <= t_) {
        for (int i = 0; i < DOF; ++i) {
            double s = 0.0;
            double v = 0.0;
            double a = 0.0;
            EvalProfile(coef_[i], t_cur_, s, v, a);
            p_target_(i) = s;
            v_target_(i) = std::clamp(v, -limit_.max_v(i), limit_.max_v(i));
        }
    } else {
        p_target_ = p_cmd_;
        v_target_ = v_cmd_;
    }

    for (int i = 0; i < DOF; ++i) {
        p_flt_[i][flt_i_] = p_target_(i);
    }
    V7d joint = V7d::Zero();
    for (int i = 0; i < DOF; ++i) {
        for (int j = 0; j < kFilterSize; ++j) {
            joint(i) += p_flt_[i][j];
        }
        joint(i) /= kFilterSize;
    }
    flt_i_ = (flt_i_ + 1) < kFilterSize ? (flt_i_ + 1) : 0;
    t_cur_ += kControlDt;

    const V7d v_out = (joint - last_out_) / kControlDt;
    last_out_ = joint;
    WriteJointQ(ref_rs, joint, v_out);
}

MotionMovL::MotionMovL(const CartLimit& limit, int arm_serial)
    : limit_(limit), arm_serial_(arm_serial) {}

void MotionMovL::SetLimit(const CartLimit& limit) { limit_ = limit; }

bool MotionMovL::IsDone() const { return done_; }

void MotionMovL::InitPlan(const Pose& target_pose, const RobotState& ref_rs) {
    target_pose_ = target_pose;
    start_pose_ = ref_rs.cart_state.pose;
    last_q_ = ref_rs.joint_state.q;

    const V3d delta = target_pose_.pos - start_pose_.pos;
    line_dist_ = delta.norm();
    Quat q0 = start_pose_.quat.normalized();
    Quat q1 = target_pose_.quat.normalized();
    if (q0.dot(q1) < 0.0) {
        q1.coeffs() *= -1.0;
    }
    rot_angle_ = q0.angularDistance(q1);
    if (line_dist_ < 1e-6) {
        line_dist_ = 0.0;
    }
    if (rot_angle_ < 1e-9) {
        rot_angle_ = 0.0;
    }
    ruckig::InputParameter<2> input;
    input.current_position = {0.0, 0.0};
    input.current_velocity = {0.0, 0.0};
    input.current_acceleration = {0.0, 0.0};
    input.target_position = {line_dist_, rot_angle_};
    input.target_velocity = {0.0, 0.0};
    input.target_acceleration = {0.0, 0.0};
    input.max_velocity = {limit_.max_line_v, limit_.max_angle_v};
    input.max_acceleration = {limit_.max_line_a, limit_.max_angle_a};
    input.max_jerk = {limit_.max_line_j, limit_.max_angle_j};
    input.min_velocity = {-limit_.max_line_v, -limit_.max_angle_v};
    input.min_acceleration = {-limit_.max_line_a, -limit_.max_angle_a};

    const ruckig::Result result = ruckig_.calculate(input, trajectory_);
    traj_ok_ = (result == ruckig::Result::Working);
    plan_time_ = 0.0;
    done_ = !traj_ok_;
    init_ = traj_ok_;
}

void MotionMovL::RunPlan(RobotState& ref_rs) {
    const V7d& ref_q = ref_rs.joint_state.q;
    if (!init_ || !traj_ok_) {
        done_ = true;
        return;
    }

    std::array<double, 2> s{};
    std::array<double, 2> sv{};
    std::array<double, 2> sa{};
    trajectory_.at_time(plan_time_, s, sv, sa);

    alignas(16) Pose pose;
    if (line_dist_ > 1e-6) {
        const double ratio = s[0] / line_dist_;
        pose.pos = start_pose_.pos + ratio * (target_pose_.pos - start_pose_.pos);
    } else {
        pose.pos = target_pose_.pos;
    }
    if (rot_angle_ > 1e-9) {
        const double t = s[1] / rot_angle_;
        pose.quat = SlerpQuat(start_pose_.quat, target_pose_.quat, t);
    } else {
        pose.quat = target_pose_.quat;
    }

    if (plan_time_ < 0.5 * kControlDt) {
        WriteJointQ(ref_rs, ref_q);
        plan_time_ += kControlDt;
        if (plan_time_ > trajectory_.get_duration()) {
            done_ = true;
            init_ = false;
        }
        return;
    }

    V7d q;
    if (!IkSolver::Solve(arm_serial_, pose, ref_q, q)) {
        done_ = true;
        init_ = false;
        return;
    }
    const V7d v = (q - last_q_) / kControlDt;
    WriteJointQ(ref_rs, q, v);
    last_q_ = q;

    plan_time_ += kControlDt;
    if (plan_time_ > trajectory_.get_duration()) {
        done_ = true;
        init_ = false;
    }
}

MotionServoP::MotionServoP(const CartLimit& limit, int arm_serial)
    : limit_(limit), arm_serial_(arm_serial) {}

void MotionServoP::SetLimit(const CartLimit& limit) { limit_ = limit; }

void MotionServoP::SetPdGain(double /*p_gain*/, double /*d_gain*/) {}

void MotionServoP::ResetFilter(const Pose& seed) {
    for (int j = 0; j < kFilterSize; ++j) {
        p_flt_[j] = seed;
    }
    flt_i_ = 0;
}

Pose MotionServoP::FilterPose() const {
    V3d pos = V3d::Zero();
    V4d q_acc = V4d::Zero();
    for (int j = 0; j < kFilterSize; ++j) {
        pos += p_flt_[j].pos;
        V4d qc = p_flt_[j].quat.normalized().coeffs();
        if (q_acc.squaredNorm() > 1e-12 && q_acc.dot(qc) < 0.0) {
            qc *= -1.0;
        }
        q_acc += qc;
    }
    pos /= static_cast<double>(kFilterSize);
    alignas(16) Pose out;
    out.pos = pos;
    if (q_acc.squaredNorm() > 1e-12) {
        out.quat = Quat(q_acc.normalized());
    } else {
        out.quat = p_flt_[0].quat;
    }
    return out;
}

void MotionServoP::Reset() {
    init_ = false;
    last_q_.setZero();
}

void MotionServoP::InitPlan(const Pose& target_pose, const RobotState& ref_rs,
                            const V7d& ref_q) {
    ref_q_ = ref_q;
    if (!IkSolver::Forward(arm_serial_, ref_rs.joint_state.q, p_cmd_last_)) {
        MvDiag::EmitMotionError(arm_serial_, "ServoP", "init_FK_fail");
        init_ = false;
        return;
    }
    last_q_ = ref_q;
    p_cmd_ = target_pose;
    p_target_ = p_cmd_last_;
    v_target_ = ref_rs.cart_state.vel;
    ResetFilter(p_target_);
    t_ = kStreamServoPeriod;
    t_cur_ = kControlDt;

    V6d vs = v_target_;
    ClampCartVel(vs, limit_);
    V6d ve = V6d::Zero();
    ve.head<3>() = (p_cmd_.pos - p_cmd_last_.pos) / t_;
    ve.tail<3>() = QuatLogLocal(p_cmd_last_.quat, p_cmd_.quat) / t_;
    ClampCartVel(ve, limit_);
    V6d err_vec = V6d::Zero();
    err_vec.head<3>() = p_cmd_.pos - p_cmd_last_.pos;
    err_vec.tail<3>() = QuatLogLocal(p_cmd_last_.quat, p_cmd_.quat);
    for (int i = 0; i < 6; ++i) {
        coef_[i] = CalCubic(0.0, vs(i), err_vec(i), ve(i), t_);
    }
    v_cmd_ = ve;
    init_ = true;
}

void MotionServoP::RePlan(const Pose& target_pose, const RobotState& ref_rs,
                          const V7d& ref_q) {
    ref_q_ = ref_q;
    p_cmd_ = target_pose;
    if (!init_) {
        InitPlan(target_pose, ref_rs, ref_q);
        return;
    }
    // 段 B 段首 = 段 A 末位姿/速度
    p_cmd_last_ = p_target_;
    t_ = kStreamServoPeriod;
    t_cur_ = kControlDt;

    V6d vs = v_target_;
    V6d ve = V6d::Zero();
    ve.head<3>() = (p_cmd_.pos - p_cmd_last_.pos) / t_;
    ve.tail<3>() = QuatLogLocal(p_cmd_last_.quat, p_cmd_.quat) / t_;
    ClampCartVel(vs, limit_);
    ClampCartVel(ve, limit_);
    V6d err_vec = V6d::Zero();
    err_vec.head<3>() = p_cmd_.pos - p_cmd_last_.pos;
    err_vec.tail<3>() = QuatLogLocal(p_cmd_last_.quat, p_cmd_.quat);
    for (int i = 0; i < 6; ++i) {
        coef_[i] = CalCubic(0.0, vs(i), err_vec(i), ve(i), t_);
    }
    v_cmd_ = ve;
    init_ = true;
}

void MotionServoP::RunPlan(RobotState& ref_rs) {
    MvDiag::ServoPicoArmTrace& trace = MvDiag::ServoPicoTraceGet().arm[arm_serial_];
    if (!init_) {
        ++trace.servo_skip_init;
        return;
    }
    ++trace.servo_run;

    if (t_cur_ <= t_) {
        V6d prof_v = V6d::Zero();
        V3d pos_vec = V3d::Zero();
        V3d ori_vec = V3d::Zero();
        for (int i = 0; i < 6; ++i) {
            double s = 0.0;
            double v = 0.0;
            double a = 0.0;
            EvalProfile(coef_[i], t_cur_, s, v, a);
            prof_v(i) = v;
            if (i < 3) {
                pos_vec(i) = s;
            } else {
                ori_vec(i - 3) = s;
            }
        }
        p_target_.pos = p_cmd_last_.pos + pos_vec;
        p_target_.quat = (p_cmd_last_.quat * QuatExp(ori_vec)).normalized();
        v_target_ = prof_v;
        ClampCartVel(v_target_, limit_);
    } else {
        p_target_ = p_cmd_;
        v_target_ = v_cmd_;
    }

    p_flt_[flt_i_] = p_target_;
    flt_i_ = (flt_i_ + 1) < kFilterSize ? (flt_i_ + 1) : 0;
    t_cur_ += kControlDt;

    alignas(16) Pose ik_pose = FilterPose();

    V7d q;
    if (!IkSolver::Solve(arm_serial_, ik_pose, ref_q_, q)) {
        ++trace.ik_fail;
        init_ = false;
        return;
    }
    ++trace.ik_ok;
    const V7d v = (q - last_q_) / kControlDt;
    WriteJointQ(ref_rs, q, v);
    last_q_ = q;
    ref_q_ = q;
}

MotionServoPByPico::MotionServoPByPico(const CartLimit& limit, int arm_serial)
    : servo_(limit, arm_serial), arm_serial_(arm_serial) {}

void MotionServoPByPico::SetLimit(const CartLimit& limit) { servo_.SetLimit(limit); }

void MotionServoPByPico::SetPdGain(double p_gain, double d_gain) {
    servo_.SetPdGain(p_gain, d_gain);
}

void MotionServoPByPico::ResetSession() {
    session_active_ = false;
    servo_.Reset();
    ref_pico_.pos.setZero();
    ref_pico_.quat = Quat::Identity();
    robot_anchor_.pos.setZero();
    robot_anchor_.quat = Quat::Identity();
}

void MotionServoPByPico::PicoToAbsTarget(const Pose& pico_pose, Pose& out) const {
    ::PicoToAbsTarget(ref_pico_, robot_anchor_, pico_pose, out);
}

void MotionServoPByPico::InitPlan(const Pose& pico_pose, const RobotState& ref_rs,
                                  const V7d& ref_q) {
    ref_pico_ = pico_pose;
    if (!IkSolver::Forward(arm_serial_, ref_rs.joint_state.q, robot_anchor_)) {
        ++MvDiag::ServoPicoTraceGet().arm[arm_serial_].session_fk_fail;
        MvDiag::EmitMotionError(arm_serial_, "ServoPByPico", "anchor_FK_fail");
        ResetSession();
        return;
    }
    session_active_ = true;
    ++MvDiag::ServoPicoTraceGet().arm[arm_serial_].session_init;
    alignas(16) Pose abs_target;
    PicoToAbsTarget(pico_pose, abs_target);
    servo_.InitPlan(abs_target, ref_rs, ref_q);
}

void MotionServoPByPico::RePlan(const Pose& pico_pose, const RobotState& ref_rs,
                                const V7d& ref_q) {
    ++MvDiag::ServoPicoTraceGet().arm[arm_serial_].replan;
    alignas(16) Pose abs_target;
    PicoToAbsTarget(pico_pose, abs_target);
    servo_.RePlan(abs_target, ref_rs, ref_q);
}

void MotionServoPByPico::RunPlan(RobotState& ref_rs) {
    MvDiag::ServoPicoArmTrace& trace = MvDiag::ServoPicoTraceGet().arm[arm_serial_];
    if (!session_active_) {
        ++trace.run_skip_session;
        return;
    }
    ++trace.run_session;
    servo_.RunPlan(ref_rs);
}
