#include "motion.hpp"

#include "ik.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace {

constexpr double kSmall = 1e-6;
constexpr double kLittle = 0.001;
constexpr double kDoneEps = 1e-4;
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

V3d QuatLogLocal(const Quat& q0, const Quat& q1) {
    Quat d = q0.conjugate() * q1;
    d.normalize();
    if (d.w() < 0.0) {
        d.coeffs() *= -1.0;
    }
    if (d.w() >= 1.0) {
        return V3d::Zero();
    }
    const double angle = 2.0 * std::acos(std::clamp(d.w(), -1.0, 1.0));
    const double s = std::sqrt(std::max(0.0, 1.0 - d.w() * d.w()));
    if (s < 1e-9) {
        return V3d::Zero();
    }
    return (angle / s) * d.vec();
}

Quat QuatExp(const V3d& w) {
    const double angle = w.norm();
    if (angle < 1e-9) {
        return Quat::Identity();
    }
    return Quat(Eigen::AngleAxisd(angle, w / angle));
}

void PushJointQ(std::deque<JointState>& predeal, const V7d& q) {
    JointState js;
    js.q = q;
    js.v.setZero();
    js.a.setZero();
    js.j.setZero();
    js.tau.setZero();
    predeal.push_back(js);
}

void V7dToArray(const V7d& v, std::array<double, DOF>& arr) {
    for (int i = 0; i < DOF; ++i) {
        arr[i] = v(i);
    }
}

}  // namespace

MotionStop::MotionStop(const JointLimit& limit) : limit_(limit) {}

bool MotionStop::IsDone() const { return done_; }

void MotionStop::SetLimit(const JointLimit& limit) { limit_ = limit; }

void MotionStop::InitPlan(const RobotState& resp_rs) {
    const V7d& v = resp_rs.joint_state.v;
    const V7d& a = resp_rs.joint_state.a;
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

void MotionStop::RunPlan(const RobotState& resp_rs, std::deque<JointState>& predeal) {
    if (!init_) {
        done_ = true;
        return;
    }
    V7d q = resp_rs.joint_state.q;
    if (resp_rs.joint_state.v.norm() > kSmall) {
        const double t0 = cycle_count_ * kControlDt;
        double vel = speed_norm_ + acc_norm_ * t0 - stopacc_ * t0;
        if (vel <= kSmall) {
            vel = 0.0;
        }
        q = resp_rs.joint_state.q + axis_t_ * vel * kControlDt;
        cycle_count_++;
        if (vel <= kSmall) {
            done_ = true;
            init_ = false;
        }
    } else {
        done_ = true;
        init_ = false;
    }
    PushJointQ(predeal, q);
}

MotionMovJ::MotionMovJ(const JointLimit& limit) : limit_(limit) {}

bool MotionMovJ::IsDone() const { return done_; }

void MotionMovJ::SetLimit(const JointLimit& limit) { limit_ = limit; }

void MotionMovJ::InitPlan(const V7d& target_q, const RobotState& resp_rs) {
    ruckig::InputParameter<DOF> input;
    input.current_position = {resp_rs.joint_state.q(0), resp_rs.joint_state.q(1),
                              resp_rs.joint_state.q(2), resp_rs.joint_state.q(3),
                              resp_rs.joint_state.q(4), resp_rs.joint_state.q(5),
                              resp_rs.joint_state.q(6)};
    input.current_velocity = {resp_rs.joint_state.v(0), resp_rs.joint_state.v(1),
                              resp_rs.joint_state.v(2), resp_rs.joint_state.v(3),
                              resp_rs.joint_state.v(4), resp_rs.joint_state.v(5),
                              resp_rs.joint_state.v(6)};
    input.current_acceleration = {resp_rs.joint_state.a(0), resp_rs.joint_state.a(1),
                                  resp_rs.joint_state.a(2), resp_rs.joint_state.a(3),
                                  resp_rs.joint_state.a(4), resp_rs.joint_state.a(5),
                                  resp_rs.joint_state.a(6)};
    input.target_position = {target_q(0), target_q(1), target_q(2), target_q(3),
                             target_q(4), target_q(5), target_q(6)};
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

    traj_ok_ = (ruckig_.calculate(input, trajectory_) == ruckig::Result::Working);
    plan_time_ = 0.0;
    done_ = !traj_ok_;
    init_ = traj_ok_;
}

void MotionMovJ::RunPlan(std::deque<JointState>& predeal) {
    if (!init_ || !traj_ok_) {
        done_ = true;
        return;
    }
    std::array<double, DOF> pos{};
    std::array<double, DOF> vel{};
    std::array<double, DOF> acc{};
    trajectory_.at_time(plan_time_, pos, vel, acc);

    JointState js;
    for (int i = 0; i < DOF; ++i) {
        js.q(i) = pos[i];
        js.v(i) = vel[i];
        js.a(i) = acc[i];
    }
    js.j.setZero();
    js.tau.setZero();
    predeal.push_back(js);

    plan_time_ += kControlDt;
    if (plan_time_ > trajectory_.get_duration()) {
        done_ = true;
        init_ = false;
    }
}

MotionServoJ::MotionServoJ(const JointLimit& limit) : limit_(limit) {}

bool MotionServoJ::IsDone() const { return done_; }

void MotionServoJ::SetLimit(const JointLimit& limit) { limit_ = limit; }

void MotionServoJ::InitPlan(const V7d& target_q, const RobotState& resp_rs) {
    p_cmd_ = resp_rs.joint_state.q;
    p_current_ = p_cmd_;
    v_current_.setZero();
    v_cmd_.setZero();
    p_target_ = p_current_;
    v_target_.setZero();
    for (int i = 0; i < DOF; ++i) {
        for (int j = 0; j < kFilterSize; ++j) {
            p_flt_[i][j] = p_current_(i);
        }
    }
    flt_i_ = 0;
    t_cur_ = 0.0;

    const double dis = (p_target_ - target_q).norm();
    if (dis > kSmall) {
        for (int i = 0; i < DOF; ++i) {
            const double ve = std::clamp((target_q(i) - p_target_(i)) / t_, -limit_.max_v(i),
                                         limit_.max_v(i));
            coef_[i] = CalCubic(p_target_(i), v_target_(i), target_q(i), ve, t_);
            v_cmd_(i) = ve;
        }
        p_cmd_ = target_q;
        t_cur_ = kControlDt;
    } else {
        t_cur_ = t_ + 0.02;
    }
    init_ = true;
    done_ = false;
}

void MotionServoJ::RePlan(const V7d& target_q, const RobotState& resp_rs) {
    p_current_ = resp_rs.joint_state.q;
    v_current_.setZero();
    for (int i = 0; i < DOF; ++i) {
        for (int j = 0; j < kFilterSize; ++j) {
            p_flt_[i][j] = p_current_(i);
        }
    }
    flt_i_ = 0;
    p_cmd_ = target_q;

    const double dis = (p_cmd_ - p_current_).norm();
    if (dis > kSmall) {
        for (int i = 0; i < DOF; ++i) {
            const double ve =
                std::clamp((p_cmd_(i) - p_current_(i)) / t_, -limit_.max_v(i), limit_.max_v(i));
            coef_[i] = CalCubic(p_current_(i), 0.0, p_cmd_(i), ve, t_);
            v_cmd_(i) = ve;
        }
        t_cur_ = kControlDt;
    }
    init_ = true;
    done_ = false;
}

void MotionServoJ::RunPlan(std::deque<JointState>& predeal) {
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
    } else if (t_cur_ > t_ && t_cur_ < t_ + 0.01) {
        p_target_ = p_cmd_;
        v_target_ = v_cmd_;
    } else {
        p_target_ = p_cmd_;
        v_target_.setZero();
    }

    const V7d err = p_target_ - p_current_;
    const V7d errd = v_target_ - v_current_;
    const V7d u = p_gain_ * err + d_gain_ * errd;
    for (int i = 0; i < DOF; ++i) {
        v_current_(i) += u(i) * kControlDt;
        v_current_(i) = std::clamp(v_current_(i), -limit_.max_v(i), limit_.max_v(i));
        p_current_(i) += v_current_(i) * kControlDt;
    }

    for (int i = 0; i < DOF; ++i) {
        p_flt_[i][flt_i_] = p_current_(i);
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

    if ((p_cmd_ - joint).norm() < kDoneEps) {
        joint = p_cmd_;
        done_ = true;
    }
    PushJointQ(predeal, joint);
}

MotionMovL::MotionMovL(const CartLimit& limit, int arm_serial)
    : limit_(limit), arm_serial_(arm_serial) {}

void MotionMovL::SetLimit(const CartLimit& limit) { limit_ = limit; }

bool MotionMovL::IsDone() const { return done_; }

void MotionMovL::InitPlan(const Pose& target_pose, const RobotState& resp_rs,
                          const V7d& ref_q) {
    ref_q_ = ref_q;
    target_pose_ = target_pose;
    if (!IkSolver::Forward(arm_serial_, resp_rs.joint_state.q, start_pose_)) {
        done_ = true;
        init_ = false;
        traj_ok_ = false;
        return;
    }

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

    traj_ok_ = (ruckig_.calculate(input, trajectory_) == ruckig::Result::Working);
    plan_time_ = 0.0;
    done_ = !traj_ok_;
    init_ = traj_ok_;
}

void MotionMovL::RunPlan(std::deque<JointState>& predeal) {
    if (!init_ || !traj_ok_) {
        done_ = true;
        return;
    }

    std::array<double, 2> s{};
    std::array<double, 2> sv{};
    std::array<double, 2> sa{};
    trajectory_.at_time(plan_time_, s, sv, sa);

    Pose pose;
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

    V7d q;
    if (!IkSolver::Solve(arm_serial_, pose, ref_q_, q)) {
        done_ = true;
        init_ = false;
        return;
    }
    ref_q_ = q;
    PushJointQ(predeal, q);

    plan_time_ += kControlDt;
    if (plan_time_ > trajectory_.get_duration()) {
        done_ = true;
        init_ = false;
    }
}

MotionServoP::MotionServoP(const CartLimit& limit, int arm_serial)
    : limit_(limit), arm_serial_(arm_serial) {}

bool MotionServoP::IsDone() const { return done_; }

void MotionServoP::SetLimit(const CartLimit& limit) { limit_ = limit; }

void MotionServoP::InitPlan(const Pose& target_pose, const RobotState& resp_rs,
                            const V7d& ref_q) {
    ref_q_ = ref_q;
    if (!IkSolver::Forward(arm_serial_, resp_rs.joint_state.q, p_current_)) {
        done_ = true;
        init_ = false;
        return;
    }
    p_cmd_ = p_current_;
    v_current_.setZero();
    v_cmd_.setZero();
    p_target_ = p_current_;
    v_target_.setZero();
    for (int j = 0; j < kFilterSize; ++j) {
        p_flt_[j] = p_current_;
    }
    flt_i_ = 0;
    t_cur_ = 0.0;

    const double pos_dis = (target_pose.pos - p_target_.pos).norm();
    const double ori_dis = QuatLogLocal(p_target_.quat, target_pose.quat).norm();
    if (pos_dis > kSmall || ori_dis > kLittle) {
        V6d ve = V6d::Zero();
        ve.head<3>() = (target_pose.pos - p_target_.pos) / t_;
        ve.tail<3>() = QuatLogLocal(p_target_.quat, target_pose.quat) / t_;
        if (ve.head<3>().norm() > limit_.max_line_v) {
            ve.head<3>() = ve.head<3>().normalized() * limit_.max_line_v;
        }
        if (ve.tail<3>().norm() > limit_.max_angle_v) {
            ve.tail<3>() = ve.tail<3>().normalized() * limit_.max_angle_v;
        }
        V6d err_vec = V6d::Zero();
        err_vec.head<3>() = target_pose.pos - p_target_.pos;
        err_vec.tail<3>() = QuatLogLocal(p_target_.quat, target_pose.quat);
        for (int i = 0; i < 6; ++i) {
            coef_[i] = CalCubic(0.0, v_target_(i), err_vec(i), ve(i), t_);
        }
        p_cmd_last_ = p_target_;
        p_cmd_ = target_pose;
        v_cmd_ = ve;
        t_cur_ = kControlDt;
    } else {
        t_cur_ = t_ + 0.02;
    }
    init_ = true;
    done_ = false;
}

void MotionServoP::RePlan(const Pose& target_pose, const RobotState& resp_rs,
                          const V7d& ref_q) {
    ref_q_ = ref_q;
    if (!IkSolver::Forward(arm_serial_, resp_rs.joint_state.q, p_current_)) {
        done_ = true;
        init_ = false;
        return;
    }
    v_current_.setZero();
    for (int j = 0; j < kFilterSize; ++j) {
        p_flt_[j] = p_current_;
    }
    flt_i_ = 0;
    p_cmd_ = target_pose;

    const double pos_dis = (p_cmd_.pos - p_current_.pos).norm();
    const double ori_dis = QuatLogLocal(p_current_.quat, p_cmd_.quat).norm();
    if (pos_dis > kSmall || ori_dis > kLittle) {
        V6d ve = V6d::Zero();
        ve.head<3>() = (p_cmd_.pos - p_current_.pos) / t_;
        ve.tail<3>() = QuatLogLocal(p_current_.quat, p_cmd_.quat) / t_;
        if (ve.head<3>().norm() > limit_.max_line_v) {
            ve.head<3>() = ve.head<3>().normalized() * limit_.max_line_v;
        }
        if (ve.tail<3>().norm() > limit_.max_angle_v) {
            ve.tail<3>() = ve.tail<3>().normalized() * limit_.max_angle_v;
        }
        V6d err_vec = V6d::Zero();
        err_vec.head<3>() = p_cmd_.pos - p_current_.pos;
        err_vec.tail<3>() = QuatLogLocal(p_current_.quat, p_cmd_.quat);
        for (int i = 0; i < 6; ++i) {
            coef_[i] = CalCubic(0.0, 0.0, err_vec(i), ve(i), t_);
        }
        p_cmd_last_ = p_current_;
        v_cmd_ = ve;
        t_cur_ = kControlDt;
    }
    init_ = true;
    done_ = false;
}

void MotionServoP::RunPlan(std::deque<JointState>& predeal) {
    if (!init_) {
        return;
    }

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
        if (v_target_.head<3>().norm() > limit_.max_line_v) {
            v_target_.head<3>() = v_target_.head<3>().normalized() * limit_.max_line_v;
        }
        if (v_target_.tail<3>().norm() > limit_.max_angle_v) {
            v_target_.tail<3>() = v_target_.tail<3>().normalized() * limit_.max_angle_v;
        }
    } else if (t_cur_ > t_ && t_cur_ < t_ + 0.01) {
        p_target_ = p_cmd_;
        v_target_ = v_cmd_;
    } else {
        p_target_ = p_cmd_;
        v_target_.setZero();
    }

    V6d err = V6d::Zero();
    err.head<3>() = p_target_.pos - p_current_.pos;
    err.tail<3>() = QuatLogLocal(p_current_.quat, p_target_.quat);
    const V6d errd = v_target_ - v_current_;
    const V6d u = p_gain_ * err + d_gain_ * errd;
    for (int i = 0; i < 6; ++i) {
        v_current_(i) += u(i) * kControlDt;
    }
    if (v_current_.head<3>().norm() > limit_.max_line_v) {
        v_current_.head<3>() = v_current_.head<3>().normalized() * limit_.max_line_v;
    }
    if (v_current_.tail<3>().norm() > limit_.max_angle_v) {
        v_current_.tail<3>() = v_current_.tail<3>().normalized() * limit_.max_angle_v;
    }

    p_current_.pos += v_current_.head<3>() * kControlDt;
    p_current_.quat = (p_current_.quat * QuatExp(v_current_.tail<3>() * kControlDt)).normalized();
    p_flt_[flt_i_] = p_current_;

    V6d temp = V6d::Zero();
    for (int j = 0; j < kFilterSize; ++j) {
        temp.head<3>() += (p_flt_[j].pos - p_current_.pos);
        temp.tail<3>() += QuatLogLocal(p_current_.quat, p_flt_[j].quat);
    }
    temp /= kFilterSize;

    Pose out_pose;
    out_pose.pos = p_current_.pos + temp.head<3>();
    out_pose.quat = (QuatExp(temp.tail<3>()) * p_current_.quat).normalized();

    flt_i_ = (flt_i_ + 1) < kFilterSize ? (flt_i_ + 1) : 0;
    t_cur_ += kControlDt;

    const double pos_dis = (p_cmd_.pos - out_pose.pos).norm();
    const double ori_dis = QuatLogLocal(out_pose.quat, p_cmd_.quat).norm();
    if (pos_dis < kSmall && ori_dis < kLittle) {
        out_pose = p_cmd_;
        done_ = true;
    }

    V7d q;
    if (!IkSolver::Solve(arm_serial_, out_pose, ref_q_, q)) {
        done_ = true;
        init_ = false;
        return;
    }
    PushJointQ(predeal, q);
}
