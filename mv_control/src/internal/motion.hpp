#pragma once

#include <array>

#include <ruckig/ruckig.hpp>

#include "common.hpp"
#include "in_data.hpp"

constexpr int kFilterSize = 10;

struct CubicCoef {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;
};

class MotionStop {
public:
    explicit MotionStop(const JointLimit& limit);
    bool IsDone() const;
    void InitPlan(const RobotState& ref_rs);
    void RunPlan(RobotState& ref_rs);
    void SetLimit(const JointLimit& limit);
private:
    bool done_ = true;
    bool init_ = false;
    JointLimit limit_;
    V7d axis_t_ = V7d::Zero();
    double speed_norm_ = 0.0;
    double acc_norm_ = 0.0;
    double stopacc_ = 0.0;
    uint32_t cycle_count_ = 0;
};

class MotionMovJ {
public:
    explicit MotionMovJ(const JointLimit& limit);
    bool IsDone() const;
    bool TrajValid() const { return traj_ok_; }
    void InitPlan(const V7d& target_q, const RobotState& ref_rs);
    void RunPlan(RobotState& ref_rs);
    void SetLimit(const JointLimit& limit);
private:
    bool done_ = true;
    bool init_ = false;
    JointLimit limit_;
    ruckig::Ruckig<DOF> ruckig_{kControlDt};
    ruckig::Trajectory<DOF> trajectory_;
    double plan_time_ = 0.0;
    bool traj_ok_ = false;
};

class MotionServoJ {
public:
    explicit MotionServoJ(const JointLimit& limit);
    void InitPlan(const V7d& target_q, const RobotState& ref_rs);
    void RePlan(const V7d& target_q, const RobotState& ref_rs);
    void RunPlan(RobotState& ref_rs);
    void SetLimit(const JointLimit& limit);
    void SetPdGain(double p_gain, double d_gain);
private:
    bool init_ = false;
    JointLimit limit_;
    V7d p_target_ = V7d::Zero();  // 三次样条输出
    V7d v_target_ = V7d::Zero();
    V7d p_cmd_ = V7d::Zero();
    V7d v_cmd_ = V7d::Zero();
    std::array<std::array<double, kFilterSize>, DOF> p_flt_{};
    uint8_t flt_i_ = 0;
    double t_ = kStreamServoPeriod;
    double t_cur_ = 0.0;
    std::array<CubicCoef, DOF> coef_{};
    V7d last_out_ = V7d::Zero();
};

class MotionMovL {
public:
    MotionMovL(const CartLimit& limit, int arm_serial);
    bool IsDone() const;
    void InitPlan(const Pose& target_pose, const RobotState& ref_rs);
    void RunPlan(RobotState& ref_rs);
    void SetLimit(const CartLimit& limit);
private:
    bool done_ = true;
    bool init_ = false;
    CartLimit limit_;
    int arm_serial_ = 0;
    Pose start_pose_{};
    Pose target_pose_{};
    double line_dist_ = 0.0;
    double rot_angle_ = 0.0;
    ruckig::Ruckig<2> ruckig_{kControlDt};
    ruckig::Trajectory<2> trajectory_;
    double plan_time_ = 0.0;
    bool traj_ok_ = false;
    V7d last_q_ = V7d::Zero();
};

class MotionServoP {
public:
    MotionServoP(const CartLimit& limit, int arm_serial);
    void InitPlan(const Pose& target_pose, const RobotState& ref_rs, const V7d& ref_q);
    void RePlan(const Pose& target_pose, const RobotState& ref_rs, const V7d& ref_q);
    void RunPlan(RobotState& ref_rs);
    void Reset();
    void SetLimit(const CartLimit& limit);
    void SetPdGain(double p_gain, double d_gain);
private:
    void ResetFilter(const Pose& seed);
    Pose FilterPose() const;

    bool init_ = false;
    CartLimit limit_;
    int arm_serial_ = 0;
    V7d ref_q_ = V7d::Zero();
    Pose p_target_{};  // 三次样条输出
    Pose p_cmd_{};
    Pose p_cmd_last_{};
    V6d v_target_ = V6d::Zero();
    V6d v_cmd_ = V6d::Zero();
    std::array<Pose, kFilterSize> p_flt_{};
    uint8_t flt_i_ = 0;
    double t_ = kStreamServoPeriod;
    double t_cur_ = 0.0;
    std::array<CubicCoef, 6> coef_{};
    V7d last_q_ = V7d::Zero();
};

class MotionServoPByPico {
public:
    MotionServoPByPico(const CartLimit& limit, int arm_serial);
    void InitPlan(const Pose& pico_pose, const RobotState& ref_rs, const V7d& ref_q);
    void RePlan(const Pose& pico_pose, const RobotState& ref_rs, const V7d& ref_q);
    void RunPlan(RobotState& ref_rs);
    void ResetSession();
    void SetLimit(const CartLimit& limit);
    void SetPdGain(double p_gain, double d_gain);
private:
    void PicoToAbsTarget(const Pose& pico_pose, Pose& out) const;

    MotionServoP servo_;
    int arm_serial_ = 0;
    Pose ref_pico_{};
    Pose robot_anchor_{};
    bool session_active_ = false;
};
