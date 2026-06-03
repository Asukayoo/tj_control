#pragma once

#include "common.hpp"
#include "config.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>

#include "internal/in_data.hpp"
#include "internal/motion.hpp"

class MVControl;

class Robot {
    friend class MVControl;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit Robot(int arm_serial = 0);
    ~Robot();

    void Stop();
    void ServoJ(const V7d& q);
    void ServoP(const Pose& pose);
    void GoWork();
    void GoHome();
    void MovJ(const V7d& q);
    void MovL(const Pose& pose);
    void SetEnableMode(EnableMode enable_mode);
    void SetControlMode(ControlMode control_mode);
    RobotState GetRefState() const;
    RobotState GetRespState() const;
    void ClearError();
    StatusCode GetStatusCode() const;
    ErrorCode GetErrorCode() const;
private:
    void _SetRefState(const RobotState& rs);
    void _SetRespState(const RobotState& rs);
    bool _Init();
    void _ApplyArmConfig(const ArmConfig& cfg);
    void _Run();
    bool _Detect();
    void _PushCmd(CmdPackage pkg);
    void _ProcessCmdQueue();
    void _RunActiveMotion();
    void _ApplyPredeal();
    void _UpdateStatus();
    bool _IsServoCmd(CmdType type) const;
    bool _IsServoMotion(MotionKind kind) const;
    bool _MotionDoneForSwitch();

    int arm_serial_ = 0;
    V7d work_q_ = V7d::Zero();
    V7d home_q_ = V7d::Zero();
    RobotState ref_rs_{};
    RobotState resp_rs_{};
    EnableMode enable_mode_ = EnableMode::Disable;
    ControlMode control_mode_ = ControlMode::Position;
    StatusCode status_code_ = StatusCode::Ready;
    ErrorCode error_code_ = ErrorCode::Normal;
    JointLimit joint_limit_{};
    CartLimit cart_limit_{};

    std::deque<CmdPackage> cmd_queue_;
    std::deque<JointState> predeal_queue_;
    MotionKind active_motion_ = MotionKind::None;
    bool stop_pending_ = false;
    bool motion_inited_ = false;

    MotionStop motion_stop_;
    MotionMovJ motion_movj_;
    MotionMovL motion_movl_;
    MotionServoJ motion_servoj_;
    MotionServoP motion_servop_;
    std::optional<CmdPackage> active_cmd_;
};

class MVControl {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MVControl();
    ~MVControl();
    bool Init(uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4,
              int log_switch = 0, const char* kine_cfg = nullptr);
    bool InitFromConfig(const char* yaml_path);
    void Run();
    Robot& Left();
    Robot& Right();
private:
#ifndef MV_CONTROL_SIM
    bool _ClearHwErrors();
    bool _ReadHwToRobots();
    bool _WriteRobotsToHw();
#endif
    bool connected_ = false;
    Robot left_;
    Robot right_;
};
