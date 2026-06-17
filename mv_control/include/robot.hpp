#pragma once

#include "common.hpp"
#include "config.hpp"

#include <memory>

enum class MotionType;
struct CmdPackage;

class MVControl;

class Robot {
    friend class MVControl;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    explicit Robot(int arm_serial = 0);
    ~Robot();
    Robot(Robot&&) noexcept;
    Robot& operator=(Robot&&) noexcept;
    Robot(const Robot&) = delete;
    Robot& operator=(const Robot&) = delete;

    bool SetEnable(EnableMode mode);
    EnableState GetEnableState() const;

    void Stop();
    void EStop();

    void ServoJ(const V7d& q);
    void ServoP(const Pose& pose);
    void ServoPByPico(const Pose& pose, bool is_run);
    void GoWork();
    void GoHome();
    void MovJ(const V7d& q);
    void MovL(const Pose& pose);
    bool SetControlMode(ControlMode control_mode);

    const RobotState& GetRefState() const;
    const RobotState& GetRespState() const;

    bool ClearError();
    StatusCode GetStatusCode() const;
    ErrorCode GetErrorCode() const;
    ControlModeStatus GetControlModeStatus() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

    bool Detect();
    void RunLogic();

    bool _SetRespState(const RobotState& rs);
    void _SetRefState(const RobotState& rs);

    bool _Init(bool is_sim);
    void _ApplyConfig(const ArmConfig& arm, const ServoConfig& servo,
                      const ConnectConfig& connect, const ImpConfig& imp);
    void _UpdateEnableState();

    void _ClearMotionCmds();
    void _TickTransitionTimeouts();
    bool _CallSdkControlMode(ControlMode mode);
    void _EnterStop();
    void _EnterStopOnFault();
    bool _CanAcceptCmd() const;
    void _SubmitStream(MotionType type, const CmdPackage& pkg);
    void _ApplyStreamCmd();
    void _ProcessCmdQueue();
    void _RunActiveMotion();
    void _RunActiveMotionIfStopping();
    void _UpdateStatus();
    bool _MotionDoneForSwitch();
    void _PostEnableDiag(EnableState prev_enable);
    void _LogErrorChange(ErrorCode prev, const char* source);
};
