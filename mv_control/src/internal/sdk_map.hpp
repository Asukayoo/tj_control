#pragma once

#include "common.hpp"
#include "hw_interface.hpp"
#include "in_data.hpp"

// StateCmdType → HwStateCommand::Op（与 StateCmdType 枚举序一致）
inline constexpr HwStateCommand::Op kStateCmdHwOpMap[] = {
    HwStateCommand::Op::Enable,
    HwStateCommand::Op::Disable,
    HwStateCommand::Op::ClearError,
    HwStateCommand::Op::EStop,
    HwStateCommand::Op::SetPositionMode,
    HwStateCommand::Op::SetJointImp,
    HwStateCommand::Op::SetCartImp,
    HwStateCommand::Op::SetForce,
};

// --- 模式映射 ---
ControlMode MapSdkToControlMode(int cur_state, int imp_type);
bool IsInitArmStateAllowed(int cur_state, bool strict_init_state);
bool IsModeTransitionState(int arm_state);
bool ShouldReportSdkModeMismatch(int cur_state, int imp_type,
                                 EnableMode enable_mode,
                                 ControlMode control_mode_target);

// --- 错误映射 ---
bool HasServoErr(const SdkErrorDetail& sdk);
int ErrorPriority(ErrorCode code);
ErrorCode PickHigherPriorityError(ErrorCode a, ErrorCode b);
ErrorCode MapSdkToError(const SdkErrorDetail& sdk,
                        ErrorCode internal_hint = ErrorCode::Normal);
