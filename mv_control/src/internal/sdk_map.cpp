#include "sdk_map.hpp"

namespace {

bool IsHardwareArmErr(int code) {
    return code == 1 || code == 2 || code == 12 || code == 13;
}

bool IsModeArmErr(int code) {
    return code >= 3 && code <= 7;
}

bool IsEnableArmErr(int code) {
    return code >= 8 && code <= 11;
}

}  // namespace

ControlMode MapSdkToControlMode(int cur_state, int imp_type) {
    if (cur_state == 1) {
        return ControlMode::Position;
    }
    if (cur_state == 3) {
        switch (imp_type) {
            case 1:
                return ControlMode::JointImp;
            case 2:
                return ControlMode::CartImp;
            case 3:
                return ControlMode::Force;
            default:
                break;
        }
    }
    return ControlMode::Position;
}

bool IsInitArmStateAllowed(int cur_state, bool strict_init_state) {
    if (!strict_init_state) {
        return true;
    }
    return cur_state == 0 || cur_state == 1;
}

bool IsModeTransitionState(int arm_state) {
    return arm_state == 101 || arm_state == 102 || arm_state == 103 ||
           arm_state == 104 || arm_state == 109;
}

bool ShouldReportSdkModeMismatch(int cur_state, int imp_type,
                                 EnableMode enable_mode,
                                 ControlMode control_mode_target) {
    if (enable_mode != EnableMode::Enable) {
        return false;
    }
    if (cur_state == 2 || cur_state == 4) {
        return true;
    }
    if (cur_state == 3 && control_mode_target == ControlMode::Position) {
        return true;
    }
    if (cur_state == 1 && control_mode_target != ControlMode::Position) {
        return true;
    }
    if (cur_state == 3) {
        const ControlMode actual = MapSdkToControlMode(cur_state, imp_type);
        if (actual != control_mode_target) {
            return true;
        }
    }
    return false;
}

bool HasServoErr(const SdkErrorDetail& sdk) {
    for (long e : sdk.servo_err) {
        if (e != 0) {
            return true;
        }
    }
    return false;
}

int ErrorPriority(ErrorCode code) {
    switch (code) {
        case ErrorCode::HardwareError:
            return 7;
        case ErrorCode::ConnectError:
            return 6;
        case ErrorCode::ModeError:
            return 5;
        case ErrorCode::EnableError:
            return 4;
        case ErrorCode::ConfigError:
            return 3;
        case ErrorCode::InitError:
            return 2;
        case ErrorCode::MotionError:
        case ErrorCode::PlanErr:
            return 1;
        case ErrorCode::Normal:
        default:
            return 0;
    }
}

ErrorCode PickHigherPriorityError(ErrorCode a, ErrorCode b) {
    return ErrorPriority(a) >= ErrorPriority(b) ? a : b;
}

ErrorCode MapSdkToError(const SdkErrorDetail& sdk, ErrorCode internal_hint) {
    ErrorCode out = internal_hint;

    if (sdk.frame_stale_cycles >= kSdkFrameStaleRunCycles) {
        out = PickHigherPriorityError(out, ErrorCode::ConnectError);
    }
    if (sdk.arm_state == 100 || IsHardwareArmErr(sdk.arm_err_code) ||
        sdk.arm_err_code == 2) {
        out = PickHigherPriorityError(out, ErrorCode::HardwareError);
    }
    if (sdk.servo_err_fresh && HasServoErr(sdk)) {
        out = PickHigherPriorityError(out, ErrorCode::HardwareError);
    }
    if (IsModeArmErr(sdk.arm_err_code)) {
        out = PickHigherPriorityError(out, ErrorCode::ModeError);
    }
    if (IsEnableArmErr(sdk.arm_err_code)) {
        out = PickHigherPriorityError(out, ErrorCode::EnableError);
    }
    if (sdk.arm_err_code == 14) {
        out = PickHigherPriorityError(out, ErrorCode::ConfigError);
    }

    return out;
}
