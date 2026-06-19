#pragma once

#include "robot.hpp"

#include "config.hpp"

#include <cstdint>
#include <deque>
#include <optional>

#include "in_data.hpp"
#include "motion.hpp"

struct Robot::Impl {
    bool is_sim_ = false;
    int arm_serial_ = 0;
    V7d work_q_ = V7d::Zero();
    V7d home_q_ = V7d::Zero();
    RobotState ref_rs_{};
    RobotState resp_rs_{};
    ControlMode control_mode_target_ = ControlMode::Position;
    ControlMode control_mode_actual_ = ControlMode::Position;
    StatusCode status_code_ = StatusCode::Disabled;
    ErrorCode error_code_ = ErrorCode::Normal;
    EnableMode enable_mode_ = EnableMode::Disable;
    EnableState enable_state_ = EnableState::Disabled;
    JointLimit joint_limit_{};
    CartLimit cart_limit_{};
    int vel_ratio_ = 10;
    int acc_ratio_ = 10;
    bool strict_init_state_ = true;
    int mode_transition_timeout_cycles_ = 1000;
    int enable_transition_cycles_ = 0;  // Enabling/Disabling 周期计数
    int mode_transition_cycles_ = 0;    // 控制模式切换周期计数

    std::deque<CmdPackage> cmd_queue_;
    std::optional<CmdPackage> stream_cmd_;
    bool stream_dirty_ = false;
    MotionType active_motion_ = MotionType::None;
    bool stop_pending_ = false;
    bool motion_inited_ = false;

    MotionStop motion_stop_;
    MotionMovJ motion_movj_;
    MotionMovL motion_movl_;
    MotionServoJ motion_servoj_;
    MotionServoP motion_servop_;
    MotionServoPByPico motion_servop_pico_;
    std::optional<CmdPackage> active_cmd_;

    ImpConfig imp_config_{};
    std::deque<StateCmdPackage> pending_state_queue_;
    std::optional<StateCmdPackage> immediate_state_cmd_;

    SdkErrorDetail sdk_detail_{};
    int sdk_last_frame_serial_ = 0;
    bool read_buf_ok_ = true;
    int low_spd_flag_ = 0;

    // 诊断：使能/错误状态变化追踪
    ErrorCode diag_last_error_ = ErrorCode::Normal;
    EnableState diag_last_enable_ = EnableState::Disabled;
    bool diag_logged_enable_mismatch_ = false;

    Impl(int arm_serial, const JointLimit& joint_lim, const CartLimit& cart_lim);
};
