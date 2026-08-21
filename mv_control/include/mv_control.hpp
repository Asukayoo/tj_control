#pragma once

#include "common.hpp"
#include "config.hpp"
#include "hw_interface.hpp"
#include "robot.hpp"

#include <memory>

class MVControl {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MVControl();
    ~MVControl();
    bool Init(const char* config_path = MV_CONTROL_CONFIG_DEFAULT, bool is_sim = false,
              std::shared_ptr<HwInterface> hw = nullptr,
              const char* urdf_override = nullptr);
    void Run();
    Robot& Left();
    Robot& Right();
    bool BothArmsStationary() const;

    struct HwRunStats {
        bool sent_this_cycle = false;
        uint64_t send_clear_fail_total = 0;
        uint64_t send_slot_wait_max_us = 0;
    };
    struct ArmTransitionDiag {
        uint16_t enable_trans_cycles = 0;
        uint16_t enable_trans_limit = 0;
        int sdk_cur_state = 0;
    };
    struct TransitionDiag {
        ArmTransitionDiag left;
        ArmTransitionDiag right;
    };
    const HwRunStats& LastHwRunStats() const { return last_hw_stats_; }
    const TransitionDiag& LastTransitionDiag() const { return last_transition_diag_; }
    void ResetHwRunStats();

private:
    void _ApplySnapshot(const HwSnapshot& snap, bool track_frame_serial);
    void _FillArmWrite(Robot& arm, HwArmWrite& slot);

    std::shared_ptr<HwInterface> hw_;
    ConnectConfig connect_cfg_{};
    bool is_sim_ = false;
    bool connected_ = false;
    HwRunStats last_hw_stats_{};
    TransitionDiag last_transition_diag_{};
    Robot left_;
    Robot right_;
};
