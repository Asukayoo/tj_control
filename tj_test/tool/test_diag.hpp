#pragma once

#include "common.hpp"
#include "recorder.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

class MVControl;
class Robot;

// Init 完成后打印单臂 ref/resp 关节角 [rad]
void PrintArmInitJoints(const char* arm_name, const Robot& arm);

// Init 完成后打印左右臂 ref/resp 关节角 [rad]
void PrintInitJointStates(MVControl& ctrl);

// 500Hz 周期统计（p50/p99、in-band%、overrun），写入文件
bool SavePeriodSummary(const char* path, const std::vector<uint16_t>& periods,
                       const std::vector<uint16_t>& overruns, int target_us);

// 偏离 [lo, hi] 的周期样本写入 CSV（旧格式：cycle,period_us,kind）
bool SavePeriodAbnormalCsv(const char* path,
                           const std::vector<std::pair<int, int64_t>>& abnormal);

// 分段归因异常拍（period 越界或 work 超时）
struct PeriodStallSample {
    int cycle = 0;
    int64_t period_us = 0;
    const char* kind = "";  // catch_up | period_short | period_long | jitter | work_over | sched_gap
    int64_t wake_late_us = 0;
    uint16_t work_us = 0;
    uint16_t poll_us = 0;
    uint16_t servo_us = 0;
    uint16_t run_us = 0;
    uint16_t udp_tx_us = 0;
    uint16_t record_us = 0;
    uint16_t after_us = 0;
    uint8_t phase_packed = 0;
    uint8_t hw_sent = 0;
    const char* attr_src = "";  // prev | curr | none
};

bool SavePeriodAbnormalCsv(const char* path,
                           const std::vector<PeriodStallSample>& stalls);

// 退出时打印一行周期稳定性摘要（详情见 period_summary.txt）
void PrintPeriodSummaryBrief(const std::vector<uint16_t>& periods,
                             const std::vector<uint16_t>& overruns, int target_us,
                             std::size_t abnormal_count);

// 500Hz 遥操作验收：teleop 段与全 session 分开判定
struct PeriodVerdict {
    bool teleop_pass = false;
    bool session_pass = false;
    std::size_t teleop_n = 0;
    std::size_t teleop_abnormal = 0;
    uint16_t teleop_overrun_max = 0;
    double teleop_in_band_pct = 0.0;
};

PeriodVerdict EvaluatePeriodVerdict(const std::vector<uint16_t>& teleop_periods,
                                     const std::vector<uint16_t>& teleop_overruns,
                                     const std::vector<PeriodStallSample>& teleop_stalls,
                                     const std::vector<uint16_t>& session_periods,
                                     const std::vector<uint16_t>& session_overruns,
                                     std::size_t session_abnormal_count);

void PrintPeriodVerdict(const PeriodVerdict& v, int target_us);

// 打印最大 stall 一行（便于终端直接看归因）
void PrintMaxStallBrief(const std::vector<PeriodStallSample>& stalls);

void PrintPeriodStats(const RunRecorder& rec, int target_us);

// 从 RunRecorder 导出周期诊断并打印一行摘要（不逐条刷屏）
void SavePeriodDiagFromRecorder(const char* out_dir, const RunRecorder& rec,
                                int target_us = kControlPeriodUs,
                                int64_t lo = kControlPeriodUsLo,
                                int64_t hi = kControlPeriodUsHi);
