#pragma once

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

// 1kHz 周期统计（p50/p99、in[900,1100]%、overrun），写入文件
bool SavePeriodSummary(const char* path, const std::vector<uint16_t>& periods,
                       const std::vector<uint16_t>& overruns, int target_us);

// 偏离 [lo, hi] 的周期样本写入 CSV
bool SavePeriodAbnormalCsv(const char* path,
                           const std::vector<std::pair<int, int64_t>>& abnormal);

// 退出时打印一行周期稳定性摘要（详情见 period_summary.txt）
void PrintPeriodSummaryBrief(const std::vector<uint16_t>& periods,
                             const std::vector<uint16_t>& overruns, int target_us,
                             std::size_t abnormal_count);

void PrintPeriodStats(const RunRecorder& rec, int target_us);

// 从 RunRecorder 导出周期诊断并打印一行摘要（不逐条刷屏）
void SavePeriodDiagFromRecorder(const char* out_dir, const RunRecorder& rec,
                                int target_us = 1000, int64_t lo = 900, int64_t hi = 1100);
