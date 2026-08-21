#pragma once

#include "mv_control.hpp"
#include "periodic_loop.hpp"
#include "recorder.hpp"
#include "rt_thread.hpp"

#include <cstdint>

struct SessionOptions {
    std::size_t max_cycles = static_cast<std::size_t>(kControlMaxCycles5Min);
    int period_us = kControlPeriodUs;
    RtThreadOptions rt_thread{};
};

// 默认：仅 timer_slack（仿真/大 recorder 友好）
inline SessionOptions MakeDefaultSessionOptions(int period_us = kControlPeriodUs) {
    SessionOptions opts;
    opts.period_us = period_us;
    opts.rt_thread.timer_slack_ns = 1;
    return opts;
}

// 真机短测：FIFO + 绑核（避开末核 IRQ）；勿开 mlock（test 预分配 ~100MB recorder 会恶化抖动）
inline SessionOptions MakeHardRtSessionOptions(int period_us = kControlPeriodUs) {
    SessionOptions opts;
    opts.period_us = period_us;
    opts.rt_thread.use_fifo = true;
    opts.rt_thread.cpu = kRtCpuAutoIsolated;
    opts.rt_thread.sched_priority = 90;
    opts.rt_thread.timer_slack_ns = 1;
    return opts;
}

// 遥操作：50Hz 记录体量小，可 mlock 减少缺页抖动
inline SessionOptions MakeTeleopRtSessionOptions(int period_us = kControlPeriodUs) {
    SessionOptions opts = MakeHardRtSessionOptions(period_us);
    opts.rt_thread.lock_memory = true;
    return opts;
}

struct SessionTickResult {
    int64_t period_us = 0;
    int64_t overrun_us = 0;
    bool recorded = false;
};

class RunSession {
public:
    explicit RunSession(SessionOptions options = {});

    void ResetAnchor();
    void ReserveRecorder(std::size_t capacity);

    SessionTickResult Step(MVControl& ctrl, int cycle, uint8_t phase_packed = 0);

    RunRecorder& Recorder() { return recorder_; }
    const RunRecorder& Recorder() const { return recorder_; }

    bool ExportCsv(const char* dir, bool timing_with_phase) const;

private:
    SessionOptions options_;
    PeriodicLoop tick_;
    RunRecorder recorder_;
    uint64_t prev_clear_fail_ = 0;
};
