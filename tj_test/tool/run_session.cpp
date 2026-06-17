#include "run_session.hpp"

#include <algorithm>

RunSession::RunSession(SessionOptions options)
    : options_(options), tick_(options.period_us) {}

void RunSession::ResetAnchor() {
    ApplyRtThreadOptions(options_.rt_thread);
    tick_.ResetAnchor();
    prev_clear_fail_ = 0;
}

void RunSession::ReserveRecorder(std::size_t capacity) {
    recorder_.Reserve(capacity);
}

SessionTickResult RunSession::Step(MVControl& ctrl, int cycle, uint8_t phase_packed) {
    SessionTickResult out{};
    const CycleTiming timing = tick_.WaitCycleStart();
    out.period_us = timing.period_us;

    ctrl.Run();

    const MVControl::HwRunStats& stats = ctrl.LastHwRunStats();
    uint8_t flags = 0;
    if (stats.sent_this_cycle) {
        flags |= kSampleHwSent;
    }
    if (stats.send_clear_fail_total > prev_clear_fail_) {
        flags |= kSampleClearFail;
        prev_clear_fail_ = stats.send_clear_fail_total;
    }

    CycleSample sample{};
    FillCycleSampleFromControl(sample, ctrl, cycle, phase_packed, flags);
    if (out.period_us > 0 && out.period_us <= 65535) {
        sample.period_us = static_cast<uint16_t>(out.period_us);
    }

    const int64_t work_overrun_us = tick_.MeasureWorkOverrunUs();
    out.overrun_us = std::max(timing.wake_late_us, work_overrun_us);
    if (out.overrun_us >= 0 && out.overrun_us <= 65535) {
        sample.overrun_us = static_cast<uint16_t>(out.overrun_us);
    }

    out.recorded = recorder_.Push(sample);
    return out;
}

bool RunSession::ExportCsv(const char* dir, bool timing_with_phase) const {
    return ExportSessionCsv(recorder_, dir, timing_with_phase);
}
