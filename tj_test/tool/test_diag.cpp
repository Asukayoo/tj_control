#include "test_diag.hpp"

#include "common.hpp"
#include "mv_control.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

struct PeriodStatsBlock {
    std::size_t n = 0;
    uint16_t p50 = 0;
    uint16_t p99 = 0;
    double mean = 0.0;
    double in_band_pct = 0.0;
};

struct PeriodReport {
    PeriodStatsBlock raw{};
    PeriodStatsBlock steady{};
    std::size_t catch_up = 0;
    uint64_t overrun_sum = 0;
    uint16_t overrun_max = 0;
    std::size_t overrun_n = 0;
};

PeriodStatsBlock ComputeBlock(const std::vector<uint16_t>& samples, int lo, int hi) {
    PeriodStatsBlock out{};
    if (samples.empty()) {
        return out;
    }
    std::vector<uint16_t> sorted = samples;
    std::sort(sorted.begin(), sorted.end());
    out.n = sorted.size();
    int64_t sum = 0;
    int in_band = 0;
    for (uint16_t p : sorted) {
        sum += p;
        if (p >= lo && p <= hi) {
            ++in_band;
        }
    }
    const auto pct = [&](double q) {
        const std::size_t idx = std::min(
            out.n - 1, static_cast<std::size_t>(q * static_cast<double>(out.n - 1)));
        return sorted[idx];
    };
    out.p50 = pct(0.50);
    out.p99 = pct(0.99);
    out.mean = static_cast<double>(sum) / static_cast<double>(out.n);
    out.in_band_pct = 100.0 * static_cast<double>(in_band) / static_cast<double>(out.n);
    return out;
}

PeriodReport BuildPeriodReport(const std::vector<uint16_t>& periods,
                               const std::vector<uint16_t>& overruns, int target_us) {
    PeriodReport rep{};
    if (periods.empty()) {
        return rep;
    }
    const int lo = static_cast<int>(kControlPeriodUsLo);
    const int hi = static_cast<int>(kControlPeriodUsHi);

    rep.raw = ComputeBlock(periods, lo, hi);

    std::vector<uint16_t> steady;
    steady.reserve(periods.size());
    for (uint16_t p : periods) {
        if (p < kControlCatchUpPeriodUs) {
            ++rep.catch_up;
            continue;
        }
        steady.push_back(p);
    }
    rep.steady = ComputeBlock(steady, lo, hi);

    for (uint16_t o : overruns) {
        rep.overrun_sum += o;
        rep.overrun_max = std::max(rep.overrun_max, o);
        ++rep.overrun_n;
    }
    return rep;
}

void WritePeriodSummaryImpl(FILE* f, const PeriodReport& rep, int target_us) {
    const int lo = static_cast<int>(kControlPeriodUsLo);
    const int hi = static_cast<int>(kControlPeriodUsHi);
    if (rep.raw.n == 0) {
        std::fprintf(f, "no_period_samples=1\n");
        return;
    }
    std::fprintf(
        f,
        "raw_period_us: min=see_abnormal max=see_abnormal p50=%u p99=%u mean=%.1f  "
        "in[%d,%d]=%.1f%%  (n=%zu, target=%d)\n",
        rep.raw.p50, rep.raw.p99, rep.raw.mean, lo, hi, rep.raw.in_band_pct, rep.raw.n,
        target_us);
    std::fprintf(
        f,
        "steady_period_us: p50=%u p99=%u mean=%.1f  in[%d,%d]=%.1f%%  "
        "(n=%zu, catch_up<%dus=%zu)\n",
        rep.steady.p50, rep.steady.p99, rep.steady.mean, lo, hi,
        rep.steady.in_band_pct, rep.steady.n, kControlCatchUpPeriodUs, rep.catch_up);
    std::fprintf(
        f,
        "note: catch_up 为工作超时后 TIMER_ABSTIME 连跳产生的短 period，"
        "不代表真实高于 %dHz；稳态 jitter 看 steady 行。\n",
        kControlHz);
    if (rep.overrun_n > 0) {
        std::fprintf(f, "overrun_us: mean=%.1f max=%u  (work>周期会触发 catch_up)\n",
                     static_cast<double>(rep.overrun_sum) /
                         static_cast<double>(rep.overrun_n),
                     rep.overrun_max);
    }
}

uint16_t OverrunMax(const std::vector<uint16_t>& overruns) {
    uint16_t max_o = 0;
    for (uint16_t o : overruns) {
        max_o = std::max(max_o, o);
    }
    return max_o;
}

bool IsActionableStall(const PeriodStallSample& s) {
    if (s.kind == nullptr) {
        return false;
    }
    if (std::strcmp(s.kind, "sched_gap") == 0) {
        return s.wake_late_us >= 200;
    }
    if (std::strcmp(s.kind, "work_over") == 0) {
        return true;
    }
    if (std::strcmp(s.kind, "period_short") == 0 ||
        std::strcmp(s.kind, "period_long") == 0 ||
        std::strcmp(s.kind, "catch_up") == 0) {
        return false;
    }
    return std::strcmp(s.kind, "jitter") == 0;
}

}  // namespace

void PrintJointQLine(const char* arm_name, const char* kind, const V7d& q) {
    std::printf("[init] %s %s[rad]:", arm_name, kind);
    for (int i = 0; i < DOF; ++i) {
        std::printf(" q%d=%.4f", i, q(i));
    }
    std::printf("\n");
}

void PrintArmInitJoints(const char* arm_name, const Robot& arm) {
    PrintJointQLine(arm_name, "ref_q", arm.GetRefState().joint_state.q);
    PrintJointQLine(arm_name, "resp_q", arm.GetRespState().joint_state.q);
}

void PrintInitJointStates(MVControl& ctrl) {
    PrintArmInitJoints("left", ctrl.Left());
    PrintArmInitJoints("right", ctrl.Right());
}

bool SavePeriodSummary(const char* path, const std::vector<uint16_t>& periods,
                       const std::vector<uint16_t>& overruns, int target_us) {
    if (path == nullptr) {
        return false;
    }
    FILE* f = std::fopen(path, "w");
    if (f == nullptr) {
        return false;
    }
    WritePeriodSummaryImpl(f, BuildPeriodReport(periods, overruns, target_us), target_us);
    std::fclose(f);
    return true;
}

bool SavePeriodAbnormalCsv(const char* path,
                           const std::vector<std::pair<int, int64_t>>& abnormal) {
    if (path == nullptr) {
        return false;
    }
    FILE* f = std::fopen(path, "w");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f, "cycle,period_us,kind\n");
    for (const auto& [c, us] : abnormal) {
        const char* kind =
            (us < kControlCatchUpPeriodUs) ? "catch_up" : "jitter";
        std::fprintf(f, "%d,%ld,%s\n", c, static_cast<long>(us), kind);
    }
    std::fclose(f);
    return true;
}

bool SavePeriodAbnormalCsv(const char* path,
                           const std::vector<PeriodStallSample>& stalls) {
    if (path == nullptr) {
        return false;
    }
    FILE* f = std::fopen(path, "w");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f,
                 "cycle,period_us,kind,wake_late_us,work_us,poll_us,servo_us,run_us,"
                 "udp_tx_us,record_us,after_us,phase_packed,hw_sent,attr_src\n");
    for (const PeriodStallSample& s : stalls) {
        std::fprintf(f, "%d,%ld,%s,%ld,%u,%u,%u,%u,%u,%u,%u,%u,%u,%s\n", s.cycle,
                     static_cast<long>(s.period_us), s.kind != nullptr ? s.kind : "",
                     static_cast<long>(s.wake_late_us), s.work_us, s.poll_us, s.servo_us,
                     s.run_us, s.udp_tx_us, s.record_us, s.after_us, s.phase_packed,
                     s.hw_sent, s.attr_src != nullptr ? s.attr_src : "");
    }
    std::fclose(f);
    return true;
}

void PrintPeriodSummaryBrief(const std::vector<uint16_t>& periods,
                             const std::vector<uint16_t>& overruns, int target_us,
                             std::size_t abnormal_count) {
    const PeriodReport rep = BuildPeriodReport(periods, overruns, target_us);
    if (rep.raw.n == 0) {
        std::printf("[period] 无有效样本\n");
        return;
    }
    const int lo = static_cast<int>(kControlPeriodUsLo);
    const int hi = static_cast<int>(kControlPeriodUsHi);
    std::printf(
        "[period] session p50=%u p99=%u in[%d,%d]=%.1f%% (n=%zu)  "
        "catch_up=%zu  abnormal=%zu\n",
        rep.steady.p50, rep.steady.p99, lo, hi, rep.steady.in_band_pct, rep.steady.n,
        rep.catch_up, abnormal_count);
    if (rep.overrun_n > 0) {
        std::printf("[period] session overrun_us mean=%.1f max=%u\n",
                    static_cast<double>(rep.overrun_sum) /
                        static_cast<double>(rep.overrun_n),
                    rep.overrun_max);
    }
}

PeriodVerdict EvaluatePeriodVerdict(const std::vector<uint16_t>& teleop_periods,
                                    const std::vector<uint16_t>& teleop_overruns,
                                    const std::vector<PeriodStallSample>& teleop_stalls,
                                    const std::vector<uint16_t>& session_periods,
                                    const std::vector<uint16_t>& session_overruns,
                                    std::size_t session_abnormal_count) {
    PeriodVerdict v{};
    const int target_us = kControlPeriodUs;
    const PeriodReport teleop_rep =
        BuildPeriodReport(teleop_periods, teleop_overruns, target_us);
    v.teleop_n = teleop_rep.steady.n;
    v.teleop_in_band_pct = teleop_rep.steady.in_band_pct;
    v.teleop_overrun_max = teleop_rep.overrun_max;

    std::size_t actionable = 0;
    for (const PeriodStallSample& s : teleop_stalls) {
        if (IsActionableStall(s)) {
            ++actionable;
        }
    }
    v.teleop_abnormal = actionable;

    constexpr double kMinInBandPct = 99.5;
    constexpr uint16_t kMaxOverrunUs = 200;
    v.teleop_pass = teleop_rep.steady.n > 0 && actionable == 0 &&
                    teleop_rep.steady.in_band_pct >= kMinInBandPct &&
                    teleop_rep.overrun_max <= kMaxOverrunUs;

    const PeriodReport session_rep =
        BuildPeriodReport(session_periods, session_overruns, target_us);
    v.session_pass = session_rep.steady.in_band_pct >= kMinInBandPct &&
                     session_rep.overrun_max <= kMaxOverrunUs &&
                     session_abnormal_count <= 2;
    return v;
}

void PrintPeriodVerdict(const PeriodVerdict& v, int target_us) {
    const int lo = static_cast<int>(kControlPeriodUsLo);
    const int hi = static_cast<int>(kControlPeriodUsHi);
    std::printf(
        "[period] teleop %s p50/p99 in[%d,%d]=%.1f%% n=%zu overrun_max=%u "
        "actionable_abnormal=%zu\n",
        v.teleop_pass ? "PASS" : "FAIL", lo, hi, v.teleop_in_band_pct, v.teleop_n,
        v.teleop_overrun_max, v.teleop_abnormal);
    if (v.session_pass && !v.teleop_pass) {
        std::printf("[period] note: 全 session 含 GoHome/下使能 catch-up 对；"
                    "遥操作段已通过即可。\n");
    } else if (!v.session_pass) {
        std::printf("[period] session %s（含收尾）；目标 period=%dus "
                    "overrun_max<=200us\n",
                    v.session_pass ? "PASS" : "FAIL", target_us);
    }
}

void PrintMaxStallBrief(const std::vector<PeriodStallSample>& stalls) {
    if (stalls.empty()) {
        return;
    }
    const PeriodStallSample* best = nullptr;
    for (const PeriodStallSample& s : stalls) {
        if (!IsActionableStall(s)) {
            continue;
        }
        if (best == nullptr || s.period_us > best->period_us ||
            (s.period_us == best->period_us && s.work_us > best->work_us)) {
            best = &s;
        }
    }
    if (best == nullptr) {
        return;
    }
    std::printf(
        "[period] stall cycle=%d period=%ld kind=%s wake_late=%ld "
        "attr=%s work=%u poll=%u servo=%u run=%u udp_tx=%u record=%u after=%u "
        "phase=%u hw_sent=%u\n",
        best->cycle, static_cast<long>(best->period_us),
        best->kind != nullptr ? best->kind : "",
        static_cast<long>(best->wake_late_us),
        best->attr_src != nullptr ? best->attr_src : "", best->work_us, best->poll_us,
        best->servo_us, best->run_us, best->udp_tx_us, best->record_us, best->after_us,
        best->phase_packed, best->hw_sent);
}

void PrintPeriodStats(const RunRecorder& rec, int target_us) {
    std::vector<uint16_t> periods;
    std::vector<uint16_t> overruns;
    periods.reserve(rec.Size());
    overruns.reserve(rec.Size());
    for (std::size_t i = 0; i < rec.Size(); ++i) {
        const CycleSample& s = rec[i];
        if (s.period_us > 0) {
            periods.push_back(s.period_us);
        }
        overruns.push_back(s.overrun_us);
    }
    WritePeriodSummaryImpl(stdout, BuildPeriodReport(periods, overruns, target_us),
                           target_us);
}

void SavePeriodDiagFromRecorder(const char* out_dir, const RunRecorder& rec,
                                int target_us, int64_t lo, int64_t hi) {
    std::vector<uint16_t> periods;
    std::vector<uint16_t> overruns;
    std::vector<std::pair<int, int64_t>> abnormal;
    periods.reserve(rec.Size());
    overruns.reserve(rec.Size());
    abnormal.reserve(64);
    for (std::size_t i = 0; i < rec.Size(); ++i) {
        const CycleSample& s = rec[i];
        if (s.period_us > 0) {
            periods.push_back(s.period_us);
            const int64_t pus = s.period_us;
            if (pus < lo || pus > hi) {
                abnormal.emplace_back(static_cast<int>(s.cycle), pus);
            }
        }
        overruns.push_back(s.overrun_us);
    }
    if (out_dir != nullptr && !periods.empty()) {
        const std::string base(out_dir);
        SavePeriodSummary((base + "/period_summary.txt").c_str(), periods, overruns,
                          target_us);
        SavePeriodAbnormalCsv((base + "/period_abnormal.csv").c_str(), abnormal);
    }
    PrintPeriodSummaryBrief(periods, overruns, target_us, abnormal.size());
}
