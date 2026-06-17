#include "test_diag.hpp"

#include "mv_control.hpp"

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

namespace {

// period_us 低于此阈值视为 TIMER_ABSTIME 追赶拍（非真实高频）
constexpr int kCatchUpPeriodUs = 500;

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
    const int lo = target_us - 100;
    const int hi = target_us + 100;

    rep.raw = ComputeBlock(periods, lo, hi);

    std::vector<uint16_t> steady;
    steady.reserve(periods.size());
    for (uint16_t p : periods) {
        if (p < kCatchUpPeriodUs) {
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
    if (rep.raw.n == 0) {
        std::fprintf(f, "no_period_samples=1\n");
        return;
    }
    std::fprintf(
        f,
        "raw_period_us: min=see_abnormal max=see_abnormal p50=%u p99=%u mean=%.1f  "
        "in[900,1100]=%.1f%%  (n=%zu, target=%d)\n",
        rep.raw.p50, rep.raw.p99, rep.raw.mean, rep.raw.in_band_pct, rep.raw.n,
        target_us);
    std::fprintf(
        f,
        "steady_period_us: p50=%u p99=%u mean=%.1f  in[900,1100]=%.1f%%  "
        "(n=%zu, catch_up<%dus=%zu)\n",
        rep.steady.p50, rep.steady.p99, rep.steady.mean, rep.steady.in_band_pct,
        rep.steady.n, kCatchUpPeriodUs, rep.catch_up);
    std::fprintf(
        f,
        "note: catch_up 为工作超时后 TIMER_ABSTIME 连跳产生的短 period，"
        "不代表真实高于 1kHz；稳态 jitter 看 steady 行。\n");
    if (rep.overrun_n > 0) {
        std::fprintf(f, "overrun_us: mean=%.1f max=%u  (work>1ms 会触发 catch_up)\n",
                     static_cast<double>(rep.overrun_sum) /
                         static_cast<double>(rep.overrun_n),
                     rep.overrun_max);
    }
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
            (us < kCatchUpPeriodUs) ? "catch_up" : "jitter";
        std::fprintf(f, "%d,%ld,%s\n", c, static_cast<long>(us), kind);
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
    std::printf(
        "[period] steady p50=%u p99=%u in[900,1100]=%.1f%% (n=%zu)  "
        "catch_up=%zu  raw_in_band=%.1f%%  abnormal=%zu\n",
        rep.steady.p50, rep.steady.p99, rep.steady.in_band_pct, rep.steady.n,
        rep.catch_up, rep.raw.in_band_pct, abnormal_count);
    if (rep.overrun_n > 0) {
        std::printf("[period] overrun_us mean=%.1f max=%u\n",
                    static_cast<double>(rep.overrun_sum) /
                        static_cast<double>(rep.overrun_n),
                    rep.overrun_max);
    }
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
