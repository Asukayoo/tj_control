// 固定频率周期调度：CLOCK_MONOTONIC + clock_nanosleep(TIMER_ABSTIME)
// 标准模式：先睡到绝对节拍点，再执行本拍工作（无累积漂移）
#pragma once

#include <cerrno>
#include <cstdint>
#include <ctime>

struct CycleTiming {
    int64_t period_us = 0;    // 距上一拍起始的间隔 [us]
    int64_t wake_late_us = 0;   // 本拍睡醒相对绝对刻度的迟到 [us]
};

class PeriodicLoop {
public:
    explicit PeriodicLoop(int period_us = 1000) : period_ns_(period_us * 1000LL) {}

    // 重置单调时钟锚点；首拍不等待，立即开始
    void ResetAnchor() {
        clock_gettime(CLOCK_MONOTONIC, &next_tick_);
        has_prev_ = false;
        first_cycle_ = true;
    }

    // 阻塞至绝对节拍点，返回周期间隔与睡醒迟到
    CycleTiming WaitCycleStart() {
        CycleTiming out{};

        if (!first_cycle_) {
            const timespec deadline = next_tick_;
            while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline,
                                   nullptr) != 0) {
                if (errno != EINTR) {
                    break;
                }
            }
            clock_gettime(CLOCK_MONOTONIC, &cycle_start_);
            out.wake_late_us = DiffUs(cycle_start_, deadline);
            if (out.wake_late_us < 0) {
                out.wake_late_us = 0;
            }
        } else {
            clock_gettime(CLOCK_MONOTONIC, &cycle_start_);
        }

        if (has_prev_) {
            out.period_us = DiffUs(cycle_start_, prev_cycle_start_);
        }
        prev_cycle_start_ = cycle_start_;
        has_prev_ = true;

        AddNs(next_tick_, period_ns_);
        first_cycle_ = false;
        return out;
    }

    // 本拍工作结束后调用：相对下一绝对刻度的超时 [us]
    int64_t MeasureWorkOverrunUs() const {
        timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);
        const int64_t over = DiffUs(now, next_tick_);
        return over > 0 ? over : 0;
    }

    int64_t period_us() const { return period_ns_ / 1000; }

private:
    static int64_t DiffUs(const timespec& a, const timespec& b) {
        return (a.tv_sec - b.tv_sec) * 1000000LL +
               (a.tv_nsec - b.tv_nsec) / 1000LL;
    }

    static void AddNs(timespec& ts, int64_t ns) {
        ts.tv_nsec += ns;
        while (ts.tv_nsec >= 1000000000L) {
            ts.tv_nsec -= 1000000000L;
            ++ts.tv_sec;
        }
    }

    int64_t period_ns_;
    timespec next_tick_{};
    timespec cycle_start_{};
    timespec prev_cycle_start_{};
    bool has_prev_ = false;
    bool first_cycle_ = true;
};
