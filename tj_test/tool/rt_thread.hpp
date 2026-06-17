// POSIX 实时线程配置：固定频率控制循环的标准前置条件
#pragma once

inline constexpr int kRtCpuOff = -1;        // 不绑核
inline constexpr int kRtCpuAutoLast = -2;   // 绑定到最后一个逻辑核

// 1 kHz 控制环 RT 选项（默认仅收紧 timer_slack；FIFO/绑核按需开启）
struct RtThreadOptions {
    int cpu = kRtCpuOff;           // kRtCpuOff / kRtCpuAutoLast / >=0
    int sched_priority = 80;       // SCHED_FIFO 1–99
    bool use_fifo = false;         // 需 setcap；非 RT 核上未必更稳
    bool lock_memory = false;      // mlockall，按需
    int timer_slack_ns = 1;        // prctl(PR_SET_TIMERSLACK)，默认开启
    bool abort_on_rt_failure = false;
};

struct RtThreadStatus {
    bool timer_slack_ok = false;
    bool affinity_ok = false;
    bool memory_locked = false;
    bool fifo_ok = false;
    int bound_cpu = -1;  // 实际绑定的 CPU；-1 表示未绑
};

// 对当前线程应用 RT 选项；可重复调用，内部只执行一次
RtThreadStatus ApplyRtThreadOptions(const RtThreadOptions& opt = {});
