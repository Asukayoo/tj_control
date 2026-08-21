// POSIX 实时线程配置：固定频率控制循环的标准前置条件
#pragma once

inline constexpr int kRtCpuOff = -1;           // 不绑核
inline constexpr int kRtCpuAutoLast = -2;        // 绑定到最后一个逻辑核
inline constexpr int kRtCpuAutoIsolated = -3;  // 避开末核（常承载 IRQ）；≥4 核时用 ncpu-2

// kControlHz 控制环 RT 选项（默认仅收紧 timer_slack；FIFO/绑核按需开启）
struct RtThreadOptions {
    int cpu = kRtCpuOff;  // kRtCpuOff / kRtCpuAuto* / >=0；可被环境变量 TJ_RT_CPU 覆盖
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

// 解析绑核目标；TJ_RT_CPU 优先于 cpu_hint
int ResolveRtCpu(int cpu_hint = kRtCpuAutoIsolated);

// 对当前线程应用 RT 选项；可重复调用，内部只执行一次
RtThreadStatus ApplyRtThreadOptions(const RtThreadOptions& opt = {});
