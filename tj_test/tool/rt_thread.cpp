#include "rt_thread.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <unistd.h>

namespace {

bool g_rt_applied = false;

void AbortRtSetup(const char* reason) {
    std::fprintf(stderr, "[rt] FATAL: %s\n", reason);
    std::fprintf(stderr,
                 "[rt] 请执行其一后重试:\n"
                 "       sudo setcap cap_sys_nice,cap_ipc_lock+ep <test_binary>\n"
                 "       sudo chrt -f 80 <test_binary>\n"
                 "       或在 /etc/security/limits.conf 配置 rtprio/memlock 后重新登录\n");
    std::exit(1);
}

}  // namespace

int ResolveRtCpu(int cpu_hint) {
    if (const char* env = std::getenv("TJ_RT_CPU")) {
        const int from_env = std::atoi(env);
        if (from_env >= 0) {
            return from_env;
        }
    }
    if (cpu_hint == kRtCpuOff) {
        return -1;
    }
    const int ncpu = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
    if (ncpu <= 0) {
        return 0;
    }
    if (cpu_hint == kRtCpuAutoIsolated) {
        return ncpu >= 4 ? ncpu - 2 : ncpu - 1;
    }
    if (cpu_hint == kRtCpuAutoLast) {
        return ncpu - 1;
    }
    return cpu_hint;
}

RtThreadStatus ApplyRtThreadOptions(const RtThreadOptions& opt) {
    RtThreadStatus status{};
    if (g_rt_applied) {
        return status;
    }
    g_rt_applied = true;

    if (prctl(PR_SET_TIMERSLACK, opt.timer_slack_ns) == 0) {
        status.timer_slack_ok = true;
    } else {
        std::fprintf(stderr, "[rt] PR_SET_TIMERSLACK failed: %s\n",
                     std::strerror(errno));
    }

    const int target_cpu = ResolveRtCpu(opt.cpu);
    const bool want_affinity = target_cpu >= 0;
    if (want_affinity) {
        const int ncpu = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
        if (target_cpu < ncpu) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(target_cpu, &cpuset);
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0) {
                status.affinity_ok = true;
                status.bound_cpu = target_cpu;
            } else {
                std::fprintf(stderr, "[rt] pthread_setaffinity_np(cpu=%d) failed: %s\n",
                             target_cpu, std::strerror(errno));
            }
        } else {
            std::fprintf(stderr, "[rt] cpu=%d out of range (ncpu=%d)\n", target_cpu, ncpu);
        }
    } else {
        status.affinity_ok = true;
    }

    if (opt.lock_memory) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
            status.memory_locked = true;
        } else {
            std::fprintf(stderr,
                         "[rt] mlockall failed (try cap_ipc_lock or root): %s\n",
                         std::strerror(errno));
        }
    }

    if (opt.use_fifo) {
        sched_param sp{};
        sp.sched_priority = opt.sched_priority;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0) {
            status.fifo_ok = true;
        } else {
            std::fprintf(stderr,
                         "[rt] SCHED_FIFO prio=%d failed (need root/CAP_SYS_NICE): %s\n",
                         opt.sched_priority, std::strerror(errno));
        }
    } else {
        status.fifo_ok = true;
    }

    std::fprintf(stderr,
                 "[rt] timer_slack=%s cpu=%d affinity=%s mlock=%s fifo=%s prio=%d\n",
                 status.timer_slack_ok ? "ok" : "no",
                 status.bound_cpu,
                 want_affinity ? (status.affinity_ok ? "ok" : "no") : "skip",
                 opt.lock_memory ? (status.memory_locked ? "ok" : "no") : "skip",
                 opt.use_fifo ? (status.fifo_ok ? "ok" : "no") : "skip",
                 opt.use_fifo ? opt.sched_priority : 0);

    if (!opt.abort_on_rt_failure) {
        return status;
    }
    if (want_affinity && !status.affinity_ok) {
        AbortRtSetup("CPU affinity required but not applied");
    }
    if (opt.use_fifo && !status.fifo_ok) {
        AbortRtSetup("SCHED_FIFO required but not granted");
    }
    return status;
}
