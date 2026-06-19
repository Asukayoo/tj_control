#include "diag.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace MvDiag {

bool Verbose() {
    static int cached = -1;
    if (cached < 0) {
        const char* env = std::getenv("MV_DIAG");
        cached = (env != nullptr && env[0] != '0') ? 1 : 0;
    }
    return cached != 0;
}

namespace {

void LogImpl(const char* tag, int arm_serial, const char* fmt, va_list ap) {
    if (arm_serial < 0) {
        std::fprintf(stderr, "[mv_%s] ", tag);
    } else {
        std::fprintf(stderr, "[mv_%s] arm=%d ", tag, arm_serial);
    }
    std::vfprintf(stderr, fmt, ap);
    std::fprintf(stderr, "\n");
}

}  // namespace

void LogEnable(int arm_serial, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    LogImpl("enable", arm_serial, fmt, ap);
    va_end(ap);
}

void LogMotion(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    LogImpl("movj", -1, fmt, ap);
    va_end(ap);
}

void LogVerbose(int arm_serial, const char* fmt, ...) {
    if (!Verbose()) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    LogImpl("diag", arm_serial, fmt, ap);
    va_end(ap);
}

namespace {

DiagEventCallback g_event_cb = nullptr;
void* g_event_user = nullptr;

void EmitOrPrintIk(const DiagEvent& e) {
    if (g_event_cb != nullptr) {
        g_event_cb(e, g_event_user);
        return;
    }
    if (std::strcmp(e.reason, "kdl_nr") == 0) {
        std::printf(
            "[IKError] arm=%d reason=kdl_nr ret=%d target=[%.2f,%.2f,%.2f] mm\n", e.arm,
            e.code, e.x, e.y, e.z);
    } else if (std::strcmp(e.reason, "pose_tol") == 0) {
        std::printf(
            "[IKError] arm=%d reason=pose_tol pos_err=%.2f mm ori_err=%.3f rad "
            "target=[%.2f,%.2f,%.2f] fk=[%.2f,%.2f,%.2f]\n",
            e.arm, e.extra0, e.extra1, e.x, e.y, e.z, e.fk_x, e.fk_y, e.fk_z);
    } else if (std::strcmp(e.reason, "FK_verify") == 0) {
        std::printf("[IKError] arm=%d reason=FK_verify target_pos=[%.2f,%.2f,%.2f] mm\n",
                    e.arm, e.x, e.y, e.z);
    }
    std::fflush(stdout);
}

}  // namespace

void SetDiagEventCallback(DiagEventCallback cb, void* user) {
    g_event_cb = cb;
    g_event_user = user;
}

void ClearDiagEventCallback() {
    g_event_cb = nullptr;
    g_event_user = nullptr;
}

void EmitIkError(int arm, const char* reason, int code, double x, double y, double z,
                 double extra0, double extra1, double fk_x, double fk_y, double fk_z) {
    DiagEvent e{};
    e.arm = arm;
    e.category = "ik";
    e.reason = reason;
    e.code = code;
    e.x = x;
    e.y = y;
    e.z = z;
    e.extra0 = extra0;
    e.extra1 = extra1;
    e.fk_x = fk_x;
    e.fk_y = fk_y;
    e.fk_z = fk_z;
    EmitOrPrintIk(e);
}

void EmitMotionError(int arm, const char* motion, const char* reason) {
    if (g_event_cb != nullptr) {
        DiagEvent e{};
        e.arm = arm;
        e.category = "motion";
        e.motion = motion;
        e.reason = reason;
        e.code = 0;
        g_event_cb(e, g_event_user);
        return;
    }
    std::printf("[IKError] arm=%d motion=%s reason=%s\n", arm, motion, reason);
    std::fflush(stdout);
}

ServoPicoTrace& ServoPicoTraceGet() {
    static ServoPicoTrace trace;
    return trace;
}

void ServoPicoTraceReset() {
    ServoPicoTraceGet() = ServoPicoTrace{};
}

void ServoPicoTraceExport(FILE* f) {
    if (f == nullptr) {
        return;
    }
    const ServoPicoTrace& t = ServoPicoTraceGet();
    std::fprintf(f, "\n[servo_pico_trace]\n");
    for (int i = 0; i < 2; ++i) {
        const ServoPicoArmTrace& a = t.arm[i];
        std::fprintf(f, "arm%d api_enter=%llu api_reject=%llu stream_submit=%llu\n", i,
                     static_cast<unsigned long long>(a.api_enter),
                     static_cast<unsigned long long>(a.api_reject),
                     static_cast<unsigned long long>(a.stream_submit));
        std::fprintf(f,
                     "arm%d motion_init=%llu session_init=%llu session_fk_fail=%llu "
                     "replan=%llu\n",
                     i, static_cast<unsigned long long>(a.motion_init),
                     static_cast<unsigned long long>(a.session_init),
                     static_cast<unsigned long long>(a.session_fk_fail),
                     static_cast<unsigned long long>(a.replan));
        std::fprintf(f,
                     "arm%d run_session=%llu run_skip_session=%llu servo_run=%llu "
                     "servo_skip_init=%llu\n",
                     i, static_cast<unsigned long long>(a.run_session),
                     static_cast<unsigned long long>(a.run_skip_session),
                     static_cast<unsigned long long>(a.servo_run),
                     static_cast<unsigned long long>(a.servo_skip_init));
        std::fprintf(f, "arm%d ik_ok=%llu ik_fail=%llu\n", i,
                     static_cast<unsigned long long>(a.ik_ok),
                     static_cast<unsigned long long>(a.ik_fail));
    }
}

}  // namespace MvDiag
