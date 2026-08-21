// test_rt_teleop：Pico UDP → 扳机门控 ServoPByPico；500Hz 关节 UDP；50Hz 记录
#include "internal/diag.hpp"
#include "config.hpp"
#include "mv_control.hpp"
#include "periodic_loop.hpp"
#include "recorder.hpp"
#include "run_session.hpp"
#include "rt_thread.hpp"
#include "test_diag.hpp"
#include "udp_io.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#ifndef TJ_DATA_DEFAULT
#define TJ_DATA_DEFAULT "data/test_rt_teleop"
#endif

namespace {

constexpr int kEnableAtCycle = 10;
constexpr int kWaitAfterEnable = kControlCyclesPerSecond;
constexpr int kCycleMax = kControlMaxCycles5Min;
constexpr int64_t kPeriodUsLo = kControlPeriodUsLo;
constexpr int64_t kPeriodUsHi = kControlPeriodUsHi;
constexpr int64_t kWorkStallUs = kControlWorkStallUs;
constexpr int64_t kCatchUpPeriodUs = kControlCatchUpPeriodUs;
constexpr double kRefMovedEps = 1e-6;
constexpr double kM2Mm = 1000.0;
constexpr int kPicoServoHz = 50;
constexpr int kPicoServoPeriodCycles = kControlHz / kPicoServoHz;
constexpr int kRecordHz = 50;
constexpr int kRecordPeriodCycles = kControlHz / kRecordHz;
constexpr float kTriggerPressThreshold = 0.99f;    // 按下门限
constexpr float kTriggerReleaseThreshold = 0.95f;  // 松开迟滞（仅 50Hz 判定）
constexpr uint8_t kPhaseBothTeleop = 44;           // PackPhase: 4*10+4
constexpr int kDisableLowSpdWaitMax = kControlCyclesPerSecond * 2;
constexpr int kDisableRetryMax = 2;
constexpr int kDisableForceDoneCycles = kControlCyclesPerSecond * 3;
constexpr uint32_t kPicoMagic = 0x5049434F;  // "PICO"
constexpr std::size_t kPicoPktSize = 137;
constexpr std::size_t kJointPktSize = 14 * sizeof(double);
constexpr int kPicoStaleMs = 200;

constexpr int kDefaultPicoPort = 30101;
constexpr int kDefaultJointPort = 30100;
const char* kDefaultJointHost = "127.0.0.1";

std::atomic<bool> g_stop_requested{false};
std::atomic<int> g_sigint_count{0};

void OnSigInt(int /*signo*/) {
    g_stop_requested = true;
    g_sigint_count.fetch_add(1);
}

enum class RtPhase : int {
    WaitEnable = 0,
    WaitSettle,
    WaitGoWork,
    Teleop,
    TeardownHome,
    WaitDisable,
    Done,
};

struct ArmFlow {
    const char* name = "";
    RtPhase phase = RtPhase::WaitEnable;
    int wait_cnt = 0;
    double ref_q0[DOF]{};
    bool servo_session_active = false;  // 本臂 ServoPByPico session 是否建立
    int disable_retry = 0;              // WaitDisable 遇 EnableError 重试次数
};

struct PicoState {
    bool valid = false;
    uint32_t seq = 0;
    uint64_t timestamp_ns = 0;
    Pose left_pose{};
    Pose right_pose{};
    double left_pose_m[7]{};   // [x,y,z,qx,qy,qz,qw] 米，与 UDP 一致
    double right_pose_m[7]{};
    float left_trigger = 0.0f;
    float right_trigger = 0.0f;
    std::chrono::steady_clock::time_point recv_time{};
};

struct RtOptions {
    const char* out_dir = TJ_DATA_DEFAULT;
    const char* joint_host = kDefaultJointHost;
    int pico_port = kDefaultPicoPort;
    int joint_port = kDefaultJointPort;
    int sim = -1;  // -1=交互, 0=hw, 1=sim
    bool pico_print = false;
};

bool ParseSimFlag(const char* arg, int& sim) {
    if (std::strcmp(arg, "--sim") == 0) {
        if (sim != -1) {
            return false;
        }
        sim = 1;
        return true;
    }
    if (std::strcmp(arg, "--hw") == 0) {
        if (sim != -1) {
            return false;
        }
        sim = 0;
        return true;
    }
    return false;
}

bool ParseOptions(int argc, char** argv, RtOptions& opt) {
    int pos = 0;
    for (int i = 1; i < argc; ++i) {
        if (ParseSimFlag(argv[i], opt.sim)) {
            continue;
        }
        if (std::strcmp(argv[i], "--pico-port") == 0) {
            if (i + 1 >= argc) {
                return false;
            }
            opt.pico_port = std::atoi(argv[++i]);
            continue;
        }
        if (std::strcmp(argv[i], "--joint-host") == 0) {
            if (i + 1 >= argc) {
                return false;
            }
            opt.joint_host = argv[++i];
            continue;
        }
        if (std::strcmp(argv[i], "--joint-port") == 0) {
            if (i + 1 >= argc) {
                return false;
            }
            opt.joint_port = std::atoi(argv[++i]);
            continue;
        }
        if (std::strcmp(argv[i], "--pico-print") == 0) {
            opt.pico_print = true;
            continue;
        }
        // --model 已废弃：启动时强制交互选择 URDF（615/696）
        if (std::strcmp(argv[i], "--model") == 0) {
            if (i + 1 >= argc) {
                return false;
            }
            ++i;
            std::fprintf(stderr, "[WARN] --model 已忽略，启动时将交互选择 URDF\n");
            continue;
        }
        if (argv[i][0] == '-' || argv[i][0] == '\0') {
            continue;
        }
        if (pos == 0) {
            opt.out_dir = argv[i];
        }
        ++pos;
    }
    return true;
}

bool PromptSimMode(int& sim) {
    while (sim != 0 && sim != 1) {
        std::printf("0=硬件  1=仿真: ");
        if (std::scanf("%d", &sim) != 1) {
            return false;
        }
    }
    return true;
}

// 每次启动强制选择 URDF；返回 "615" / "696"
bool PromptRobotModel(const char*& model_out, std::string& urdf_out) {
    std::string urdf696;
    std::string urdf615;
    if (!ResolveRobotModelUrdf(MV_CONTROL_CONFIG_DEFAULT, "696", urdf696)) {
        std::fprintf(stderr, "[FAIL] 696 URDF 缺失\n");
        return false;
    }
    if (!ResolveRobotModelUrdf(MV_CONTROL_CONFIG_DEFAULT, "615", urdf615)) {
        std::fprintf(stderr, "[FAIL] 615 URDF 缺失\n");
        return false;
    }
    int choice = -1;
    while (choice != 0 && choice != 1) {
        std::printf("选择 URDF 型号（可视化与控制必须一致）:\n");
        std::printf("  0 = 696\n      %s\n", urdf696.c_str());
        std::printf("  1 = 615\n      %s\n", urdf615.c_str());
        std::printf("请选择 [0/1]: ");
        if (std::scanf("%d", &choice) != 1) {
            return false;
        }
    }
    if (choice == 0) {
        model_out = "696";
        urdf_out = std::move(urdf696);
    } else {
        model_out = "615";
        urdf_out = std::move(urdf615);
    }
    return true;
}

Pose MakePoseFromPicoM(double x, double y, double z, double qx, double qy, double qz,
                       double qw) {
    Pose p;
    p.pos = V3d(x, y, z) * kM2Mm;
    p.quat = Quat(qw, qx, qy, qz).normalized();
    return p;
}

bool ParsePicoPacket(const uint8_t* data, std::size_t len, PicoState& out) {
    if (len < kPicoPktSize) {
        return false;
    }
    uint32_t magic = 0;
    std::memcpy(&magic, data, sizeof(magic));
    if (magic != kPicoMagic) {
        return false;
    }

    PicoState pkt;
    std::memcpy(&pkt.seq, data + 4, sizeof(pkt.seq));
    std::memcpy(&pkt.timestamp_ns, data + 8, sizeof(pkt.timestamp_ns));

    double rp[7] = {};
    double lp[7] = {};
    std::memcpy(rp, data + 16, sizeof(rp));
    std::memcpy(lp, data + 16 + 7 * sizeof(double), sizeof(lp));
    std::memcpy(pkt.right_pose_m, rp, sizeof(rp));
    std::memcpy(pkt.left_pose_m, lp, sizeof(lp));
    pkt.right_pose = MakePoseFromPicoM(rp[0], rp[1], rp[2], rp[3], rp[4], rp[5], rp[6]);
    pkt.left_pose = MakePoseFromPicoM(lp[0], lp[1], lp[2], lp[3], lp[4], lp[5], lp[6]);

    std::memcpy(&pkt.right_trigger, data + 128, sizeof(pkt.right_trigger));
    std::memcpy(&pkt.left_trigger, data + 132, sizeof(pkt.left_trigger));
    pkt.valid = true;
    pkt.recv_time = std::chrono::steady_clock::now();
    out = pkt;
    return true;
}

void PollPicoUdp(UdpReceiver& rx, PicoState& state, uint32_t& last_seq) {
    uint8_t buf[512];
    while (true) {
        const ssize_t n = rx.Recv(buf, sizeof(buf));
        if (n <= 0) {
            break;
        }
        PicoState pkt;
        if (!ParsePicoPacket(buf, static_cast<std::size_t>(n), pkt)) {
            continue;
        }
        if (pkt.seq == last_seq) {
            continue;
        }
        last_seq = pkt.seq;
        state = pkt;
    }
}

bool PicoFresh(const PicoState& state) {
    if (!state.valid) {
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now -
                                                                          state.recv_time)
                        .count();
    return ms <= kPicoStaleMs;
}

// 50Hz 打印订阅到的 Pico UDP（与 pico_teleop.csv 字段一致，单位 m + xyzw）
void PrintPicoSubscribed(int cycle, const PicoState& pico, bool fresh) {
    if (!pico.valid) {
        std::fprintf(stderr, "[pico_sub] cycle=%d valid=0 fresh=%d\n", cycle,
                     fresh ? 1 : 0);
        return;
    }
    const double* r = pico.right_pose_m;
    const double* l = pico.left_pose_m;
    std::fprintf(stderr,
                 "[pico_sub] cycle=%d seq=%u ts=%llu valid=1 fresh=%d "
                 "R_trig=%.3f L_trig=%.3f\n",
                 cycle, pico.seq,
                 static_cast<unsigned long long>(pico.timestamp_ns), fresh ? 1 : 0,
                 pico.right_trigger, pico.left_trigger);
    std::fprintf(stderr,
                 "  R[m] x=%.4f y=%.4f z=%.4f qx=%.4f qy=%.4f qz=%.4f qw=%.4f\n", r[0],
                 r[1], r[2], r[3], r[4], r[5], r[6]);
    std::fprintf(stderr,
                 "  L[m] x=%.4f y=%.4f z=%.4f qx=%.4f qy=%.4f qz=%.4f qw=%.4f\n", l[0],
                 l[1], l[2], l[3], l[4], l[5], l[6]);
}

void SnapshotRef(const Robot& arm, ArmFlow& flow) {
    const V7d& q = arm.GetRefState().joint_state.q;
    for (int i = 0; i < DOF; ++i) {
        flow.ref_q0[i] = q(i);
    }
}

bool RefMoved(const Robot& arm, const ArmFlow& flow) {
    const V7d& q = arm.GetRefState().joint_state.q;
    for (int i = 0; i < DOF; ++i) {
        if (std::abs(q(i) - flow.ref_q0[i]) > kRefMovedEps) {
            return true;
        }
    }
    return false;
}

bool MotionFinished(const Robot& arm, const ArmFlow& flow) {
    const StatusCode st = arm.GetStatusCode();
    if (st == StatusCode::Fault) {
        return true;
    }
    return st == StatusCode::Ready && RefMoved(arm, flow);
}

void IssueGoWork(Robot& arm, ArmFlow& flow, int cycle) {
    if (arm.GetErrorCode() != ErrorCode::Normal) {
        arm.ClearError();
    }
    SnapshotRef(arm, flow);
    flow.phase = RtPhase::WaitGoWork;
    std::cerr << ">>> GoWork " << flow.name << " cycle=" << cycle << std::endl;
    arm.GoWork();
}

void IssueGoHome(Robot& arm, ArmFlow& flow, int cycle) {
    if (arm.GetErrorCode() != ErrorCode::Normal) {
        arm.ClearError();
    }
    SnapshotRef(arm, flow);
    flow.wait_cnt = 0;
    flow.phase = RtPhase::TeardownHome;
    std::cerr << ">>> GoHome " << flow.name << " cycle=" << cycle << std::endl;
    arm.GoHome();
}

void IssueDisable(Robot& arm, ArmFlow& flow, int cycle) {
    std::cerr << ">>> " << flow.name << " 下使能 cycle=" << cycle << std::endl;
    if (arm.GetErrorCode() != ErrorCode::Normal) {
        arm.ClearError();
    }
    arm.SetEnable(EnableMode::Disable);
    flow.wait_cnt = 0;
    flow.phase = RtPhase::WaitDisable;
}

void ForceDisableDone(ArmFlow& flow, int cycle, const char* why) {
    std::cerr << ">>> " << flow.name << " 下使能强制结束 cycle=" << cycle
              << " (" << why << ")" << std::endl;
    flow.phase = RtPhase::Done;
}

void StopServoIfNeeded(Robot& arm, ArmFlow& flow) {
    if (flow.servo_session_active) {
        arm.ServoPByPico(Pose{}, false);
        flow.servo_session_active = false;
    }
}

// 迟滞：session 内用 release 门限，避免 50Hz 采样抖动反复 InitPlan
bool TriggerHeldForSession(float trigger, bool session_active) {
    if (session_active) {
        return trigger >= kTriggerReleaseThreshold;
    }
    return trigger >= kTriggerPressThreshold;
}

// 50Hz 统一 session：首次有效 InitPlan，持续有效 RePlan，松开清空
void TickServoArmAt50Hz(Robot& arm, ArmFlow& flow, float trigger, const Pose& pose,
                        bool teleop_active, bool pico_fresh) {
    if (!teleop_active) {
        StopServoIfNeeded(arm, flow);
        return;
    }
    if (!pico_fresh) {
        return;  // 无新 UDP 帧时不误判松开，也不提交新目标
    }
    const bool held = TriggerHeldForSession(trigger, flow.servo_session_active);
    if (held) {
        arm.ServoPByPico(pose, true);
        flow.servo_session_active = true;
        return;
    }
    StopServoIfNeeded(arm, flow);
}

void TickServoAt50Hz(Robot& left, Robot& right, ArmFlow& lf, ArmFlow& rf,
                     const PicoState& pico, bool teleop_active, int cycle) {
    if (cycle % kPicoServoPeriodCycles != 0) {
        return;
    }
    const bool fresh = PicoFresh(pico);
    TickServoArmAt50Hz(right, rf, pico.right_trigger, pico.right_pose, teleop_active,
                       fresh);
    TickServoArmAt50Hz(left, lf, pico.left_trigger, pico.left_pose, teleop_active,
                       fresh);
}

bool PublishJointsUdp(UdpSender& tx, MVControl& ctrl) {
    const V7d& lq = ctrl.Left().GetRefState().joint_state.q;
    const V7d& rq = ctrl.Right().GetRefState().joint_state.q;
    double buf[DOF * 2];
    for (int i = 0; i < DOF; ++i) {
        buf[i] = lq(i);
        buf[i + DOF] = rq(i);
    }
    return tx.Send(buf, kJointPktSize) == static_cast<ssize_t>(kJointPktSize);
}

int PackPhase(const ArmFlow& left, const ArmFlow& right) {
    const auto pack = [](const ArmFlow& f) {
        if (f.phase == RtPhase::WaitGoWork || f.phase == RtPhase::TeardownHome) {
            return 2;
        }
        if (f.phase == RtPhase::Teleop) {
            return 4;
        }
        return static_cast<int>(f.phase);
    };
    return pack(left) * 10 + pack(right);
}

void TickArmAfter(Robot& arm, ArmFlow& flow, int cycle, bool enable_cmd_sent) {
    switch (flow.phase) {
        case RtPhase::WaitEnable:
            if (enable_cmd_sent && arm.GetEnableState() == EnableState::Enabled) {
                std::cerr << ">>> " << flow.name << " 上使能成功 cycle=" << cycle
                          << std::endl;
                flow.wait_cnt = 0;
                flow.phase = RtPhase::WaitSettle;
            }
            break;
        case RtPhase::WaitSettle:
            if (++flow.wait_cnt >= kWaitAfterEnable) {
                IssueGoWork(arm, flow, cycle);
            }
            break;
        case RtPhase::WaitGoWork:
            if (!MotionFinished(arm, flow)) {
                break;
            }
            if (arm.GetStatusCode() == StatusCode::Fault) {
                std::cerr << ">>> GoWork " << flow.name << " fault cycle=" << cycle
                          << std::endl;
            } else {
                std::cerr << ">>> GoWork " << flow.name << " 完成 cycle=" << cycle
                          << std::endl;
                flow.phase = RtPhase::Teleop;
            }
            break;
        case RtPhase::TeardownHome:
            if (!MotionFinished(arm, flow)) {
                break;
            }
            // GoHome 规划结束 ≠ 伺服低速；等 LowSpd 再 Disable，避免 CurState 卡在过渡
            if (!arm.IsStationary() && flow.wait_cnt < kDisableLowSpdWaitMax) {
                if (flow.wait_cnt == 0) {
                    std::cerr << ">>> " << flow.name
                              << " GoHome 完成，等待 LowSpd 后下使能 cycle=" << cycle
                              << std::endl;
                }
                ++flow.wait_cnt;
                break;
            }
            if (!arm.IsStationary()) {
                std::cerr << ">>> " << flow.name
                          << " LowSpd 等待超时，仍尝试下使能 cycle=" << cycle
                          << std::endl;
            }
            flow.wait_cnt = 0;
            IssueDisable(arm, flow, cycle);
            break;
        case RtPhase::WaitDisable:
            if (arm.GetEnableState() == EnableState::Disabled) {
                std::cerr << ">>> " << flow.name << " 下使能成功 cycle=" << cycle
                          << std::endl;
                flow.phase = RtPhase::Done;
                break;
            }
            ++flow.wait_cnt;
            if (arm.GetErrorCode() == ErrorCode::EnableError) {
                if (flow.disable_retry < kDisableRetryMax) {
                    ++flow.disable_retry;
                    std::cerr << ">>> " << flow.name
                              << " 下使能超时，清错并重试 (" << flow.disable_retry
                              << "/" << kDisableRetryMax << ") cycle=" << cycle
                              << std::endl;
                    arm.ClearError();
                    arm.SetEnable(EnableMode::Disable);
                    flow.wait_cnt = 0;
                } else if (flow.wait_cnt >= kDisableForceDoneCycles) {
                    ForceDisableDone(flow, cycle, "EnableError after retries");
                }
                break;
            }
            if (flow.wait_cnt >= kDisableForceDoneCycles) {
                ForceDisableDone(flow, cycle, "WaitDisable watchdog");
            }
            break;
        default:
            break;
    }
}

bool TeleopReady(const ArmFlow& left, const ArmFlow& right) {
    return left.phase == RtPhase::Teleop && right.phase == RtPhase::Teleop;
}

bool AllDone(const ArmFlow& left, const ArmFlow& right) {
    return left.phase == RtPhase::Done && right.phase == RtPhase::Done;
}

void BeginTeardown(MVControl& ctrl, ArmFlow& left, ArmFlow& right, int cycle) {
    StopServoIfNeeded(ctrl.Left(), left);
    StopServoIfNeeded(ctrl.Right(), right);

    auto teardown_arm = [&](Robot& arm, ArmFlow& flow) {
        if (flow.phase == RtPhase::Done || flow.phase == RtPhase::WaitDisable ||
            flow.phase == RtPhase::TeardownHome) {
            return;
        }
        if (flow.phase == RtPhase::Teleop) {
            IssueGoHome(arm, flow, cycle);
        } else {
            IssueDisable(arm, flow, cycle);
        }
    };
    teardown_arm(ctrl.Left(), left);
    teardown_arm(ctrl.Right(), right);
}

struct CycleSectionUs {
    int cycle = -1;
    uint16_t period_us = 0;
    uint16_t wake_late_us = 0;
    uint16_t work_us = 0;
    uint16_t poll_us = 0;
    uint16_t servo_us = 0;
    uint16_t run_us = 0;
    uint16_t udp_tx_us = 0;
    uint16_t record_us = 0;
    uint16_t after_us = 0;
    uint16_t work_overrun_us = 0;
    uint8_t phase_packed = 0;
    uint8_t hw_sent = 0;
    bool valid = false;
};

int64_t MonoNowUs() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000LL +
           static_cast<int64_t>(ts.tv_nsec) / 1000LL;
}

uint16_t ClampU16(int64_t v) {
    if (v <= 0) {
        return 0;
    }
    if (v >= 65535) {
        return 65535;
    }
    return static_cast<uint16_t>(v);
}

bool IsTeleopSampleCycle(int cycle, int teleop_start, int teleop_end) {
    return teleop_start >= 0 && cycle >= teleop_start &&
           (teleop_end < 0 || cycle < teleop_end);
}

void MaybeCollectTeleopPeriod(int cycle, const CycleTiming& timing, int teleop_start,
                              int teleop_end,
                              std::vector<uint16_t>& teleop_periods) {
    if (!IsTeleopSampleCycle(cycle, teleop_start, teleop_end)) {
        return;
    }
    if (timing.period_us <= 0 || timing.period_us > 65535) {
        return;
    }
    teleop_periods.push_back(static_cast<uint16_t>(timing.period_us));
}

void MaybeCollectTeleopOverrun(int cycle, int64_t overrun_us, int teleop_start,
                               int teleop_end, std::vector<uint16_t>& teleop_overruns) {
    if (!IsTeleopSampleCycle(cycle, teleop_start, teleop_end)) {
        return;
    }
    if (overrun_us >= 0 && overrun_us <= 65535) {
        teleop_overruns.push_back(static_cast<uint16_t>(overrun_us));
    }
}

void MaybeCollectTeleopStalls(int cycle, const PeriodStallSample& s, int teleop_start,
                              int teleop_end,
                              std::vector<PeriodStallSample>& teleop_stalls) {
    if (!IsTeleopSampleCycle(cycle, teleop_start, teleop_end)) {
        return;
    }
    teleop_stalls.push_back(s);
}

void FillStallFromSections(PeriodStallSample& out, int cycle, int64_t period_us,
                           int64_t wake_late_us, const char* kind, const char* attr_src,
                           const CycleSectionUs& sec) {
    out.cycle = cycle;
    out.period_us = period_us;
    out.kind = kind;
    out.wake_late_us = wake_late_us;
    out.work_us = sec.work_us;
    out.poll_us = sec.poll_us;
    out.servo_us = sec.servo_us;
    out.run_us = sec.run_us;
    out.udp_tx_us = sec.udp_tx_us;
    out.record_us = sec.record_us;
    out.after_us = sec.after_us;
    out.phase_packed = sec.phase_packed;
    out.hw_sent = sec.hw_sent;
    out.attr_src = attr_src;
}

// Wait 后：只采 period / wake_late；period 越界时用上一拍分段归因
void CollectPeriodAtWait(int cycle, const CycleTiming& timing,
                         const CycleSectionUs& prev_sec,
                         std::vector<uint16_t>& period_samples,
                         std::vector<PeriodStallSample>& stalls, int teleop_start,
                         int teleop_end, std::vector<uint16_t>& teleop_periods,
                         std::vector<PeriodStallSample>& teleop_stalls) {
    if (timing.period_us <= 0 || timing.period_us > 65535) {
        return;
    }
    period_samples.push_back(static_cast<uint16_t>(timing.period_us));
    MaybeCollectTeleopPeriod(cycle, timing, teleop_start, teleop_end, teleop_periods);
    if (timing.period_us >= kPeriodUsLo && timing.period_us <= kPeriodUsHi) {
        return;
    }
    PeriodStallSample s{};
    if (timing.period_us < kCatchUpPeriodUs) {
        FillStallFromSections(s, cycle, timing.period_us, timing.wake_late_us, "catch_up",
                              prev_sec.valid ? "prev" : "none", prev_sec);
    } else if (timing.period_us < kPeriodUsLo) {
        FillStallFromSections(s, cycle, timing.period_us, timing.wake_late_us, "period_short",
                              prev_sec.valid ? "prev" : "none", prev_sec);
    } else if (timing.period_us > kPeriodUsHi &&
               prev_sec.valid && prev_sec.work_overrun_us > 0) {
        FillStallFromSections(s, cycle, timing.period_us, timing.wake_late_us, "work_over",
                              "prev", prev_sec);
    } else if (timing.period_us > kPeriodUsHi &&
               prev_sec.valid && prev_sec.work_us <= kWorkStallUs &&
               timing.wake_late_us >= 200) {
        FillStallFromSections(s, cycle, timing.period_us, timing.wake_late_us, "sched_gap",
                              "prev", prev_sec);
    } else if (timing.period_us > kPeriodUsHi) {
        FillStallFromSections(s, cycle, timing.period_us, timing.wake_late_us, "period_long",
                              prev_sec.valid ? "prev" : "none", prev_sec);
    } else {
        FillStallFromSections(s, cycle, timing.period_us, timing.wake_late_us, "jitter",
                              prev_sec.valid ? "prev" : "none", prev_sec);
    }
    stalls.push_back(s);
    MaybeCollectTeleopStalls(cycle, s, teleop_start, teleop_end, teleop_stalls);
}

// 整拍工作结束后：记录 work overrun（work_over 归因在分段填完后另记）
void CollectOverrunAtWorkEnd(const CycleTiming& timing, PeriodicLoop& tick,
                             CycleSectionUs& sec, std::vector<uint16_t>& overrun_samples) {
    const int64_t work_overrun_us = tick.MeasureWorkOverrunUs();
    sec.work_overrun_us = ClampU16(work_overrun_us);
    const int64_t overrun_us = std::max(timing.wake_late_us, work_overrun_us);
    if (overrun_us >= 0 && overrun_us <= 65535) {
        overrun_samples.push_back(static_cast<uint16_t>(overrun_us));
    }
}

void FillPicoSample(PicoSample& s, int cycle, const PicoState& pico, bool fresh) {
    s.cycle = static_cast<uint32_t>(cycle);
    s.timestamp_ns = pico.timestamp_ns;
    s.seq = pico.seq;
    s.valid = pico.valid ? 1 : 0;
    s.fresh = fresh ? 1 : 0;
    s.left_trigger = pico.left_trigger;
    s.right_trigger = pico.right_trigger;
    for (int i = 0; i < 7; ++i) {
        s.right_pose[i] = pico.right_pose_m[i];
        s.left_pose[i] = pico.left_pose_m[i];
    }
}

bool PushRecordSample(int cycle, MVControl& ctrl, const PicoState& pico, bool pico_fresh,
                      uint8_t phase_packed, uint8_t flags, const CycleTiming& timing,
                      int64_t overrun_us, RunRecorder& recorder,
                      PicoRecorder& pico_recorder) {
    CycleSample sample{};
    FillCycleSampleFromControl(sample, ctrl, cycle, phase_packed, flags);
    if (timing.period_us > 0 && timing.period_us <= 65535) {
        sample.period_us = static_cast<uint16_t>(timing.period_us);
    }
    if (overrun_us >= 0 && overrun_us <= 65535) {
        sample.overrun_us = static_cast<uint16_t>(overrun_us);
    }
    if (!recorder.Push(sample)) {
        return false;
    }
    PicoSample ps{};
    FillPicoSample(ps, cycle, pico, pico_fresh);
    return pico_recorder.Push(ps);
}

bool WriteRunMeta(const char* dir, int pico_port, int joint_port,
                  std::size_t robot_samples, std::size_t pico_samples,
                  const char* model, const char* urdf) {
    const std::string path = std::string(dir) + "/run_meta.txt";
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f, "servo_hz=%d\n", kPicoServoHz);
    std::fprintf(f, "record_hz=%d\n", kRecordHz);
    std::fprintf(f, "joint_udp_hz=%d\n", kControlHz);
    std::fprintf(f, "model=%s\n", model != nullptr ? model : "");
    std::fprintf(f, "urdf=%s\n", urdf != nullptr ? urdf : "");
    std::fprintf(f, "pico_udp_port=%d\n", pico_port);
    std::fprintf(f, "joint_udp_port=%d\n", joint_port);
    std::fprintf(f, "robot_samples=%zu\n", robot_samples);
    std::fprintf(f, "pico_samples=%zu\n", pico_samples);
    std::fclose(f);
    return true;
}

struct DiagCapture {
    DiagEventRecorder* recorder = nullptr;
    int* cycle = nullptr;
};

void OnDiagEvent(const DiagEvent& e, void* user) {
    auto* ctx = static_cast<DiagCapture*>(user);
    if (ctx == nullptr || ctx->recorder == nullptr || ctx->cycle == nullptr) {
        return;
    }
    ctx->recorder->Push(static_cast<uint32_t>(*ctx->cycle), e.arm, e.category, e.motion,
                        e.reason, e.code, e.x, e.y, e.z, e.extra0, e.extra1, e.fk_x,
                        e.fk_y, e.fk_z);
}

}  // namespace

int main(int argc, char** argv) {
    RtOptions opt;
    if (!ParseOptions(argc, argv, opt)) {
        std::fprintf(stderr, "用法: %s [out_dir] [--sim|--hw] "
                             "[--pico-port N] [--joint-host H] [--joint-port N] "
                             "[--pico-print]\n"
                             "启动后交互选择硬件/仿真与 URDF 型号(615/696)。\n",
                     argv[0]);
        return 1;
    }

    if (opt.sim == -1 && !PromptSimMode(opt.sim)) {
        return 1;
    }
    const bool is_sim = (opt.sim == 1);

    const char* model = nullptr;
    std::string model_urdf;
    if (!PromptRobotModel(model, model_urdf)) {
        return 1;
    }
    const char* urdf_override = model_urdf.c_str();

    std::printf("[start] %s  model=%s  out=%s  pico_udp=:%d  joint_udp=%s:%d\n",
                is_sim ? "仿真" : "硬件", model, opt.out_dir, opt.pico_port,
                opt.joint_host, opt.joint_port);
    std::printf("[start] urdf=%s\n", urdf_override);

    MVControl ctrl;
    if (!ctrl.Init(MV_CONTROL_CONFIG_DEFAULT, is_sim, nullptr, urdf_override)) {
        std::fprintf(stderr, "[FAIL] Init (%s)\n", is_sim ? "sim" : "hw");
        return 2;
    }
    PrintInitJointStates(ctrl);

    std::printf("[teleop] control_mode=Position\n");

    int go = -1;
    while (go != 0 && go != 1) {
        std::printf("0=退出  1=进入500Hz循环: ");
        if (std::scanf("%d", &go) != 1) {
            return 1;
        }
    }
    if (go == 0) {
        std::printf("[exit] 未进入循环\n");
        return 0;
    }

    signal(SIGINT, OnSigInt);

    UdpReceiver pico_rx;
    if (!pico_rx.Open(opt.pico_port)) {
        std::fprintf(stderr, "[FAIL] Pico UDP bind :%d\n", opt.pico_port);
        return 2;
    }
    UdpSender joint_tx;
    if (!joint_tx.Open() || !joint_tx.SetDestination(opt.joint_host, opt.joint_port)) {
        std::fprintf(stderr, "[FAIL] joint UDP -> %s:%d\n", opt.joint_host,
                     opt.joint_port);
        return 2;
    }

    ArmFlow left_flow{"left"};
    ArmFlow right_flow{"right"};
    PicoState pico{};
    uint32_t last_pico_seq = 0;
    bool enable_cmd_sent = false;
    bool teardown_started = false;
    int cycle = 0;
    bool loop_error = false;
    bool interrupted = false;
    int teleop_start_cycle = -1;
    int teleop_end_cycle = -1;
    std::vector<uint16_t> period_samples;
    std::vector<uint16_t> overrun_samples;
    std::vector<uint16_t> teleop_period_samples;
    std::vector<uint16_t> teleop_overrun_samples;
    std::vector<PeriodStallSample> stalls;
    std::vector<PeriodStallSample> teleop_stalls;
    period_samples.reserve(kCycleMax);
    overrun_samples.reserve(kCycleMax);
    teleop_period_samples.reserve(kCycleMax);
    teleop_overrun_samples.reserve(kCycleMax);
    stalls.reserve(256);
    teleop_stalls.reserve(64);
    CycleSectionUs prev_sec{};
    CycleSectionUs section_ring[64]{};
    std::size_t section_ring_i = 0;

    RunRecorder recorder;
    recorder.Reserve(static_cast<std::size_t>(kCycleMax / kRecordPeriodCycles));
    PicoRecorder pico_recorder;
    pico_recorder.Reserve(static_cast<std::size_t>(kCycleMax / kRecordPeriodCycles));
    DiagEventRecorder diag_recorder;
    diag_recorder.Reserve(50000);
    DiagCapture diag_ctx{&diag_recorder, &cycle};
    MvDiag::SetDiagEventCallback(OnDiagEvent, &diag_ctx);
    MvDiag::ServoPicoTraceReset();

    PeriodicLoop tick(kControlPeriodUs);
    // sim/hw 统一 HardRt：FIFO + 绑核；勿 mlock（大 recorder 会恶化抖动）
    const RtThreadOptions rt_opt = MakeTeleopRtSessionOptions().rt_thread;
    ApplyRtThreadOptions(rt_opt);
    tick.ResetAnchor();
    ctrl.ResetHwRunStats();
    uint64_t prev_clear_fail = 0;

    std::printf("\n======== 500Hz RT teleop Position (GoWork→ServoPByPico→GoHome) ========\n");
    try {
        // Ctrl+C 只置位；收尾中须继续 500Hz 直到 GoHome→下使能完成，不能因
        // g_stop_requested 立刻退出（否则 BeginTeardown 后下一圈 while 直接跳出）。
        while (!loop_error) {
            const CycleTiming timing = tick.WaitCycleStart();
            CollectPeriodAtWait(cycle, timing, prev_sec, period_samples, stalls,
                                teleop_start_cycle, teleop_end_cycle, teleop_period_samples,
                                teleop_stalls);

            if (TeleopReady(left_flow, right_flow)) {
                if (teleop_start_cycle < 0) {
                    teleop_start_cycle = cycle;
                }
            }

            if (cycle == kEnableAtCycle) {
                // 位置跟随：上使能即 Position（CurState=1）；每拍 OnSetJointCmdPos
                std::cerr << ">>> SetControlMode(Position) cycle=" << cycle << std::endl;
                if (!ctrl.Left().SetControlMode(ControlMode::Position) ||
                    !ctrl.Right().SetControlMode(ControlMode::Position)) {
                    std::fprintf(stderr,
                                 "[FAIL] SetControlMode(Position) left_err=%d right_err=%d\n",
                                 static_cast<int>(ctrl.Left().GetErrorCode()),
                                 static_cast<int>(ctrl.Right().GetErrorCode()));
                    loop_error = true;
                    break;
                }
                std::cerr << ">>> 双臂上使能 cycle=" << cycle << std::endl;
                ctrl.Left().SetEnable(EnableMode::Enable);
                ctrl.Right().SetEnable(EnableMode::Enable);
                enable_cmd_sent = true;
            }

            if (g_stop_requested && !teardown_started) {
                teleop_end_cycle = cycle;
                BeginTeardown(ctrl, left_flow, right_flow, cycle);
                teardown_started = true;
            }
            // 第二次 Ctrl+C：强制结束收尾，避免 WaitDisable 卡死
            if (teardown_started && g_sigint_count.load() >= 2) {
                std::fprintf(stderr,
                             "[INTERRUPT] second Ctrl+C → force Done @ cycle=%d\n",
                             cycle);
                left_flow.phase = RtPhase::Done;
                right_flow.phase = RtPhase::Done;
                loop_error = true;
                break;
            }

            CycleSectionUs sec{};
            sec.cycle = cycle;
            sec.period_us = ClampU16(timing.period_us);
            sec.wake_late_us = ClampU16(timing.wake_late_us);

            const int64_t t0 = MonoNowUs();
            PollPicoUdp(pico_rx, pico, last_pico_seq);
            const int64_t t1 = MonoNowUs();

            const bool teleop_active = TeleopReady(left_flow, right_flow);
            TickServoAt50Hz(ctrl.Left(), ctrl.Right(), left_flow, right_flow, pico,
                            teleop_active, cycle);
            const int64_t t2 = MonoNowUs();

            ctrl.Run();
            const int64_t t3 = MonoNowUs();

            if (!PublishJointsUdp(joint_tx, ctrl)) {
                // 可视化未启动时允许 sendto 失败，不中断控制环
            }
            const int64_t t4 = MonoNowUs();

            const MVControl::HwRunStats& stats = ctrl.LastHwRunStats();
            uint8_t flags = 0;
            if (stats.sent_this_cycle) {
                flags |= kSampleHwSent;
            }
            if (stats.send_clear_fail_total > prev_clear_fail) {
                flags |= kSampleClearFail;
                prev_clear_fail = stats.send_clear_fail_total;
            }
            sec.hw_sent = stats.sent_this_cycle ? 1 : 0;

            TickArmAfter(ctrl.Left(), left_flow, cycle, enable_cmd_sent);
            TickArmAfter(ctrl.Right(), right_flow, cycle, enable_cmd_sent);
            const int64_t t5 = MonoNowUs();

            sec.poll_us = ClampU16(t1 - t0);
            sec.servo_us = ClampU16(t2 - t1);
            sec.run_us = ClampU16(t3 - t2);
            sec.udp_tx_us = ClampU16(t4 - t3);
            sec.after_us = ClampU16(t5 - t4);
            sec.phase_packed =
                static_cast<uint8_t>(PackPhase(left_flow, right_flow));

            // 在 record 前测 overrun（50Hz Push 通常 ≪1ms；权威 overrun 统计在此）
            CollectOverrunAtWorkEnd(timing, tick, sec, overrun_samples);
            const int64_t overrun_us = std::max(
                timing.wake_late_us, static_cast<int64_t>(sec.work_overrun_us));
            MaybeCollectTeleopOverrun(cycle, overrun_us, teleop_start_cycle, teleop_end_cycle,
                                      teleop_overrun_samples);

            if (cycle % kRecordPeriodCycles == 0) {
                const int64_t t_record0 = MonoNowUs();
                const bool pico_fresh = PicoFresh(pico);
                if (opt.pico_print) {
                    PrintPicoSubscribed(cycle, pico, pico_fresh);
                }
                if (!PushRecordSample(cycle, ctrl, pico, pico_fresh, sec.phase_packed,
                                      flags, timing, overrun_us, recorder,
                                      pico_recorder)) {
                    std::fprintf(stderr, "[FAIL] recorder full at cycle %d\n", cycle);
                    loop_error = true;
                    break;
                }
                sec.record_us = ClampU16(MonoNowUs() - t_record0);
            }
            sec.work_us = ClampU16(MonoNowUs() - t0);
            sec.valid = true;

            if (sec.work_us > kWorkStallUs) {
                PeriodStallSample s{};
                FillStallFromSections(s, cycle, timing.period_us, timing.wake_late_us,
                                      "work_over", "curr", sec);
                stalls.push_back(s);
                MaybeCollectTeleopStalls(cycle, s, teleop_start_cycle, teleop_end_cycle,
                                         teleop_stalls);
            }

            section_ring[section_ring_i] = sec;
            section_ring_i = (section_ring_i + 1) % 64;
            prev_sec = sec;

            if (teardown_started && AllDone(left_flow, right_flow)) {
                break;
            }

            if (++cycle > kCycleMax) {
                std::fprintf(stderr, "[FAIL] timeout cycle>%d\n", kCycleMax);
                loop_error = true;
                break;
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[exception] %s\n", e.what());
        loop_error = true;
    } catch (...) {
        std::fprintf(stderr, "[exception] unknown\n");
        loop_error = true;
    }

    MvDiag::ClearDiagEventCallback();

    interrupted = g_stop_requested.load();
    // 异常/超时跳出时若收尾未完成，继续跑完 GoHome→下使能
    if (!AllDone(left_flow, right_flow)) {
        if (!teardown_started) {
            teleop_end_cycle = cycle;
            BeginTeardown(ctrl, left_flow, right_flow, cycle);
            teardown_started = true;
        }
        for (int i = 0; i < 120000 && !AllDone(left_flow, right_flow); ++i) {
            if (g_sigint_count.load() >= 2) {
                std::fprintf(stderr, "[INTERRUPT] second Ctrl+C in teardown drain\n");
                left_flow.phase = RtPhase::Done;
                right_flow.phase = RtPhase::Done;
                break;
            }
            const CycleTiming timing = tick.WaitCycleStart();
            CollectPeriodAtWait(cycle, timing, prev_sec, period_samples, stalls,
                                teleop_start_cycle, teleop_end_cycle, teleop_period_samples,
                                teleop_stalls);

            CycleSectionUs sec{};
            sec.cycle = cycle;
            sec.period_us = ClampU16(timing.period_us);
            sec.wake_late_us = ClampU16(timing.wake_late_us);
            const int64_t t0 = MonoNowUs();
            ctrl.Run();
            const int64_t t1 = MonoNowUs();
            TickArmAfter(ctrl.Left(), left_flow, cycle, enable_cmd_sent);
            TickArmAfter(ctrl.Right(), right_flow, cycle, enable_cmd_sent);
            const int64_t t2 = MonoNowUs();
            sec.run_us = ClampU16(t1 - t0);
            sec.after_us = ClampU16(t2 - t1);
            sec.phase_packed =
                static_cast<uint8_t>(PackPhase(left_flow, right_flow));

            CollectOverrunAtWorkEnd(timing, tick, sec, overrun_samples);
            const int64_t overrun_us = std::max(
                timing.wake_late_us, static_cast<int64_t>(sec.work_overrun_us));
            MaybeCollectTeleopOverrun(cycle, overrun_us, teleop_start_cycle, teleop_end_cycle,
                                      teleop_overrun_samples);

            if (cycle % kRecordPeriodCycles == 0) {
                const int64_t t_record0 = MonoNowUs();
                const bool pico_fresh = PicoFresh(pico);
                PushRecordSample(cycle, ctrl, pico, pico_fresh, sec.phase_packed, 0,
                                 timing, overrun_us, recorder, pico_recorder);
                sec.record_us = ClampU16(MonoNowUs() - t_record0);
            }
            sec.work_us = ClampU16(MonoNowUs() - t0);
            sec.valid = true;
            if (sec.work_us > kWorkStallUs) {
                PeriodStallSample s{};
                FillStallFromSections(s, cycle, timing.period_us, timing.wake_late_us,
                                      "work_over", "curr", sec);
                stalls.push_back(s);
                MaybeCollectTeleopStalls(cycle, s, teleop_start_cycle, teleop_end_cycle,
                                         teleop_stalls);
            }
            section_ring[section_ring_i] = sec;
            section_ring_i = (section_ring_i + 1) % 64;
            prev_sec = sec;
            ++cycle;
        }
    }

    if (!ExportSessionCsv(recorder, opt.out_dir, true)) {
        std::fprintf(stderr, "[FAIL] export robot csv to %s (samples=%zu)\n", opt.out_dir,
                     recorder.Size());
        return 1;
    }
    if (!ExportPicoCsv(pico_recorder, opt.out_dir)) {
        std::fprintf(stderr, "[WARN] export pico_teleop.csv failed or empty (samples=%zu)\n",
                     pico_recorder.Size());
    }
    if (!WriteRunMeta(opt.out_dir, opt.pico_port, opt.joint_port, recorder.Size(),
                      pico_recorder.Size(), model, urdf_override)) {
        std::fprintf(stderr, "[WARN] write run_meta.txt failed\n");
    }

    const std::string period_summary_path =
        std::string(opt.out_dir) + "/period_summary.txt";
    const std::string period_abnormal_path =
        std::string(opt.out_dir) + "/period_abnormal.csv";
    if (!period_samples.empty()) {
        SavePeriodSummary(period_summary_path.c_str(), period_samples, overrun_samples,
                          kControlPeriodUs);
        // 归因说明追加到 summary
        FILE* sf = std::fopen(period_summary_path.c_str(), "a");
        if (sf != nullptr) {
            std::fprintf(sf,
                         "note: period 越界行 attr_src=prev（拖长 period 的是上一拍 work）；"
                         "work_over 为 curr；sched_gap 表示 prev.work 正常但 wake_late/period "
                         "很大（调度间隙）。\n");
            std::fclose(sf);
        }
    }
    SavePeriodAbnormalCsv(period_abnormal_path.c_str(), stalls);
    ExportDiagEventsCsv(diag_recorder, opt.out_dir);
    ExportDiagSummary(diag_recorder, opt.out_dir);
    ExportTeleopDiagReport(pico_recorder, opt.out_dir);

    PrintPeriodSummaryBrief(period_samples, overrun_samples, kControlPeriodUs,
                            stalls.size());
    const PeriodVerdict verdict = EvaluatePeriodVerdict(
        teleop_period_samples, teleop_overrun_samples, teleop_stalls, period_samples,
        overrun_samples, stalls.size());
    PrintPeriodVerdict(verdict, kControlPeriodUs);
    PrintMaxStallBrief(stalls);

    if (interrupted) {
        std::printf("\n[INTERRUPT] Ctrl+C @ cycle=%d, csv saved → %s\n", cycle,
                    opt.out_dir);
    } else {
        std::printf("\n[%s] cycles=%d  robot_samples=%zu  pico_samples=%zu  saved → %s\n",
                    loop_error ? "FAIL" : "PASS", cycle, recorder.Size(),
                    pico_recorder.Size(), opt.out_dir);
    }
    std::printf("诊断落盘: period_summary.txt period_abnormal.csv diag_events.csv "
                "diag_summary.txt teleop_diag.txt\n");
    return (loop_error && !interrupted) ? 3 : 0;
}
