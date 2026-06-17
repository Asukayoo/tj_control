// test_rt_teleop：Pico UDP → 扳机门控 ServoPByPico；1kHz 关节 UDP；50Hz 记录
#include "internal/diag.hpp"
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
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#ifndef TJ_DATA_DEFAULT
#define TJ_DATA_DEFAULT "data/test_rt_teleop"
#endif

namespace {

constexpr int kEnableAtCycle = 10;
constexpr int kWaitAfterEnable = 1000;
constexpr int kCycleMax = 300000;
constexpr int64_t kPeriodUsLo = 900;
constexpr int64_t kPeriodUsHi = 1100;
constexpr double kRefMovedEps = 1e-6;
constexpr double kM2Mm = 1000.0;
constexpr int kPicoServoHz = 50;
constexpr int kPicoServoPeriodCycles = 1000 / kPicoServoHz;
constexpr int kRecordHz = 50;
constexpr int kRecordPeriodCycles = 1000 / kRecordHz;
constexpr float kTriggerThreshold = 0.99f;
constexpr uint32_t kPicoMagic = 0x5049434F;  // "PICO"
constexpr std::size_t kPicoPktSize = 137;
constexpr std::size_t kJointPktSize = 14 * sizeof(double);
constexpr int kPicoStaleMs = 200;

constexpr int kDefaultPicoPort = 30101;
constexpr int kDefaultJointPort = 30100;
const char* kDefaultJointHost = "127.0.0.1";

std::atomic<bool> g_stop_requested{false};

void OnSigInt(int /*signo*/) { g_stop_requested = true; }

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
    bool trigger_was_pressed = false;
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
    flow.phase = RtPhase::TeardownHome;
    std::cerr << ">>> GoHome " << flow.name << " cycle=" << cycle << std::endl;
    arm.GoHome();
}

void IssueDisable(Robot& arm, ArmFlow& flow, int cycle) {
    std::cerr << ">>> " << flow.name << " 下使能 cycle=" << cycle << std::endl;
    arm.SetEnable(EnableMode::Disable);
    flow.phase = RtPhase::WaitDisable;
}

void StopServoIfNeeded(Robot& arm, ArmFlow& flow) {
    if (flow.trigger_was_pressed) {
        arm.ServoPByPico(Pose{}, false);
        flow.trigger_was_pressed = false;
    }
}

void TickTriggerRelease(Robot& arm, ArmFlow& flow, float trigger, bool teleop_active) {
    if (!flow.trigger_was_pressed) {
        return;
    }
    const bool pressed = teleop_active && (trigger >= kTriggerThreshold);
    if (!pressed) {
        arm.ServoPByPico(Pose{}, false);
        flow.trigger_was_pressed = false;
    }
}

void TickServoAt50Hz(Robot& left, Robot& right, ArmFlow& lf, ArmFlow& rf,
                     const PicoState& pico, bool teleop_active, int cycle) {
    if (cycle % kPicoServoPeriodCycles != 0) {
        return;
    }
    if (!teleop_active || !PicoFresh(pico)) {
        return;
    }
    if (pico.right_trigger >= kTriggerThreshold) {
        right.ServoPByPico(pico.right_pose, true);
        rf.trigger_was_pressed = true;
    }
    if (pico.left_trigger >= kTriggerThreshold) {
        left.ServoPByPico(pico.left_pose, true);
        lf.trigger_was_pressed = true;
    }
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
            IssueDisable(arm, flow, cycle);
            break;
        case RtPhase::WaitDisable:
            if (arm.GetEnableState() == EnableState::Disabled) {
                std::cerr << ">>> " << flow.name << " 下使能成功 cycle=" << cycle
                          << std::endl;
                flow.phase = RtPhase::Done;
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

void CollectCycleTiming(int cycle, const CycleTiming& timing, PeriodicLoop& tick,
                        std::vector<uint16_t>& period_samples,
                        std::vector<uint16_t>& overrun_samples,
                        std::vector<std::pair<int, int64_t>>& abnormal_periods) {
    if (timing.period_us > 0 && timing.period_us <= 65535) {
        period_samples.push_back(static_cast<uint16_t>(timing.period_us));
        if (timing.period_us < kPeriodUsLo || timing.period_us > kPeriodUsHi) {
            abnormal_periods.emplace_back(cycle, timing.period_us);
        }
    }
    const int64_t work_overrun_us = tick.MeasureWorkOverrunUs();
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
                      PeriodicLoop& tick, RunRecorder& recorder,
                      PicoRecorder& pico_recorder) {
    CycleSample sample{};
    FillCycleSampleFromControl(sample, ctrl, cycle, phase_packed, flags);
    if (timing.period_us > 0 && timing.period_us <= 65535) {
        sample.period_us = static_cast<uint16_t>(timing.period_us);
    }
    const int64_t work_overrun_us = tick.MeasureWorkOverrunUs();
    const int64_t overrun_us = std::max(timing.wake_late_us, work_overrun_us);
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
                  std::size_t robot_samples, std::size_t pico_samples) {
    const std::string path = std::string(dir) + "/run_meta.txt";
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f, "servo_hz=%d\n", kPicoServoHz);
    std::fprintf(f, "record_hz=%d\n", kRecordHz);
    std::fprintf(f, "joint_udp_hz=1000\n");
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
                             "[--pico-port N] [--joint-host H] [--joint-port N]\n",
                     argv[0]);
        return 1;
    }

    if (opt.sim == -1 && !PromptSimMode(opt.sim)) {
        return 1;
    }
    const bool is_sim = (opt.sim == 1);
    std::printf("[start] %s  out=%s  pico_udp=:%d  joint_udp=%s:%d\n",
                is_sim ? "仿真" : "硬件", opt.out_dir, opt.pico_port, opt.joint_host,
                opt.joint_port);

    MVControl ctrl;
    if (!ctrl.Init(MV_CONTROL_CONFIG_DEFAULT, is_sim)) {
        std::fprintf(stderr, "[FAIL] Init (%s)\n", is_sim ? "sim" : "hw");
        return 2;
    }
    PrintInitJointStates(ctrl);

    int go = -1;
    while (go != 0 && go != 1) {
        std::printf("0=退出  1=进入1kHz循环: ");
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
    std::vector<uint16_t> period_samples;
    std::vector<uint16_t> overrun_samples;
    std::vector<std::pair<int, int64_t>> abnormal_periods;
    period_samples.reserve(kCycleMax);
    overrun_samples.reserve(kCycleMax);
    abnormal_periods.reserve(64);

    RunRecorder recorder;
    recorder.Reserve(static_cast<std::size_t>(kCycleMax / kRecordPeriodCycles));
    PicoRecorder pico_recorder;
    pico_recorder.Reserve(static_cast<std::size_t>(kCycleMax / kRecordPeriodCycles));
    DiagEventRecorder diag_recorder;
    diag_recorder.Reserve(50000);
    DiagCapture diag_ctx{&diag_recorder, &cycle};
    MvDiag::SetDiagEventCallback(OnDiagEvent, &diag_ctx);

    PeriodicLoop tick(1000);
    const RtThreadOptions rt_opt =
        is_sim ? MakeDefaultSessionOptions(1000).rt_thread
               : MakeHardRtSessionOptions(1000).rt_thread;
    ApplyRtThreadOptions(rt_opt);
    tick.ResetAnchor();
    ctrl.ResetHwRunStats();
    uint64_t prev_clear_fail = 0;

    std::printf("\n======== 1kHz RT teleop (GoWork→ServoPByPico→GoHome) ========\n");
    try {
        while (!g_stop_requested && !loop_error) {
            const CycleTiming timing = tick.WaitCycleStart();
            CollectCycleTiming(cycle, timing, tick, period_samples, overrun_samples,
                               abnormal_periods);

            if (cycle == kEnableAtCycle) {
                std::cerr << ">>> 双臂上使能 cycle=" << cycle << std::endl;
                ctrl.Left().SetEnable(EnableMode::Enable);
                ctrl.Right().SetEnable(EnableMode::Enable);
                enable_cmd_sent = true;
            }

            if (g_stop_requested && !teardown_started) {
                BeginTeardown(ctrl, left_flow, right_flow, cycle);
                teardown_started = true;
            }

            PollPicoUdp(pico_rx, pico, last_pico_seq);

            const bool teleop_active = TeleopReady(left_flow, right_flow);
            TickServoAt50Hz(ctrl.Left(), ctrl.Right(), left_flow, right_flow, pico,
                            teleop_active, cycle);
            const bool fresh = PicoFresh(pico);
            const float rt = fresh ? pico.right_trigger : 0.0f;
            const float lt = fresh ? pico.left_trigger : 0.0f;
            TickTriggerRelease(ctrl.Right(), right_flow, rt, teleop_active);
            TickTriggerRelease(ctrl.Left(), left_flow, lt, teleop_active);

            ctrl.Run();

            if (!PublishJointsUdp(joint_tx, ctrl)) {
                // 可视化未启动时允许 sendto 失败，不中断控制环
            }

            const MVControl::HwRunStats& stats = ctrl.LastHwRunStats();
            uint8_t flags = 0;
            if (stats.sent_this_cycle) {
                flags |= kSampleHwSent;
            }
            if (stats.send_clear_fail_total > prev_clear_fail) {
                flags |= kSampleClearFail;
                prev_clear_fail = stats.send_clear_fail_total;
            }

            if (cycle % kRecordPeriodCycles == 0) {
                const bool pico_fresh = PicoFresh(pico);
                if (!PushRecordSample(
                        cycle, ctrl, pico, pico_fresh,
                        static_cast<uint8_t>(PackPhase(left_flow, right_flow)), flags,
                        timing, tick, recorder, pico_recorder)) {
                    std::fprintf(stderr, "[FAIL] recorder full at cycle %d\n", cycle);
                    loop_error = true;
                    break;
                }
            }

            TickArmAfter(ctrl.Left(), left_flow, cycle, enable_cmd_sent);
            TickArmAfter(ctrl.Right(), right_flow, cycle, enable_cmd_sent);

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
    if (!teardown_started) {
        BeginTeardown(ctrl, left_flow, right_flow, cycle);
        teardown_started = true;
        for (int i = 0; i < 120000 && !AllDone(left_flow, right_flow); ++i) {
            const CycleTiming timing = tick.WaitCycleStart();
            CollectCycleTiming(cycle, timing, tick, period_samples, overrun_samples,
                               abnormal_periods);
            ctrl.Run();
            TickArmAfter(ctrl.Left(), left_flow, cycle, enable_cmd_sent);
            TickArmAfter(ctrl.Right(), right_flow, cycle, enable_cmd_sent);
            if (cycle % kRecordPeriodCycles == 0) {
                const bool pico_fresh = PicoFresh(pico);
                PushRecordSample(cycle, ctrl, pico, pico_fresh,
                                 static_cast<uint8_t>(PackPhase(left_flow, right_flow)),
                                 0, timing, tick, recorder, pico_recorder);
            }
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
                      pico_recorder.Size())) {
        std::fprintf(stderr, "[WARN] write run_meta.txt failed\n");
    }

    const std::string period_summary_path =
        std::string(opt.out_dir) + "/period_summary.txt";
    const std::string period_abnormal_path =
        std::string(opt.out_dir) + "/period_abnormal.csv";
    if (!period_samples.empty()) {
        SavePeriodSummary(period_summary_path.c_str(), period_samples, overrun_samples,
                          1000);
    }
    SavePeriodAbnormalCsv(period_abnormal_path.c_str(), abnormal_periods);
    ExportDiagEventsCsv(diag_recorder, opt.out_dir);
    ExportDiagSummary(diag_recorder, opt.out_dir);
    ExportTeleopDiagReport(pico_recorder, opt.out_dir);

    PrintPeriodSummaryBrief(period_samples, overrun_samples, 1000,
                            abnormal_periods.size());

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
