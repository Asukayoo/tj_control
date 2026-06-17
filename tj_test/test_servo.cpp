// test_servo：读 Pico CSV → 分臂使能 → GoWork → 50Hz ServoPByPico → GoHome → 导出 CSV
#include "mv_control.hpp"
#include "run_session.hpp"
#include "test_diag.hpp"

#include <algorithm>
#include <atomic>
#include <climits>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#ifndef TJ_DATA_DEFAULT
#define TJ_DATA_DEFAULT "data/test_ServoPByPico"
#endif
#ifndef TJ_PICO_CSV_DEFAULT
#define TJ_PICO_CSV_DEFAULT                                                     \
    "data/test_teleop_data/pico_record_20260615_211537.csv"
#endif

namespace {

constexpr int kEnableAtCycle = 10;
constexpr int kWaitAfterEnable = 1000;
constexpr int kCycleMax = 300000;
constexpr int64_t kPeriodUsLo = 900;
constexpr int64_t kPeriodUsHi = 1100;
constexpr double kRefMovedEps = 1e-6;
constexpr double kM2Mm = 1000.0;
// 外部输入：test_servo 以该频率调用 ServoPByPico（1kHz 下每 20 周期一帧）
constexpr int kPicoServoHz = 50;
constexpr int kPicoServoPeriodCycles = 1000 / kPicoServoHz;
// 内部插值 kStreamServoCycles（common.hpp，40 周期）由 motion 层处理，与上式无关

std::atomic<bool> g_stop_requested{false};

void OnSigInt(int /*signo*/) { g_stop_requested = true; }

void PrintArmDiag(const char* name, const Robot& arm) {
    std::printf("[%s] 控制模式=%d 错误码=%d StatusCode=%d\n", name,
                static_cast<int>(arm.GetControlModeStatus()),
                static_cast<int>(arm.GetErrorCode()),
                static_cast<int>(arm.GetStatusCode()));
}

enum class ArmPhase : int {
    WaitEnable = 0,
    WaitSettle,
    WaitMovJ,
    ServoSend,
    ServoStopWait,
    WaitDisable,
    Done,
};

enum class MovJLeg : uint8_t { Work = 0, Home = 1 };

struct ArmFlow {
    const char* name = "";
    ArmPhase phase = ArmPhase::WaitEnable;
    int wait_cnt = 0;
    int servo_div = 0;
    std::size_t servo_idx = 0;
    MovJLeg movj_leg = MovJLeg::Work;
    double ref_q0[DOF]{};
    const std::vector<Pose>* poses = nullptr;
};

bool AllArmsDone(const ArmFlow& left, const ArmFlow& right) {
    return left.phase == ArmPhase::Done && right.phase == ArmPhase::Done;
}

Pose MakePoseFromCsvM(double x, double y, double z, double qw, double qx, double qy,
                      double qz) {
    Pose p;
    p.pos = V3d(x, y, z) * kM2Mm;  // Pico CSV 为 m，Pose.pos 为 mm
    p.quat = Quat(qw, qx, qy, qz).normalized();
    return p;
}

bool SkipCsvField(const char*& p) {
    if (p == nullptr || *p == '\0') {
        return false;
    }
    p = std::strchr(p, ',');
    if (p == nullptr) {
        return false;
    }
    ++p;
    return true;
}

bool ParsePoseFields(const char*& p, Pose& out) {
    if (p != nullptr && *p == ',') {
        ++p;
    }
    double v[7] = {};
    for (int i = 0; i < 7; ++i) {
        if (p == nullptr || *p == '\0') {
            return false;
        }
        char* end = nullptr;
        v[i] = std::strtod(p, &end);
        if (end == p) {
            return false;
        }
        p = end;
        if (i < 6) {
            if (*p != ',') {
                return false;
            }
            ++p;
        }
    }
    out = MakePoseFromCsvM(v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
    return true;
}

bool LoadPicoCsv(const char* path, std::vector<Pose>& left,
                 std::vector<Pose>& right) {
    std::ifstream in(path);
    if (!in.is_open()) {
        std::fprintf(stderr, "[FAIL] 无法打开 Pico CSV: %s\n", path);
        return false;
    }

    left.clear();
    right.clear();
    std::string line;
    if (!std::getline(in, line)) {
        std::fprintf(stderr, "[FAIL] Pico CSV 为空: %s\n", path);
        return false;
    }

    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        const char* p = line.c_str();
        if (!SkipCsvField(p) || !SkipCsvField(p)) {
            continue;
        }
        Pose rp;
        Pose lp;
        if (!ParsePoseFields(p, rp)) {
            continue;
        }
        if (!ParsePoseFields(p, lp)) {
            continue;
        }
        right.push_back(rp);
        left.push_back(lp);
    }

    if (left.empty() || right.empty()) {
        std::fprintf(stderr, "[FAIL] Pico CSV 无有效帧: %s\n", path);
        return false;
    }
    if (left.size() != right.size()) {
        std::fprintf(stderr, "[WARN] 左右帧数不一致 L=%zu R=%zu，取较短\n",
                     left.size(), right.size());
        const std::size_t n = std::min(left.size(), right.size());
        left.resize(n);
        right.resize(n);
    }
    std::printf("[load] %s  frames=%zu  (pos m→mm, quat wxyz)\n", path,
                left.size());
    return true;
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

const char* MovJLegName(MovJLeg leg) {
    return leg == MovJLeg::Work ? "work" : "home";
}

void IssueMovJLeg(Robot& arm, ArmFlow& flow, MovJLeg leg, int cycle) {
    if (arm.GetErrorCode() != ErrorCode::Normal) {
        arm.ClearError();
    }
    SnapshotRef(arm, flow);
    flow.movj_leg = leg;
    flow.phase = ArmPhase::WaitMovJ;
    std::cerr << ">>> MovJ " << MovJLegName(leg) << " " << flow.name << " cycle=" << cycle
              << std::endl;
    if (leg == MovJLeg::Work) {
        arm.GoWork();
    } else {
        arm.GoHome();
    }
}

void IssueDisable(Robot& arm, ArmFlow& flow, int cycle) {
    std::cerr << ">>> " << flow.name << " 下使能 cycle=" << cycle << std::endl;
    arm.SetEnable(EnableMode::Disable);
    flow.phase = ArmPhase::WaitDisable;
}

void BeginServo(Robot& arm, ArmFlow& flow, int cycle) {
    flow.servo_idx = 0;
    flow.servo_div = 0;
    flow.wait_cnt = 0;
    flow.phase = ArmPhase::ServoSend;
    std::cerr << ">>> ServoPByPico " << flow.name << " start frames="
              << flow.poses->size() << " cycle=" << cycle << std::endl;
    if (!flow.poses->empty()) {
        arm.ServoPByPico((*flow.poses)[0], true);
        flow.servo_idx = 1;
    }
}

void EndServoSession(Robot& arm, ArmFlow& flow, int cycle) {
    arm.ServoPByPico(Pose{}, false);
    flow.wait_cnt = 0;
    flow.phase = ArmPhase::ServoStopWait;
    std::cerr << ">>> ServoPByPico " << flow.name << " stop session cycle=" << cycle
              << std::endl;
}

void TickArmBefore(Robot& arm, ArmFlow& flow, int cycle) {
    switch (flow.phase) {
        case ArmPhase::WaitSettle:
            if (++flow.wait_cnt >= kWaitAfterEnable) {
                IssueMovJLeg(arm, flow, MovJLeg::Work, cycle);
            }
            break;
        case ArmPhase::ServoSend:
            if (flow.poses == nullptr || flow.poses->empty()) {
                EndServoSession(arm, flow, cycle);
                break;
            }
            if (flow.servo_idx >= flow.poses->size()) {
                EndServoSession(arm, flow, cycle);
                break;
            }
            if (++flow.servo_div >= kPicoServoPeriodCycles) {
                flow.servo_div = 0;
                arm.ServoPByPico((*flow.poses)[flow.servo_idx], true);
                ++flow.servo_idx;
                if (flow.servo_idx >= flow.poses->size()) {
                    EndServoSession(arm, flow, cycle);
                }
            }
            break;
        default:
            break;
    }
}

void TickArmAfter(Robot& arm, ArmFlow& flow, int cycle, bool enable_cmd_sent) {
    switch (flow.phase) {
        case ArmPhase::WaitEnable:
            if (enable_cmd_sent &&
                arm.GetEnableState() == EnableState::Enabled) {
                std::cerr << ">>> " << flow.name << " 上使能成功 cycle=" << cycle
                          << std::endl;
                flow.wait_cnt = 0;
                flow.phase = ArmPhase::WaitSettle;
            }
            break;
        case ArmPhase::WaitMovJ:
            if (!MotionFinished(arm, flow)) {
                break;
            }
            if (arm.GetStatusCode() == StatusCode::Fault) {
                std::cerr << ">>> MovJ " << MovJLegName(flow.movj_leg) << " "
                          << flow.name << " fault cycle=" << cycle << std::endl;
            } else {
                std::cerr << ">>> MovJ " << MovJLegName(flow.movj_leg) << " "
                          << flow.name << " 完成 cycle=" << cycle << std::endl;
            }
            if (flow.movj_leg == MovJLeg::Work) {
                BeginServo(arm, flow, cycle);
            } else {
                IssueDisable(arm, flow, cycle);
            }
            break;
        case ArmPhase::ServoStopWait:
            if (arm.GetStatusCode() == StatusCode::Ready ||
                arm.GetStatusCode() == StatusCode::Fault) {
                IssueMovJLeg(arm, flow, MovJLeg::Home, cycle);
            }
            break;
        case ArmPhase::WaitDisable:
            if (arm.GetEnableState() == EnableState::Disabled) {
                std::cerr << ">>> " << flow.name << " 下使能成功 cycle=" << cycle
                          << std::endl;
                flow.phase = ArmPhase::Done;
            }
            break;
        default:
            break;
    }
}

int PackPhase(const ArmFlow& left, const ArmFlow& right) {
    const auto pack = [](const ArmFlow& f) {
        if (f.phase == ArmPhase::WaitMovJ) {
            return 2 + static_cast<int>(f.movj_leg);
        }
        if (f.phase == ArmPhase::ServoSend) {
            return 4;
        }
        if (f.phase == ArmPhase::ServoStopWait) {
            return 5;
        }
        return static_cast<int>(f.phase);
    };
    return pack(left) * 10 + pack(right);
}

bool WriteRunMeta(const char* dir, const char* pico_csv, std::size_t frames) {
    const std::string path = std::string(dir) + "/run_meta.txt";
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f, "servo_hz=%d\n", kPicoServoHz);
    std::fprintf(f, "pico_csv=%s\n", pico_csv);
    std::fprintf(f, "pico_frames=%zu\n", frames);
    std::fclose(f);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const char* out_dir = TJ_DATA_DEFAULT;
    const char* pico_csv = TJ_PICO_CSV_DEFAULT;
    int pos = 0;
    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] == '-' || argv[i][0] == '\0') {
            continue;
        }
        if (pos == 0) {
            out_dir = argv[i];
        } else if (pos == 1) {
            pico_csv = argv[i];
        }
        ++pos;
    }

    std::vector<Pose> left_poses;
    std::vector<Pose> right_poses;
    if (!LoadPicoCsv(pico_csv, left_poses, right_poses)) {
        return 2;
    }

    int sim = -1;
    while (sim != 0 && sim != 1) {
        std::printf("0=硬件  1=仿真: ");
        if (std::scanf("%d", &sim) != 1) {
            return 1;
        }
    }
    const bool is_sim = (sim == 1);
    std::printf("[start] %s  pico=%s  out=%s  servo=%dHz\n",
                is_sim ? "仿真" : "硬件", pico_csv, out_dir, kPicoServoHz);

    MVControl ctrl;
    if (!ctrl.Init(MV_CONTROL_CONFIG_DEFAULT, is_sim)) {
        std::fprintf(stderr, "[FAIL] Init\n");
        return 2;
    }

    PrintArmDiag("left", ctrl.Left());
    PrintArmDiag("right", ctrl.Right());
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

    ArmFlow left_flow{"left"};
    ArmFlow right_flow{"right"};
    left_flow.poses = &left_poses;
    right_flow.poses = &right_poses;

    bool enable_cmd_sent = false;
    int cycle = 0;
    int64_t period_min = LLONG_MAX;
    int64_t period_max = 0;
    int64_t period_sum = 0;
    int period_cnt = 0;
    std::vector<std::pair<int, int64_t>> abnormal_periods;
    abnormal_periods.reserve(64);
    bool interrupted = false;
    bool loop_error = false;

    RunSession session;
    session.ResetAnchor();
    session.ReserveRecorder(kCycleMax);
    ctrl.ResetHwRunStats();

    std::printf("\n======== 1kHz loop (GoWork→ServoPByPico→GoHome) ========\n");
    try {
        while (!AllArmsDone(left_flow, right_flow) && !g_stop_requested) {
            if (cycle == kEnableAtCycle) {
                std::cerr << ">>> left 上使能 cycle=" << cycle << std::endl;
                ctrl.Left().SetEnable(EnableMode::Enable);
                std::cerr << ">>> right 上使能 cycle=" << cycle << std::endl;
                ctrl.Right().SetEnable(EnableMode::Enable);
                enable_cmd_sent = true;
            }

            TickArmBefore(ctrl.Left(), left_flow, cycle);
            TickArmBefore(ctrl.Right(), right_flow, cycle);

            const SessionTickResult tick = session.Step(
                ctrl, cycle, static_cast<uint8_t>(PackPhase(left_flow, right_flow)));

            TickArmAfter(ctrl.Left(), left_flow, cycle, enable_cmd_sent);
            TickArmAfter(ctrl.Right(), right_flow, cycle, enable_cmd_sent);

            if (tick.period_us > 0) {
                period_min = std::min(period_min, tick.period_us);
                period_max = std::max(period_max, tick.period_us);
                period_sum += tick.period_us;
                ++period_cnt;
                if (tick.period_us < kPeriodUsLo || tick.period_us > kPeriodUsHi) {
                    abnormal_periods.emplace_back(cycle, tick.period_us);
                }
            }
            if (!tick.recorded) {
                std::fprintf(stderr, "[FAIL] recorder full at cycle %d\n", cycle);
                loop_error = true;
                break;
            }

            if (++cycle > kCycleMax) {
                std::fprintf(stderr, "[FAIL] timeout L_phase=%d R_phase=%d cycle>%d\n",
                             static_cast<int>(left_flow.phase),
                             static_cast<int>(right_flow.phase), kCycleMax);
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

    interrupted = g_stop_requested.load();
    if (!session.ExportCsv(out_dir, true)) {
        std::fprintf(stderr, "[FAIL] export csv to %s\n", out_dir);
        return 1;
    }
    if (!WriteRunMeta(out_dir, pico_csv, left_poses.size())) {
        std::fprintf(stderr, "[WARN] write run_meta.txt failed\n");
    }

    const bool pass = !loop_error && !interrupted &&
                      ctrl.Left().GetEnableState() == EnableState::Disabled &&
                      ctrl.Right().GetEnableState() == EnableState::Disabled;
    if (interrupted) {
        std::printf("\n[INTERRUPT] Ctrl+C @ cycle=%d, csv saved → %s\n", cycle,
                    out_dir);
    } else {
        std::printf("\n[%s] cycles=%d  saved → %s\n", pass ? "PASS" : "FAIL", cycle,
                    out_dir);
    }
    if (period_cnt > 0) {
        PrintPeriodStats(session.Recorder(), 1000);
    }
    return (pass || interrupted) ? 0 : 3;
}
