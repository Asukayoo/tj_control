// test_movj：Init → 分臂使能 → MovJ(work→home) → 下使能 → 导出 CSV
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
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#ifndef TJ_DATA_DEFAULT
#define TJ_DATA_DEFAULT "data/test_movj"
#endif

namespace {

constexpr int kEnableAtCycle = 10;
constexpr int kWaitAfterEnable = kControlCyclesPerSecond;
constexpr int kCycleMax = kControlMaxCycles5Min;
constexpr int64_t kPeriodUsLo = kControlPeriodUsLo;
constexpr int64_t kPeriodUsHi = kControlPeriodUsHi;
constexpr double kRefMovedEps = 1e-6;

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
    WaitDisable,
    Done,
};

enum class MovJLeg : uint8_t { Work = 0, Home = 1 };

struct ArmFlow {
    const char* name = "";
    ArmPhase phase = ArmPhase::WaitEnable;
    int wait_cnt = 0;
    MovJLeg movj_leg = MovJLeg::Work;
    double ref_q0[DOF]{};
};

bool AllArmsDone(const ArmFlow& left, const ArmFlow& right) {
    return left.phase == ArmPhase::Done && right.phase == ArmPhase::Done;
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

// Run 之后判定：Ready 且 ref 已变化，或 Fault
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

void TickArmBefore(Robot& arm, ArmFlow& flow, int cycle) {
    if (flow.phase == ArmPhase::WaitSettle && ++flow.wait_cnt >= kWaitAfterEnable) {
        IssueMovJLeg(arm, flow, MovJLeg::Work, cycle);
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
                std::cerr << ">>> MovJ " << MovJLegName(flow.movj_leg) << " " << flow.name
                          << " fault cycle=" << cycle
                          << " err=" << static_cast<int>(arm.GetErrorCode());
                if (flow.movj_leg == MovJLeg::Work) {
                    std::cerr << "，继续 home";
                }
                std::cerr << std::endl;
            } else {
                std::cerr << ">>> MovJ " << MovJLegName(flow.movj_leg) << " " << flow.name
                          << " 完成 cycle=" << cycle << std::endl;
            }
            if (flow.movj_leg == MovJLeg::Work) {
                IssueMovJLeg(arm, flow, MovJLeg::Home, cycle);
            } else {
                IssueDisable(arm, flow, cycle);
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
        return static_cast<int>(f.phase);
    };
    return pack(left) * 10 + pack(right);
}

}  // namespace

int main(int argc, char** argv) {
    const char* out_dir = TJ_DATA_DEFAULT;
    for (int i = 1; i < argc; ++i) {
        if (argv[i][0] != '-' && argv[i][0] != '\0') {
            out_dir = argv[i];
        }
    }

    int sim = -1;
    while (sim != 0 && sim != 1) {
        std::printf("0=硬件  1=仿真: ");
        if (std::scanf("%d", &sim) != 1) {
            return 1;
        }
    }
    const bool is_sim = (sim == 1);
    std::printf("[start] %s  out=%s\n", is_sim ? "仿真" : "硬件", out_dir);

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
        std::printf("0=退出  1=进入500Hz循环: ");
        if (std::scanf("%d", &go) != 1) {
            return 1;
        }
    }
    if (go == 0) {
        std::printf("[exit] 未进入循环\n");
        return 0;
    }

    std::signal(SIGINT, OnSigInt);

    ArmFlow left_flow{"left"};
    ArmFlow right_flow{"right"};
    bool enable_cmd_sent = false;
    int cycle = 0;
    bool interrupted = false;
    bool loop_error = false;

    RunSession session;
    session.ResetAnchor();
    session.ReserveRecorder(kCycleMax);
    ctrl.ResetHwRunStats();

    std::printf("\n======== 500Hz loop (使能→MovJ work/home→下使能) ========\n");
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

    const bool pass = !loop_error && !interrupted &&
                      ctrl.Left().GetEnableState() == EnableState::Disabled &&
                      ctrl.Right().GetEnableState() == EnableState::Disabled;
    if (interrupted) {
        std::printf("\n[INTERRUPT] Ctrl+C @ cycle=%d, csv saved → %s\n", cycle, out_dir);
    } else {
        std::printf("\n[%s] cycles=%d  saved → %s\n", pass ? "PASS" : "FAIL", cycle,
                    out_dir);
    }
    SavePeriodDiagFromRecorder(out_dir, session.Recorder(), kControlPeriodUs,
                               kPeriodUsLo, kPeriodUsHi);
    return (pass || interrupted) ? 0 : 3;
}
