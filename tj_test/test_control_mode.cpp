// test_control_mode：位置→关节阻抗→笛卡尔阻抗；各段 GoHome→MovJ 目标→GoHome→下使能
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

#ifndef TJ_DATA_DEFAULT
#define TJ_DATA_DEFAULT "data/test_control_mode"
#endif

namespace {

constexpr int kStartAtCycle = 10;
constexpr int kWaitAfterEnable = 1000;
constexpr int kCycleMax = 300000;
constexpr int64_t kPeriodUsLo = 900;
constexpr int64_t kPeriodUsHi = 1100;

constexpr double kLeftTargetDeg[DOF] = {50.120,  -40.842, -143.263, -100.953,
                                        17.520,  -42.639, -42.742};
constexpr double kRightTargetDeg[DOF] = {-36.574, -72.997, 116.251,  -88.448,
                                         -26.475, -32.146, 37.892};

std::atomic<bool> g_stop_requested{false};

void OnSigInt(int /*signo*/) { g_stop_requested = true; }

V7d DegToRad(const double q_deg[DOF]) {
    V7d q;
    for (int i = 0; i < DOF; ++i) {
        q(i) = q_deg[i] * D2R;
    }
    return q;
}

void PrintArmDiag(const char* name, const Robot& arm) {
    std::printf("[%s] 模式=%d 使能=%d 错误=%d Status=%d\n", name,
                static_cast<int>(arm.GetControlModeStatus()),
                static_cast<int>(arm.GetEnableState()),
                static_cast<int>(arm.GetErrorCode()),
                static_cast<int>(arm.GetStatusCode()));
}

enum class Segment : uint8_t { Position = 0, JointImp = 1, CartImp = 2, Done = 3 };

enum class Step : uint8_t {
    Idle = 0,
    SetControlMode,
    IssueEnable,
    WaitEnable,
    WaitSettle,
    WaitMovJ,
    IssueDisable,
    WaitDisable,
    Finished = 255,
};

enum class MovJLeg : uint8_t { Home1 = 0, Target = 1, Home2 = 2 };

struct ArmFlow {
    const char* name = "";
    Segment segment = Segment::Position;
    Step step = Step::Idle;
    MovJLeg movj_leg = MovJLeg::Home1;
    int wait_cnt = 0;
    V7d target_q = V7d::Zero();
};

const char* SegmentName(Segment seg) {
    switch (seg) {
        case Segment::Position:
            return "位置";
        case Segment::JointImp:
            return "关节阻抗";
        case Segment::CartImp:
            return "笛卡尔阻抗";
        default:
            return "完成";
    }
}

ControlMode SegmentToMode(Segment seg) {
    switch (seg) {
        case Segment::JointImp:
            return ControlMode::JointImp;
        case Segment::CartImp:
            return ControlMode::CartImp;
        default:
            return ControlMode::Position;
    }
}

const char* MovJLegName(MovJLeg leg) {
    switch (leg) {
        case MovJLeg::Home1:
            return "GoHome";
        case MovJLeg::Target:
            return "MovJ目标";
        case MovJLeg::Home2:
            return "GoHome";
    }
    return "?";
}

bool AllArmsDone(const ArmFlow& left, const ArmFlow& right) {
    return left.step == Step::Finished && right.step == Step::Finished;
}

bool MotionFinished(const Robot& arm) {
    if (arm.GetStatusCode() == StatusCode::Fault) {
        return true;
    }
    return arm.GetStatusCode() == StatusCode::Ready;
}

void IssueMovJLeg(Robot& arm, ArmFlow& flow, MovJLeg leg, int cycle) {
    if (arm.GetErrorCode() != ErrorCode::Normal) {
        arm.ClearError();
    }
    flow.movj_leg = leg;
    flow.step = Step::WaitMovJ;
    std::cerr << ">>> " << MovJLegName(leg) << " " << flow.name << " ["
              << SegmentName(flow.segment) << "] 开始 cycle=" << cycle << std::endl;
    switch (leg) {
        case MovJLeg::Home1:
        case MovJLeg::Home2:
            arm.GoHome();
            break;
        case MovJLeg::Target:
            arm.MovJ(flow.target_q);
            break;
    }
}

void ExecSetControlMode(Robot& arm, ArmFlow& flow, int cycle) {
    std::cerr << ">>> SetControlMode " << SegmentName(flow.segment) << " " << flow.name
              << " cycle=" << cycle << std::endl;
    if (!arm.SetControlMode(SegmentToMode(flow.segment))) {
        std::cerr << ">>> SetControlMode " << SegmentName(flow.segment) << " "
                  << flow.name << " 失败 err=" << static_cast<int>(arm.GetErrorCode())
                  << " cycle=" << cycle << std::endl;
        return;
    }
    std::cerr << ">>> SetControlMode " << SegmentName(flow.segment) << " " << flow.name
              << " 成功 cycle=" << cycle << std::endl;
    flow.step = Step::IssueEnable;
}

void ExecEnable(Robot& arm, ArmFlow& flow, int cycle) {
    std::cerr << ">>> 上使能 " << flow.name << " [" << SegmentName(flow.segment)
              << "] cycle=" << cycle << std::endl;
    arm.SetEnable(EnableMode::Enable);
    flow.step = Step::WaitEnable;
}

void ExecDisable(Robot& arm, ArmFlow& flow, int cycle) {
    std::cerr << ">>> 下使能 " << flow.name << " [" << SegmentName(flow.segment)
              << "] cycle=" << cycle << std::endl;
    arm.SetEnable(EnableMode::Disable);
    flow.step = Step::WaitDisable;
}

void AdvanceSegment(ArmFlow& flow) {
    if (flow.segment == Segment::Position) {
        flow.segment = Segment::JointImp;
        flow.step = Step::SetControlMode;
        return;
    }
    if (flow.segment == Segment::JointImp) {
        flow.segment = Segment::CartImp;
        flow.step = Step::SetControlMode;
        return;
    }
    flow.segment = Segment::Done;
    flow.step = Step::Finished;
}

void TickArmBefore(Robot& arm, ArmFlow& flow, int cycle, bool started) {
    if (!started || flow.step == Step::Finished || flow.step == Step::Idle) {
        return;
    }
    switch (flow.step) {
        case Step::SetControlMode:
            ExecSetControlMode(arm, flow, cycle);
            break;
        case Step::IssueEnable:
            ExecEnable(arm, flow, cycle);
            break;
        case Step::WaitSettle:
            if (++flow.wait_cnt >= kWaitAfterEnable) {
                IssueMovJLeg(arm, flow, MovJLeg::Home1, cycle);
            }
            break;
        default:
            break;
    }
}

void TickArmAfter(Robot& arm, ArmFlow& flow, int cycle, bool started) {
    if (!started) {
        return;
    }
    switch (flow.step) {
        case Step::WaitEnable:
            if (arm.GetEnableState() == EnableState::Enabled) {
                std::cerr << ">>> 上使能 " << flow.name << " ["
                          << SegmentName(flow.segment) << "] 成功 cycle=" << cycle
                          << std::endl;
                flow.wait_cnt = 0;
                flow.step = Step::WaitSettle;
            }
            break;
        case Step::WaitMovJ:
            if (!MotionFinished(arm)) {
                break;
            }
            if (arm.GetStatusCode() == StatusCode::Fault) {
                std::cerr << ">>> " << MovJLegName(flow.movj_leg) << " " << flow.name
                          << " fault cycle=" << cycle << std::endl;
            } else {
                std::cerr << ">>> " << MovJLegName(flow.movj_leg) << " " << flow.name
                          << " [" << SegmentName(flow.segment) << "] 完成 cycle="
                          << cycle << std::endl;
            }
            if (flow.movj_leg == MovJLeg::Home1) {
                IssueMovJLeg(arm, flow, MovJLeg::Target, cycle);
            } else if (flow.movj_leg == MovJLeg::Target) {
                IssueMovJLeg(arm, flow, MovJLeg::Home2, cycle);
            } else {
                ExecDisable(arm, flow, cycle);
            }
            break;
        case Step::WaitDisable:
            if (arm.GetEnableState() == EnableState::Disabled) {
                std::cerr << ">>> 下使能 " << flow.name << " ["
                          << SegmentName(flow.segment) << "] 成功 cycle=" << cycle
                          << std::endl;
                AdvanceSegment(flow);
            }
            break;
        default:
            break;
    }
}

int PackPhase(const ArmFlow& left, const ArmFlow& right) {
    const auto pack = [](const ArmFlow& f) {
        return static_cast<int>(f.segment) * 100 + static_cast<int>(f.step) * 10 +
               static_cast<int>(f.movj_leg);
    };
    return pack(left) * 10000 + pack(right);
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
        std::printf("0=退出  1=进入1kHz循环: ");
        if (std::scanf("%d", &go) != 1) {
            return 1;
        }
    }
    if (go == 0) {
        std::printf("[exit] 未进入循环\n");
        return 0;
    }

    std::signal(SIGINT, OnSigInt);

    ArmFlow left_flow{"left", Segment::Position, Step::Idle, MovJLeg::Home1, 0,
                      DegToRad(kLeftTargetDeg)};
    ArmFlow right_flow{"right", Segment::Position, Step::Idle, MovJLeg::Home1, 0,
                       DegToRad(kRightTargetDeg)};

    bool started = false;
    int cycle = 0;
    bool interrupted = false;
    bool loop_error = false;

    RunSession session;
    session.ResetAnchor();
    session.ReserveRecorder(kCycleMax);
    ctrl.ResetHwRunStats();

    std::printf("\n======== 1kHz 控制模式测试 ========\n");
    std::printf("左目标[deg]: %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                kLeftTargetDeg[0], kLeftTargetDeg[1], kLeftTargetDeg[2],
                kLeftTargetDeg[3], kLeftTargetDeg[4], kLeftTargetDeg[5],
                kLeftTargetDeg[6]);
    std::printf("右目标[deg]: %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                kRightTargetDeg[0], kRightTargetDeg[1], kRightTargetDeg[2],
                kRightTargetDeg[3], kRightTargetDeg[4], kRightTargetDeg[5],
                kRightTargetDeg[6]);

    try {
        while (!AllArmsDone(left_flow, right_flow) && !g_stop_requested) {
            if (cycle == kStartAtCycle && !started) {
                // 初始即为位置模式，直接上使能
                left_flow.step = Step::IssueEnable;
                right_flow.step = Step::IssueEnable;
                started = true;
                std::printf(">>> 流程启动 cycle=%d\n", cycle);
            }

            TickArmBefore(ctrl.Left(), left_flow, cycle, started);
            TickArmBefore(ctrl.Right(), right_flow, cycle, started);

            const SessionTickResult tick = session.Step(
                ctrl, cycle, static_cast<uint8_t>(PackPhase(left_flow, right_flow) & 0xFF));

            TickArmAfter(ctrl.Left(), left_flow, cycle, started);
            TickArmAfter(ctrl.Right(), right_flow, cycle, started);

            if (!tick.recorded) {
                std::fprintf(stderr, "[FAIL] recorder full at cycle %d\n", cycle);
                loop_error = true;
                break;
            }

            if (++cycle > kCycleMax) {
                std::fprintf(stderr,
                             "[FAIL] timeout L_step=%d R_step=%d cycle>%d\n",
                             static_cast<int>(left_flow.step),
                             static_cast<int>(right_flow.step), kCycleMax);
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
                      ctrl.Right().GetEnableState() == EnableState::Disabled &&
                      AllArmsDone(left_flow, right_flow);
    if (interrupted) {
        std::printf("\n[INTERRUPT] Ctrl+C @ cycle=%d, csv saved → %s\n", cycle, out_dir);
    } else {
        std::printf("\n[%s] cycles=%d  saved → %s\n", pass ? "PASS" : "FAIL", cycle,
                    out_dir);
    }
    SavePeriodDiagFromRecorder(out_dir, session.Recorder(), 1000, kPeriodUsLo,
                               kPeriodUsHi);
    return (pass || interrupted) ? 0 : 3;
}
