// test_enable：Init → 500Hz(使能→运行→下使能) → 内存记录 → 导出 CSV
#include "mv_control.hpp"
#include "run_session.hpp"
#include "test_diag.hpp"

#include <algorithm>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#ifndef TJ_DATA_DEFAULT
#define TJ_DATA_DEFAULT "/home/yxc/tj_control/data/test_enable"
#endif

namespace {

constexpr int kEnableAtCycle = 10;
constexpr int kRunN = 10;
constexpr int kCycleMax = kControlHz * 5;
constexpr int64_t kPeriodUsLo = kControlPeriodUsLo;
constexpr int64_t kPeriodUsHi = kControlPeriodUsHi;

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

    int left_step = 0;
    int left_run = 0;
    int right_step = 0;
    int right_run = 0;
    int cycle = 0;

    RunSession session;
    session.ResetAnchor();
    session.ReserveRecorder(kCycleMax);
    ctrl.ResetHwRunStats();

    std::printf("\n======== 500Hz loop ========\n");
    while (true) {
        if (cycle == kEnableAtCycle) {
            std::cerr << ">>> enable cmd cycle " << cycle << std::endl;
            ctrl.Left().SetEnable(EnableMode::Enable);
            ctrl.Right().SetEnable(EnableMode::Enable);
            left_step = 1;
            right_step = 1;
        }
        if (left_step == 3) {
            std::cerr << "left disable start cycle " << cycle << std::endl;
            ctrl.Left().SetEnable(EnableMode::Disable);
            left_step = 4;
        }
        if (right_step == 3) {
            std::cerr << "right disable start cycle " << cycle << std::endl;
            ctrl.Right().SetEnable(EnableMode::Disable);
            right_step = 4;
        }

        const SessionTickResult tick = session.Step(ctrl, cycle);

        const EnableState left_en = ctrl.Left().GetEnableState();
        const EnableState right_en = ctrl.Right().GetEnableState();

        if (left_step == 1 && left_en == EnableState::Enabled) {
            left_step = 2;
            left_run = 0;
            std::cerr << "left enable done cycle " << cycle << std::endl;
        }
        if (left_step == 2) {
            ++left_run;
            if (left_run >= kRunN) {
                left_step = 3;
                std::cerr << "left run done cycle " << cycle << std::endl;
            }
        }
        if (left_step == 4 && left_en == EnableState::Disabled) {
            left_step = 5;
            std::cerr << "left disable done cycle " << cycle << std::endl;
        }

        if (right_step == 1 && right_en == EnableState::Enabled) {
            right_step = 2;
            right_run = 0;
            std::cerr << "right enable done cycle " << cycle << std::endl;
        }
        if (right_step == 2) {
            ++right_run;
            if (right_run >= kRunN) {
                right_step = 3;
                std::cerr << "right run done cycle " << cycle << std::endl;
            }
        }
        if (right_step == 4 && right_en == EnableState::Disabled) {
            right_step = 5;
            std::cerr << "right disable done cycle " << cycle << std::endl;
        }

        if (left_step == 5 && right_step == 5) {
            break;
        }
        if (++cycle > kCycleMax) {
            std::fprintf(stderr, "[FAIL] timeout cycle>%d\n", kCycleMax);
            break;
        }
    }

    if (!session.ExportCsv(out_dir, false)) {
        std::fprintf(stderr, "[FAIL] export csv\n");
        return 1;
    }

    std::printf("\n[PASS] cycles=%d  saved → %s\n", cycle, out_dir);
    SavePeriodDiagFromRecorder(out_dir, session.Recorder(), kControlPeriodUs,
                               kPeriodUsLo, kPeriodUsHi);
    return (left_step == 5 && right_step == 5) ? 0 : 3;
}
