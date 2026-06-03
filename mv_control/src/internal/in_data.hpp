#pragma once

#include "common.hpp"

// 控制周期 1kHz
constexpr double kControlDt = 0.001;

enum class CmdType {
    Stop = 0,
    ServoJ = 1,
    ServoP = 2,
    MovJ = 3,
    MovL = 4,
    GoWork = 5,
    GoHome = 6,
};

// 单条运动指令包
struct CmdPackage {
    CmdType type = CmdType::Stop;
    V7d q = V7d::Zero();
    Pose pose{};
};

enum class MotionKind {
    None = 0,
    Stop = 1,
    ServoJ = 2,
    ServoP = 3,
    MovJ = 4,
    MovL = 5,
};
