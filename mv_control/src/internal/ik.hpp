#pragma once

#include "common.hpp"

// 逆运动学封装，仅依赖 SDK kin 接口
class IkSolver {
public:
    static bool IsReady();
    static bool InitFromCfg(const char* cfg_path);
    static bool Solve(int arm_serial, const Pose& target, const V7d& ref_q,
                      V7d& out_q);
    static bool Forward(int arm_serial, const V7d& q, Pose& out_pose);
    static bool Jacobian(int arm_serial, const V7d& q, Jacob& out_jacob);
};
