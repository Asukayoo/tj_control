#pragma once

#include "common.hpp"

// 运动学：KDL + URDF；FK/IK 均在 Link_Base 链系，cart_state 直接使用 KDL 结果
class IkSolver {
public:
    static bool IsReady();
    // urdf_path：双臂 URDF；基座 Link_Base，末端 TCP_Link_L / TCP_Link_R
    static bool InitFromUrdf(const char* urdf_path);
    // ref_q [rad]；target：Link_Base 系 TCP 位姿 [mm]
    static bool Solve(int arm_serial, const Pose& target, const V7d& ref_q, V7d& out_q);
    static bool Forward(int arm_serial, const V7d& q, Pose& out_pose);
    static bool Jacobian(int arm_serial, const V7d& q, Jacob& out_jacob);
};
