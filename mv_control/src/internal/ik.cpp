#include "ik.hpp"

#include "FxRobot.h"

#include <cmath>

namespace {

bool g_kine_ready = false;

void QToSdk7(const V7d& q, Vect7 out) {
    for (int i = 0; i < DOF; ++i) {
        out[i] = q(i) * R2D;  // SDK 关节角单位：度
    }
}

void SdkToQ7(const Vect7 q, V7d& out) {
    for (int i = 0; i < DOF; ++i) {
        out(i) = q[i] * D2R;  // 内部统一用弧度
    }
}

void PoseToMatrix4(const Pose& pose, Matrix4 mat) {
    const M3d rot = pose.quat.normalized().toRotationMatrix();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            mat[r][c] = rot(r, c);
        }
    }
    mat[0][3] = pose.pos.x();
    mat[1][3] = pose.pos.y();
    mat[2][3] = pose.pos.z();
    mat[3][0] = 0.0;
    mat[3][1] = 0.0;
    mat[3][2] = 0.0;
    mat[3][3] = 1.0;
}

void Matrix4ToPose(const Matrix4 mat, Pose& pose) {
    M3d rot;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            rot(r, c) = mat[r][c];
        }
    }
    pose.quat = Quat(rot);
    pose.pos.x() = mat[0][3];
    pose.pos.y() = mat[1][3];
    pose.pos.z() = mat[2][3];
}

}  // namespace

bool IkSolver::IsReady() { return g_kine_ready; }

bool IkSolver::InitFromCfg(const char* cfg_path) {
    if (g_kine_ready) {
        return true;
    }
    FX_INT32L type[2];
    FX_DOUBLE grv[2][3];
    FX_DOUBLE dh[2][8][4];
    FX_DOUBLE pnva[2][7][4];
    FX_DOUBLE bd[2][4][3];
    FX_DOUBLE mass[2][7];
    FX_DOUBLE mcp[2][7][3];
    FX_DOUBLE inertia[2][7][6];
    if (LOADMvCfg(const_cast<char*>(cfg_path), type, grv, dh, pnva, bd, mass, mcp,
                  inertia) != FX_TRUE) {
        return false;
    }
    for (int arm = 0; arm < 2; ++arm) {
        if (FX_Robot_Init_Type(arm, type[arm]) != FX_TRUE ||
            FX_Robot_Init_Kine(arm, dh[arm]) != FX_TRUE ||
            FX_Robot_Init_Lmt(arm, pnva[arm], bd[arm]) != FX_TRUE) {
            return false;
        }
    }
    g_kine_ready = true;
    return true;
}

bool IkSolver::Solve(int arm_serial, const Pose& target, const V7d& ref_q,
                     V7d& out_q) {
    FX_InvKineSolvePara sp{};
    PoseToMatrix4(target, sp.m_Input_IK_TargetTCP);
    QToSdk7(ref_q, sp.m_Input_IK_RefJoint);
    sp.m_Input_IK_ZSPType = 0;
    if (FX_Robot_Kine_IK(arm_serial, &sp) != FX_TRUE) {
        return false;
    }
    SdkToQ7(sp.m_Output_RetJoint, out_q);
    return true;
}

bool IkSolver::Forward(int arm_serial, const V7d& q, Pose& out_pose) {
    FX_DOUBLE q_sdk[DOF];
    QToSdk7(q, q_sdk);
    Matrix4 mat{};
    if (FX_Robot_Kine_FK(arm_serial, q_sdk, mat) != FX_TRUE) {
        return false;
    }
    Matrix4ToPose(mat, out_pose);
    return true;
}

bool IkSolver::Jacobian(int arm_serial, const V7d& q, Jacob& out_jacob) {
    FX_DOUBLE q_sdk[DOF];
    QToSdk7(q, q_sdk);
    FX_Jacobi jcb{};
    if (FX_Robot_Kine_Jacb(arm_serial, q_sdk, &jcb) != FX_TRUE) {
        return false;
    }
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < DOF; ++c) {
            // SDK 雅可比按 deg/s 求导，换算为 rad/s
            out_jacob(r, c) = jcb.m_Jcb[r][c] * R2D;
        }
    }
    return true;
}
