#include "ik.hpp"
#include "diag.hpp"
#include "math.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <array>
#include <cstdlib>
#include <cstdio>
#include <memory>

namespace {

bool g_kine_ready = false;

constexpr double kM2Mm = 1000.0;
constexpr const char* kBaseLink = "Link_Base";
constexpr const char* kTipLinks[2] = {"TCP_Link_L", "TCP_Link_R"};
constexpr const char* kDefaultAmentPrefix = "/opt/ros/jazzy";

constexpr unsigned int kNrMaxIter = 100;
constexpr double kNrEps = 1e-6;  // KDL 内部单位：m / rad
constexpr double kIkPosTolMm = 5.0;
constexpr double kIkOriTolRad = 0.087;  // ~5 deg

// kdl_parser/urdf 经 pluginlib 查 ament 索引；未 source ROS 时 AMENT 为空会崩
void EnsureAmentPrefixPath() {
    const char* cur = std::getenv("AMENT_PREFIX_PATH");
    if (cur != nullptr && cur[0] != '\0') {
        return;
    }
    setenv("AMENT_PREFIX_PATH", kDefaultAmentPrefix, 1);
}

std::array<KDL::Chain, 2> g_chains;
std::array<std::unique_ptr<KDL::ChainFkSolverPos_recursive>, 2> g_fk;
std::array<std::unique_ptr<KDL::ChainIkSolverVel_pinv>, 2> g_ik_vel;
std::array<std::unique_ptr<KDL::ChainJntToJacSolver>, 2> g_jac;
std::array<std::unique_ptr<KDL::ChainIkSolverPos_NR>, 2> g_ik_nr;

void FrameToPose(const KDL::Frame& frame, Pose& pose) {
    pose.pos = V3d(frame.p.x(), frame.p.y(), frame.p.z()) * kM2Mm;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
    frame.M.GetQuaternion(x, y, z, w);
    pose.quat = Quat(w, x, y, z);
}

KDL::Frame PoseToFrame(const Pose& pose) {
    KDL::Frame frame;
    frame.p = KDL::Vector(pose.pos.x() / kM2Mm, pose.pos.y() / kM2Mm,
                          pose.pos.z() / kM2Mm);
    frame.M = KDL::Rotation::Quaternion(pose.quat.x(), pose.quat.y(),
                                        pose.quat.z(), pose.quat.w());
    return frame;
}

bool ForwardArm(int arm_serial, const V7d& q, Pose& out_arm) {
    KDL::JntArray q_kdl(DOF);
    for (int i = 0; i < DOF; ++i) {
        q_kdl(i) = q(i);
    }
    KDL::Frame frame;
    if (g_fk[arm_serial]->JntToCart(q_kdl, frame) < 0) {
        return false;
    }
    FrameToPose(frame, out_arm);
    return true;
}

bool JacobianArm(int arm_serial, const V7d& q, Jacob& out_jacob) {
    KDL::JntArray q_kdl(DOF);
    for (int i = 0; i < DOF; ++i) {
        q_kdl(i) = q(i);
    }
    KDL::Jacobian jac_kdl(DOF);
    if (g_jac[arm_serial]->JntToJac(q_kdl, jac_kdl) < 0) {
        return false;
    }
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < DOF; ++c) {
            double v = jac_kdl(r, c);
            if (r < 3) {
                v *= kM2Mm;  // m/rad -> mm/rad
            }
            out_jacob(r, c) = v;
        }
    }
    return true;
}

// FK 验误差：NR 返回成功但位姿未到位也视为 IK 失败
bool VerifyIkPose(int arm_serial, const Pose& target_arm, const V7d& q) {
    alignas(16) Pose fk;
    if (!ForwardArm(arm_serial, q, fk)) {
        MvDiag::EmitIkError(arm_serial, "FK_verify", 0, target_arm.pos.x(),
                            target_arm.pos.y(), target_arm.pos.z());
        return false;
    }
    const double pos_err = (fk.pos - target_arm.pos).norm();
    const double ori_err = QuatLogLocal(fk.quat, target_arm.quat).norm();
    if (pos_err > kIkPosTolMm || ori_err > kIkOriTolRad) {
        MvDiag::EmitIkError(arm_serial, "pose_tol", 0, target_arm.pos.x(),
                            target_arm.pos.y(), target_arm.pos.z(), pos_err, ori_err,
                            fk.pos.x(), fk.pos.y(), fk.pos.z());
        return false;
    }
    return true;
}

// KDL NR 位置 IK：目标与 ref 均在臂系
bool SolveNrArm(int arm_serial, const Pose& target_arm, const V7d& ref_q,
                V7d& out_q) {
    KDL::JntArray q_in(DOF);
    KDL::JntArray q_out(DOF);
    for (int i = 0; i < DOF; ++i) {
        q_in(i) = ref_q(i);
    }
    const KDL::Frame target = PoseToFrame(target_arm);
    const int ret = g_ik_nr[arm_serial]->CartToJnt(q_in, target, q_out);
    if (ret < 0) {
        MvDiag::EmitIkError(arm_serial, "kdl_nr", ret, target_arm.pos.x(),
                            target_arm.pos.y(), target_arm.pos.z());
        return false;
    }
    for (int i = 0; i < DOF; ++i) {
        out_q(i) = q_out(i);
    }
    return VerifyIkPose(arm_serial, target_arm, out_q);
}

}  // namespace

bool IkSolver::IsReady() { return g_kine_ready; }

bool IkSolver::InitFromUrdf(const char* urdf_path) {
    if (g_kine_ready) {
        return true;
    }
    EnsureAmentPrefixPath();
    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_path, tree)) {
        return false;
    }
    for (int arm = 0; arm < 2; ++arm) {
        if (!tree.getChain(kBaseLink, kTipLinks[arm], g_chains[arm])) {
            return false;
        }
        if (g_chains[arm].getNrOfJoints() != DOF) {
            return false;
        }
        g_fk[arm] =
            std::make_unique<KDL::ChainFkSolverPos_recursive>(g_chains[arm]);
        g_ik_vel[arm] =
            std::make_unique<KDL::ChainIkSolverVel_pinv>(g_chains[arm]);
        g_jac[arm] = std::make_unique<KDL::ChainJntToJacSolver>(g_chains[arm]);
        g_ik_nr[arm] = std::make_unique<KDL::ChainIkSolverPos_NR>(
            g_chains[arm], *g_fk[arm], *g_ik_vel[arm], kNrMaxIter, kNrEps);
    }
    g_kine_ready = true;
    return true;
}

bool IkSolver::Solve(int arm_serial, const Pose& target, const V7d& ref_q,
                     V7d& out_q) {
    return SolveNrArm(arm_serial, target, ref_q, out_q);
}

bool IkSolver::Forward(int arm_serial, const V7d& q, Pose& out_pose) {
    return ForwardArm(arm_serial, q, out_pose);
}

bool IkSolver::Jacobian(int arm_serial, const V7d& q, Jacob& out_jacob) {
    return JacobianArm(arm_serial, q, out_jacob);
}
