#include "mv_control.hpp"

#include <cstdio>
#include <filesystem>
#include <memory>
#include <random>
#include <string>

namespace {

constexpr const char* kKineCfg =
    "/home/yxc/tj_control/TJ_SDK/TJ_FX_ROBOT_CONTRL_SDK/DEMO_C++/srs.MvKDCfg";
constexpr const char* kDataDir = "/home/yxc/tj_control/data/test_movJ";
constexpr int kMaxSteps = 200000;
constexpr double kSampleDt = 0.001;

V7d RandomJointTarget(std::mt19937& rng) {
    // 随机目标关节角 [rad]，范围约 ±143°
    std::uniform_real_distribution<double> dist(-2.5, 2.5);
    V7d q;
    for (int i = 0; i < DOF; ++i) {
        q(i) = dist(rng);
    }
    return q;
}

void WriteJointHeader(FILE* f) {
    std::fprintf(f, "t");
    for (int i = 0; i < DOF; ++i) {
        std::fprintf(f, ",q%d", i);
    }
    for (int i = 0; i < DOF; ++i) {
        std::fprintf(f, ",v%d", i);
    }
    std::fprintf(f, "\n");
}

void WriteCartHeader(FILE* f) {
    std::fprintf(f, "t,px,py,pz,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz\n");
}

void WriteJointRow(FILE* f, double t, const RobotState& rs) {
    std::fprintf(f, "%.8f", t);
    for (int i = 0; i < DOF; ++i) {
        std::fprintf(f, ",%.8f", rs.joint_state.q(i));
    }
    for (int i = 0; i < DOF; ++i) {
        std::fprintf(f, ",%.8f", rs.joint_state.v(i));
    }
    std::fprintf(f, "\n");
}

void WriteCartRow(FILE* f, double t, const RobotState& rs) {
    const auto& p = rs.cart_state.pose;
    const auto& v = rs.cart_state.vel;
    std::fprintf(f,
                 "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
                 t, p.pos.x(), p.pos.y(), p.pos.z(), p.quat.w(), p.quat.x(),
                 p.quat.y(), p.quat.z(), v(0), v(1), v(2), v(3), v(4), v(5));
}

}  // namespace

int main() {
    std::error_code ec;
    std::filesystem::create_directories(kDataDir, ec);

    auto ctrl = std::make_unique<MVControl>();
    if (!ctrl->Init(0, 0, 0, 0, 0, kKineCfg)) {
        std::fprintf(stderr, "Init failed\n");
        return 1;
    }

    std::mt19937 rng(std::random_device{}());
    const V7d left_target = RandomJointTarget(rng);
    const V7d right_target = RandomJointTarget(rng);
    ctrl->Left().MovJ(left_target);
    ctrl->Right().MovJ(right_target);

    const std::string base(kDataDir);
    FILE* left_joint = std::fopen((base + "/left_joint.csv").c_str(), "w");
    FILE* right_joint = std::fopen((base + "/right_joint.csv").c_str(), "w");
    FILE* left_cart = std::fopen((base + "/left_cart.csv").c_str(), "w");
    FILE* right_cart = std::fopen((base + "/right_cart.csv").c_str(), "w");
    FILE* targets = std::fopen((base + "/targets.csv").c_str(), "w");
    if (!left_joint || !right_joint || !left_cart || !right_cart || !targets) {
        std::fprintf(stderr, "Open csv failed\n");
        return 1;
    }

    WriteJointHeader(left_joint);
    WriteJointHeader(right_joint);
    WriteCartHeader(left_cart);
    WriteCartHeader(right_cart);
    std::fprintf(targets, "arm");
    for (int i = 0; i < DOF; ++i) {
        std::fprintf(targets, ",q%d", i);
    }
    std::fprintf(targets, "\nleft");
    for (int i = 0; i < DOF; ++i) {
        std::fprintf(targets, ",%.8f", left_target(i));
    }
    std::fprintf(targets, "\nright");
    for (int i = 0; i < DOF; ++i) {
        std::fprintf(targets, ",%.8f", right_target(i));
    }
    std::fprintf(targets, "\n");

    bool recording = false;
    int step = 0;
    while (step < kMaxSteps) {
        ctrl->Run();
        const StatusCode ls = ctrl->Left().GetStatusCode();
        const StatusCode rs = ctrl->Right().GetStatusCode();
        if (ls == StatusCode::Error || rs == StatusCode::Error) {
            std::fprintf(stderr, "Robot error\n");
            return 1;
        }
        if (ls == StatusCode::Ready && rs == StatusCode::Ready) {
            break;
        }
        if (ls == StatusCode::Running || rs == StatusCode::Running) {
            recording = true;
        }
        if (recording) {
            const double t = step * kSampleDt;
            WriteJointRow(left_joint, t, ctrl->Left().GetRefState());
            WriteJointRow(right_joint, t, ctrl->Right().GetRefState());
            WriteCartRow(left_cart, t, ctrl->Left().GetRefState());
            WriteCartRow(right_cart, t, ctrl->Right().GetRefState());
        }
        ++step;
    }

    std::fclose(left_joint);
    std::fclose(right_joint);
    std::fclose(left_cart);
    std::fclose(right_cart);
    std::fclose(targets);
    std::printf("saved %d samples to %s\n", step, kDataDir);
    return 0;
}
