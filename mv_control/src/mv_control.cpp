#include "config.hpp"
#include "mv_control.hpp"
#include "internal/ik.hpp"

#ifndef MV_CONTROL_SIM
#include "MarvinSDK.h"
#endif

#include <memory>

namespace {

constexpr int kLeftArmIdx = 0;
constexpr int kRightArmIdx = 1;

void V7dToArray(const V7d& q, double joints[DOF]) {
    for (int i = 0; i < DOF; ++i) {
        joints[i] = q(i);
    }
}

void V7dToSdkDeg(const V7d& q, double joints[DOF]) {
    for (int i = 0; i < DOF; ++i) {
        joints[i] = q(i) * R2D;
    }
}

}  // namespace

MVControl::MVControl() : left_(kLeftArmIdx), right_(kRightArmIdx) {}

MVControl::~MVControl() {
#ifndef MV_CONTROL_SIM
    if (connected_) {
        OnRelease();
        connected_ = false;
    }
#endif
}

bool MVControl::Init(uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4,
                     int log_switch, const char* kine_cfg) {
    if (connected_) {
        return true;
    }
    if (kine_cfg != nullptr && !IkSolver::InitFromCfg(kine_cfg)) {
        return false;
    }
#ifdef MV_CONTROL_SIM
    (void)ip1;
    (void)ip2;
    (void)ip3;
    (void)ip4;
    (void)log_switch;
    connected_ = true;
    if (!left_._Init() || !right_._Init()) {
        connected_ = false;
        return false;
    }
    return true;
#else
    if (log_switch != 0) {
        OnLogOn();
    }
    if (!OnLinkTo(ip1, ip2, ip3, ip4)) {
        return false;
    }
    if (!_ClearHwErrors()) {
        OnRelease();
        return false;
    }
    connected_ = true;
    if (!left_._Init() || !right_._Init()) {
        OnRelease();
        connected_ = false;
        return false;
    }
    return _ReadHwToRobots();
#endif
}

bool MVControl::InitFromConfig(const char* yaml_path) {
    auto cfg = std::make_unique<MvConfig>();
    if (!LoadMvConfig(yaml_path, *cfg)) {
        return false;
    }
    left_._ApplyArmConfig(cfg->left);
    right_._ApplyArmConfig(cfg->right);
    const char* kine = cfg->kine_cfg_path.empty() ? nullptr : cfg->kine_cfg_path.c_str();
    return Init(cfg->connect_ip[0], cfg->connect_ip[1], cfg->connect_ip[2], cfg->connect_ip[3],
                cfg->log_switch, kine);
}

void MVControl::Run() {
    if (!connected_) {
        return;
    }
#ifdef MV_CONTROL_SIM
    left_._SetRespState(left_.GetRefState());
    right_._SetRespState(right_.GetRefState());
#else
    _ReadHwToRobots();
#endif
    left_._Run();
    right_._Run();
#ifdef MV_CONTROL_SIM
    left_._SetRespState(left_.GetRefState());
    right_._SetRespState(right_.GetRefState());
#else
    _WriteRobotsToHw();
#endif
    left_._Detect();
    right_._Detect();
}

Robot& MVControl::Left() { return left_; }

Robot& MVControl::Right() { return right_; }

#ifndef MV_CONTROL_SIM
RobotState BuildRespState(const RT_OUT& rt_out) {
    RobotState rs;
    for (int i = 0; i < DOF; ++i) {
        rs.joint_state.q(i) = rt_out.m_FB_Joint_Pos[i] * D2R;
        rs.joint_state.v(i) = rt_out.m_FB_Joint_Vel[i] * D2R;
        rs.joint_state.tau(i) = rt_out.m_FB_Joint_SToq[i];
    }
    return rs;
}

bool MVControl::_ClearHwErrors() {
    OnClearSet();
    OnClearErr_A();
    OnClearErr_B();
    return OnSetSend();
}

bool MVControl::_ReadHwToRobots() {
    DCSS dcss;
    if (!OnGetBuf(&dcss)) {
        return false;
    }
    left_._SetRespState(BuildRespState(dcss.m_Out[kLeftArmIdx]));
    right_._SetRespState(BuildRespState(dcss.m_Out[kRightArmIdx]));
    return true;
}

bool MVControl::_WriteRobotsToHw() {
    bool has_cmd = false;
    double joints[DOF];

    OnClearSet();
    const auto should_write = [](StatusCode st) {
        return st == StatusCode::Running || st == StatusCode::Stop;
    };
    if (should_write(left_.status_code_) &&
        left_.control_mode_ == ControlMode::Position) {
        V7dToSdkDeg(left_.ref_rs_.joint_state.q, joints);
        OnSetJointCmdPos_A(joints);
        has_cmd = true;
    }
    if (should_write(right_.status_code_) &&
        right_.control_mode_ == ControlMode::Position) {
        V7dToSdkDeg(right_.ref_rs_.joint_state.q, joints);
        OnSetJointCmdPos_B(joints);
        has_cmd = true;
    }
    if (!has_cmd) {
        return true;
    }
    return OnSetSend();
}
#endif
