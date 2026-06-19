#pragma once

#include <algorithm>
#include <cmath>

#include "common.hpp"

inline void PoseCompose(const Pose& parent, const Pose& child, Pose& out) {
    out.quat = (parent.quat * child.quat).normalized();
    out.pos = parent.pos + parent.quat * child.pos;
}

inline void PoseInverse(const Pose& p, Pose& out) {
    out.quat = p.quat.conjugate();
    out.pos = -(out.quat * p.pos);
}

inline void PicoToAbsTarget(const Pose& ref_pico, const Pose& robot_anchor,
                            const Pose& pico_pose, Pose& out) {
    // 平移：基座系左乘 Δt（同系 1:1 叠加）
    out.pos = robot_anchor.pos + (pico_pose.pos - ref_pico.pos);
    // 姿态：ΔR_world = R_now R_ref^T，左乘到 anchor（与 FLUZ 轴一致）
    out.quat = (pico_pose.quat * ref_pico.quat.conjugate() * robot_anchor.quat).normalized();
}

inline V3d QuatLogLocal(const Quat& q0, const Quat& q1) {
    Quat d = q0.conjugate() * q1;
    d.normalize();
    if (d.w() < 0.0) {
        d.coeffs() *= -1.0;
    }
    if (d.w() >= 1.0) {
        return V3d::Zero();
    }
    const double angle = 2.0 * std::acos(std::clamp(d.w(), -1.0, 1.0));
    const double s = std::sqrt(std::max(0.0, 1.0 - d.w() * d.w()));
    if (s < 1e-9) {
        return V3d::Zero();
    }
    return (angle / s) * d.vec();
}

inline Quat QuatExp(const V3d& w) {
    const double angle = w.norm();
    if (angle < 1e-9) {
        return Quat::Identity();
    }
    return Quat(Eigen::AngleAxisd(angle, w / angle));
}

inline void V7dToSdkDeg(const V7d& q, double joints[DOF]) {
    for (int i = 0; i < DOF; ++i) {
        joints[i] = q(i) * R2D;
    }
}

inline void V7dToArray(const V7d& src, double dst[DOF]) {
    for (int i = 0; i < DOF; ++i) {
        dst[i] = src(i);
    }
}
