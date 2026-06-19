#include "recorder.hpp"

#include "internal/diag.hpp"
#include "mv_control.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <string>

namespace {

void WriteJointHeader(FILE* f) {
    std::fprintf(f, "cycle");
    for (int i = 0; i < DOF; ++i) {
        std::fprintf(f, ",q%d", i);
    }
    std::fprintf(f, "\n");
}

void WriteJointBody(FILE* f, const RunRecorder& rec, bool left, bool resp) {
    for (std::size_t i = 0; i < rec.Size(); ++i) {
        const CycleSample& s = rec[i];
        std::fprintf(f, "%u", s.cycle);
        const double* q = left ? (resp ? s.l_resp_q : s.l_ref_q)
                               : (resp ? s.r_resp_q : s.r_ref_q);
        for (int j = 0; j < DOF; ++j) {
            std::fprintf(f, ",%.8f", q[j]);
        }
        std::fprintf(f, "\n");
    }
}

void WriteCartBody(FILE* f, const RunRecorder& rec, bool left, bool resp) {
    for (std::size_t i = 0; i < rec.Size(); ++i) {
        const CycleSample& s = rec[i];
        const double* c = left ? (resp ? s.l_resp_cart : s.l_ref_cart)
                               : (resp ? s.r_resp_cart : s.r_ref_cart);
        std::fprintf(f, "%u,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n", s.cycle, c[0],
                     c[1], c[2], c[3], c[4], c[5], c[6]);
    }
}

bool OpenWriteClose(const std::string& path,
                    void (*writer)(FILE*, const RunRecorder&),
                    const RunRecorder& rec) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }
    writer(f, rec);
    std::fclose(f);
    return true;
}

void WriteLeftRefJoint(FILE* f, const RunRecorder& rec) {
    WriteJointHeader(f);
    WriteJointBody(f, rec, true, false);
}

void WriteLeftRespJoint(FILE* f, const RunRecorder& rec) {
    WriteJointHeader(f);
    WriteJointBody(f, rec, true, true);
}

void WriteRightRefJoint(FILE* f, const RunRecorder& rec) {
    WriteJointHeader(f);
    WriteJointBody(f, rec, false, false);
}

void WriteRightRespJoint(FILE* f, const RunRecorder& rec) {
    WriteJointHeader(f);
    WriteJointBody(f, rec, false, true);
}

void WriteLeftRefCart(FILE* f, const RunRecorder& rec) {
    std::fprintf(f, "cycle,px,py,pz,qw,qx,qy,qz\n");
    WriteCartBody(f, rec, true, false);
}

void WriteLeftRespCart(FILE* f, const RunRecorder& rec) {
    std::fprintf(f, "cycle,px,py,pz,qw,qx,qy,qz\n");
    WriteCartBody(f, rec, true, true);
}

void WriteRightRefCart(FILE* f, const RunRecorder& rec) {
    std::fprintf(f, "cycle,px,py,pz,qw,qx,qy,qz\n");
    WriteCartBody(f, rec, false, false);
}

void WriteRightRespCart(FILE* f, const RunRecorder& rec) {
    std::fprintf(f, "cycle,px,py,pz,qw,qx,qy,qz\n");
    WriteCartBody(f, rec, false, true);
}

void WriteTimingMovj(FILE* f, const RunRecorder& rec) {
    std::fprintf(f, "cycle,phase_packed,period_us,overrun_us\n");
    for (std::size_t i = 0; i < rec.Size(); ++i) {
        const CycleSample& s = rec[i];
        std::fprintf(f, "%u,%u,%u,%u\n", s.cycle, s.phase_packed, s.period_us,
                     s.overrun_us);
    }
}

void WriteTimingEnable(FILE* f, const RunRecorder& rec) {
    std::fprintf(f, "cycle,period_us,overrun_us\n");
    for (std::size_t i = 0; i < rec.Size(); ++i) {
        const CycleSample& s = rec[i];
        std::fprintf(f, "%u,%u,%u\n", s.cycle, s.period_us, s.overrun_us);
    }
}

}  // namespace

namespace {

void CopyJoints(const V7d& q, double out[DOF]) {
    for (int i = 0; i < DOF; ++i) {
        out[i] = q(i);
    }
}

}  // namespace

void FillCycleSampleFromControl(CycleSample& s, MVControl& ctrl, int cycle,
                                uint8_t phase_packed, uint8_t flags) {
    s.cycle = static_cast<uint32_t>(cycle);
    s.phase_packed = phase_packed;
    s.flags = flags;
    s.left_en = static_cast<int8_t>(ctrl.Left().GetEnableState());
    s.right_en = static_cast<int8_t>(ctrl.Right().GetEnableState());
    s.left_err = static_cast<int8_t>(ctrl.Left().GetErrorCode());
    s.right_err = static_cast<int8_t>(ctrl.Right().GetErrorCode());
    s.left_st = static_cast<int8_t>(ctrl.Left().GetStatusCode());
    s.right_st = static_cast<int8_t>(ctrl.Right().GetStatusCode());

    const RobotState& lr = ctrl.Left().GetRefState();
    const RobotState& lresp = ctrl.Left().GetRespState();
    const RobotState& rr = ctrl.Right().GetRefState();
    const RobotState& rresp = ctrl.Right().GetRespState();

    CopyJoints(lr.joint_state.q, s.l_ref_q);
    CopyJoints(lresp.joint_state.q, s.l_resp_q);
    CopyJoints(rr.joint_state.q, s.r_ref_q);
    CopyJoints(rresp.joint_state.q, s.r_resp_q);
    CopyCartPose(lr, s.l_ref_cart);
    CopyCartPose(lresp, s.l_resp_cart);
    CopyCartPose(rr, s.r_ref_cart);
    CopyCartPose(rresp, s.r_resp_cart);
}

void CopyCartPose(const RobotState& rs, double out[7]) {
    const auto& p = rs.cart_state.pose;
    out[0] = p.pos.x();
    out[1] = p.pos.y();
    out[2] = p.pos.z();
    out[3] = p.quat.w();
    out[4] = p.quat.x();
    out[5] = p.quat.y();
    out[6] = p.quat.z();
}

void RunRecorder::Reserve(std::size_t capacity) {
    capacity_ = capacity;
    samples_.clear();
    samples_.reserve(capacity);
}

bool RunRecorder::Push(const CycleSample& sample) {
    if (Full()) {
        return false;
    }
    samples_.push_back(sample);
    return true;
}

void RunRecorder::Clear() {
    samples_.clear();
}

void PicoRecorder::Reserve(std::size_t capacity) {
    capacity_ = capacity;
    samples_.clear();
    samples_.reserve(capacity);
}

bool PicoRecorder::Push(const PicoSample& sample) {
    if (Full()) {
        return false;
    }
    samples_.push_back(sample);
    return true;
}

void PicoRecorder::Clear() {
    samples_.clear();
}

namespace {

void WritePicoPoseCsvFields(FILE* f, const double pose[7]) {
  // SDK/UDP: x,y,z,qx,qy,qz,qw → CSV: x,y,z,qw,qx,qy,qz
    std::fprintf(f, ",%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f", pose[0], pose[1], pose[2],
                 pose[6], pose[3], pose[4], pose[5]);
}

void WritePicoTeleopCsv(FILE* f, const PicoRecorder& rec) {
    std::fprintf(f,
                 "cycle,timestamp_ns,seq,valid,fresh,"
                 "right_ctrl_x,right_ctrl_y,right_ctrl_z,"
                 "right_ctrl_qw,right_ctrl_qx,right_ctrl_qy,right_ctrl_qz,"
                 "left_ctrl_x,left_ctrl_y,left_ctrl_z,"
                 "left_ctrl_qw,left_ctrl_qx,left_ctrl_qy,left_ctrl_qz,"
                 "left_trigger,right_trigger\n");
    for (std::size_t i = 0; i < rec.Size(); ++i) {
        const PicoSample& s = rec[i];
        std::fprintf(f, "%u,%llu,%u,%u,%u", s.cycle,
                     static_cast<unsigned long long>(s.timestamp_ns), s.seq, s.valid,
                     s.fresh);
        WritePicoPoseCsvFields(f, s.right_pose);
        WritePicoPoseCsvFields(f, s.left_pose);
        std::fprintf(f, ",%.8f,%.8f\n", s.left_trigger, s.right_trigger);
    }
}

}  // namespace

bool ExportPicoCsv(const PicoRecorder& recorder, const char* dir) {
    if (dir == nullptr || recorder.Size() == 0) {
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = std::string(dir) + "/pico_teleop.csv";
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }
    WritePicoTeleopCsv(f, recorder);
    std::fclose(f);
    return true;
}

bool ExportSessionCsv(const RunRecorder& recorder, const char* dir,
                      bool timing_with_phase) {
    if (dir == nullptr || recorder.Size() == 0) {
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);

    const auto path = [&](const char* name) {
        return std::string(dir) + "/" + name;
    };

    struct Item {
        const char* name;
        void (*writer)(FILE*, const RunRecorder&);
    };
    const Item files[] = {
        {"left_ref_joint.csv", WriteLeftRefJoint},
        {"left_resp_joint.csv", WriteLeftRespJoint},
        {"left_ref_cart.csv", WriteLeftRefCart},
        {"left_resp_cart.csv", WriteLeftRespCart},
        {"right_ref_joint.csv", WriteRightRefJoint},
        {"right_resp_joint.csv", WriteRightRespJoint},
        {"right_ref_cart.csv", WriteRightRefCart},
        {"right_resp_cart.csv", WriteRightRespCart},
    };
    for (const Item& item : files) {
        if (!OpenWriteClose(path(item.name), item.writer, recorder)) {
            return false;
        }
    }
    return OpenWriteClose(path("timing.csv"),
                          timing_with_phase ? WriteTimingMovj : WriteTimingEnable,
                          recorder);
}

void DiagEventRecorder::Reserve(std::size_t capacity) {
    capacity_ = capacity;
    samples_.clear();
    samples_.reserve(capacity);
}

namespace {

void CopyField(char* dst, std::size_t n, const char* src) {
    if (src == nullptr) {
        dst[0] = '\0';
        return;
    }
    std::snprintf(dst, n, "%s", src);
}

bool PoseHasNan(const double pose[7]) {
    for (int i = 0; i < 7; ++i) {
        if (!std::isfinite(pose[i])) {
            return true;
        }
    }
    return false;
}

}  // namespace

bool DiagEventRecorder::Push(uint32_t cycle, int arm, const char* category,
                             const char* motion, const char* reason, int code, double x,
                             double y, double z, double extra0, double extra1, double fk_x,
                             double fk_y, double fk_z) {
    if (samples_.size() >= capacity_) {
        return false;
    }
    DiagEventSample s{};
    s.cycle = cycle;
    s.arm = static_cast<int8_t>(arm);
    CopyField(s.category, sizeof(s.category), category);
    CopyField(s.motion, sizeof(s.motion), motion);
    CopyField(s.reason, sizeof(s.reason), reason);
    s.code = code;
    s.x = x;
    s.y = y;
    s.z = z;
    s.extra0 = extra0;
    s.extra1 = extra1;
    s.fk_x = fk_x;
    s.fk_y = fk_y;
    s.fk_z = fk_z;
    samples_.push_back(s);
    return true;
}

bool ExportDiagEventsCsv(const DiagEventRecorder& recorder, const char* dir) {
    if (dir == nullptr) {
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = std::string(dir) + "/diag_events.csv";
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f,
                 "cycle,arm,category,motion,reason,code,x,y,z,extra0,extra1,fk_x,fk_y,"
                 "fk_z\n");
    for (std::size_t i = 0; i < recorder.Size(); ++i) {
        const DiagEventSample& s = recorder[i];
        std::fprintf(f, "%u,%d,%s,%s,%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                     s.cycle, static_cast<int>(s.arm), s.category, s.motion, s.reason,
                     s.code, s.x, s.y, s.z, s.extra0, s.extra1, s.fk_x, s.fk_y, s.fk_z);
    }
    std::fclose(f);
    return recorder.Size() > 0;
}

bool ExportDiagSummary(const DiagEventRecorder& recorder, const char* dir) {
    if (dir == nullptr) {
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = std::string(dir) + "/diag_summary.txt";
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }
    std::fprintf(f, "total_events=%zu\n", recorder.Size());
    struct Key {
        int arm;
        char category[16];
        char motion[24];
        char reason[32];
        bool operator<(const Key& o) const {
            if (arm != o.arm) {
                return arm < o.arm;
            }
            const int c = std::strcmp(category, o.category);
            if (c != 0) {
                return c < 0;
            }
            const int m = std::strcmp(motion, o.motion);
            if (m != 0) {
                return m < 0;
            }
            return std::strcmp(reason, o.reason) < 0;
        }
    };
    std::map<Key, uint64_t> counts;
    for (std::size_t i = 0; i < recorder.Size(); ++i) {
        const DiagEventSample& s = recorder[i];
        Key k{};
        k.arm = s.arm;
        CopyField(k.category, sizeof(k.category), s.category);
        CopyField(k.motion, sizeof(k.motion), s.motion);
        CopyField(k.reason, sizeof(k.reason), s.reason);
        counts[k]++;
    }
    std::fprintf(f, "type_counts:\n");
    for (const auto& [k, n] : counts) {
        std::fprintf(f, "  arm=%d category=%s motion=%s reason=%s count=%llu\n", k.arm,
                     k.category, k.motion, k.reason,
                     static_cast<unsigned long long>(n));
    }
    std::fclose(f);
    return true;
}

bool ExportTeleopDiagReport(const PicoRecorder& pico, const char* dir) {
    if (dir == nullptr || pico.Size() == 0) {
        return false;
    }
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = std::string(dir) + "/teleop_diag.txt";
    FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
        return false;
    }

    uint64_t valid_cnt = 0;
    uint64_t fresh_cnt = 0;
    uint64_t nan_pose_cnt = 0;
    uint64_t trigger_hi_cnt = 0;
    uint32_t first_valid_cycle = 0;
    bool seen_valid = false;
    double rx_min = 0.0;
    double rx_max = 0.0;
    bool rx_init = false;

    for (std::size_t i = 0; i < pico.Size(); ++i) {
        const PicoSample& s = pico[i];
        if (s.fresh) {
            ++fresh_cnt;
        }
        if (!s.valid) {
            continue;
        }
        ++valid_cnt;
        if (!seen_valid) {
            first_valid_cycle = s.cycle;
            seen_valid = true;
        }
        if (PoseHasNan(s.right_pose) || PoseHasNan(s.left_pose)) {
            ++nan_pose_cnt;
        }
        if (s.right_trigger >= 0.99f || s.left_trigger >= 0.99f) {
            ++trigger_hi_cnt;
        }
        if (!rx_init) {
            rx_min = s.right_pose[0];
            rx_max = s.right_pose[0];
            rx_init = true;
        } else {
            rx_min = std::min(rx_min, s.right_pose[0]);
            rx_max = std::max(rx_max, s.right_pose[0]);
        }
    }

    std::fprintf(f, "pico_samples=%zu\n", pico.Size());
    std::fprintf(f, "valid_samples=%llu fresh_samples=%llu\n",
                 static_cast<unsigned long long>(valid_cnt),
                 static_cast<unsigned long long>(fresh_cnt));
    std::fprintf(f, "first_valid_cycle=%u\n", first_valid_cycle);
    std::fprintf(f, "nan_pose_samples=%llu\n",
                 static_cast<unsigned long long>(nan_pose_cnt));
    std::fprintf(f, "trigger_ge_0.99_samples=%llu\n",
                 static_cast<unsigned long long>(trigger_hi_cnt));
    if (rx_init) {
        std::fprintf(f, "right_x_span_m=%.6f\n", rx_max - rx_min);
    }

    std::fprintf(f, "\n结论:\n");
    if (valid_cnt == 0) {
        std::fprintf(f,
                     "- 全程无 valid Pico 包：优先检查 pico_udp_publisher 是否已启动、端口是否一致。\n");
        std::fprintf(f, "- 订阅解析链路未见有效帧，问题在发布侧或未收到 UDP。\n");
    } else if (nan_pose_cnt > 0 && valid_cnt == nan_pose_cnt) {
        std::fprintf(f,
                     "- 有效帧位姿含 NaN/Inf：发布侧（SDK/滤波）数据异常，非 C++ 订阅解析问题。\n");
    } else if (nan_pose_cnt > 0) {
        std::fprintf(f,
                     "- 部分有效帧含 NaN/Inf（%llu/%llu）：发布侧偶发非法四元数/位姿。\n",
                     static_cast<unsigned long long>(nan_pose_cnt),
                     static_cast<unsigned long long>(valid_cnt));
        std::fprintf(f, "- 订阅已收到并解析 valid 帧，链路基本正常。\n");
    } else if (rx_init && (rx_max - rx_min) > 1e-4) {
        std::fprintf(f,
                     "- 位姿有变化且无数值异常：Pico 发布与 C++ 订阅正常；若仿真不动请查 servo_pico_trace。\n");
    } else {
        std::fprintf(f,
                     "- 有 valid 帧但位姿几乎不变：手柄未动或发布侧未更新 SDK 数据。\n");
    }

    MvDiag::ServoPicoTraceExport(f);
    const MvDiag::ServoPicoTrace& sp = MvDiag::ServoPicoTraceGet();
    std::fprintf(f, "\n[servo_pico_结论]\n");
    for (int arm = 0; arm < 2; ++arm) {
        const MvDiag::ServoPicoArmTrace& a = sp.arm[arm];
        std::fprintf(f, "arm%d:\n", arm);
        if (a.api_enter == 0) {
            std::fprintf(f, "  - 位姿未进入 ServoPByPico（TickServo 未调用或扳机未达阈值）\n");
            continue;
        }
        std::fprintf(f, "  - 位姿已进入 ServoPByPico：api_enter=%llu\n",
                     static_cast<unsigned long long>(a.api_enter));
        if (a.api_reject > 0) {
            std::fprintf(f,
                         "  - _CanAcceptCmd 拒绝 %llu 次（未使能/故障态）\n",
                         static_cast<unsigned long long>(a.api_reject));
        }
        if (a.stream_submit == 0) {
            std::fprintf(f, "  - 未提交 stream（异常：api_enter>0 但 stream_submit=0）\n");
        }
        if (a.motion_init == 0) {
            std::fprintf(f,
                         "  - 规划未启动：active_motion 未切到 ServoPByPico 或 motion_inited 未触发 InitPlan\n");
        } else if (a.session_init == 0) {
            std::fprintf(f,
                         "  - InitPlan 已调用 %llu 次但 session 未建立（FK 失败=%llu）\n",
                         static_cast<unsigned long long>(a.motion_init),
                         static_cast<unsigned long long>(a.session_fk_fail));
        } else {
            std::fprintf(f,
                         "  - 规划成功：session_init=%llu replan=%llu\n",
                         static_cast<unsigned long long>(a.session_init),
                         static_cast<unsigned long long>(a.replan));
        }
        if (a.run_session == 0 && a.session_init > 0) {
            std::fprintf(f, "  - session 已建立但 RunPlan 未执行（run_skip_session=%llu）\n",
                         static_cast<unsigned long long>(a.run_skip_session));
        }
        if (a.ik_ok == 0 && a.ik_fail == 0 && a.servo_run > 0) {
            std::fprintf(f, "  - servo_run=%llu 但无 IK 计数（未到 Solve 或 init_ 为 false）\n",
                         static_cast<unsigned long long>(a.servo_run));
        } else if (a.ik_ok > 0) {
            std::fprintf(f, "  - IK 成功 %llu 次", static_cast<unsigned long long>(a.ik_ok));
            if (a.ik_fail > 0) {
                std::fprintf(f, "，失败 %llu 次", static_cast<unsigned long long>(a.ik_fail));
            }
            std::fprintf(f, "\n");
        } else if (a.ik_fail > 0) {
            std::fprintf(f, "  - IK 全部失败：ik_fail=%llu\n",
                         static_cast<unsigned long long>(a.ik_fail));
        }
    }

    std::fclose(f);
    return true;
}
