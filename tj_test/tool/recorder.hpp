#pragma once

#include "common.hpp"

#include <cstdint>
#include <vector>

class MVControl;

// 单拍记录（热路径仅 memcpy，无 IO）
struct CycleSample {
    uint32_t cycle = 0;
    uint16_t period_us = 0;
    uint16_t overrun_us = 0;
    uint8_t phase_packed = 0;
    uint8_t flags = 0;
    int8_t left_en = 0;
    int8_t right_en = 0;
    int8_t left_err = 0;
    int8_t right_err = 0;
    int8_t left_st = 0;
    int8_t right_st = 0;

    double l_ref_q[DOF]{};
    double l_resp_q[DOF]{};
    double l_ref_cart[7]{};
    double l_resp_cart[7]{};
    double r_ref_q[DOF]{};
    double r_resp_q[DOF]{};
    double r_ref_cart[7]{};
    double r_resp_cart[7]{};
};

enum CycleSampleFlags : uint8_t {
    kSampleHwSent = 1u << 0,
    kSampleClearFail = 1u << 1,
};

class RunRecorder {
public:
    void Reserve(std::size_t capacity);
    bool Push(const CycleSample& sample);
    void Clear();

    std::size_t Size() const { return samples_.size(); }
    bool Full() const { return samples_.size() >= capacity_; }
    const CycleSample& operator[](std::size_t i) const { return samples_[i]; }

private:
    std::vector<CycleSample> samples_;
    std::size_t capacity_ = 0;
};

void CopyCartPose(const RobotState& rs, double out[7]);

// 从 MVControl 填充单条 CycleSample（供 RunSession 与 decimated 记录复用）
void FillCycleSampleFromControl(CycleSample& s, MVControl& ctrl, int cycle,
                                uint8_t phase_packed, uint8_t flags);

bool ExportSessionCsv(const RunRecorder& recorder, const char* dir,
                      bool timing_with_phase);

// Pico UDP 订阅记录（50Hz，与 test_rt_teleop 记录节拍对齐）
struct PicoSample {
    uint32_t cycle = 0;
    uint64_t timestamp_ns = 0;
    uint32_t seq = 0;
    uint8_t valid = 0;
    uint8_t fresh = 0;
  // 位姿 [m] + 四元数 [qx,qy,qz,qw]（与 UDP / pico_data_receiver 一致）
    double right_pose[7]{};
    double left_pose[7]{};
    float left_trigger = 0.0f;
    float right_trigger = 0.0f;
};

class PicoRecorder {
public:
    void Reserve(std::size_t capacity);
    bool Push(const PicoSample& sample);
    void Clear();

    std::size_t Size() const { return samples_.size(); }
    bool Full() const { return samples_.size() >= capacity_; }
    const PicoSample& operator[](std::size_t i) const { return samples_[i]; }

private:
    std::vector<PicoSample> samples_;
    std::size_t capacity_ = 0;
};

bool ExportPicoCsv(const PicoRecorder& recorder, const char* dir);

// IK/运动诊断事件（test_rt_teleop 注册 MvDiag 回调后写入）
struct DiagEventSample {
    uint32_t cycle = 0;
    int8_t arm = -1;
    char category[16]{};
    char motion[24]{};
    char reason[32]{};
    int32_t code = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double extra0 = 0.0;
    double extra1 = 0.0;
    double fk_x = 0.0;
    double fk_y = 0.0;
    double fk_z = 0.0;
};

class DiagEventRecorder {
public:
    void Reserve(std::size_t capacity);
    bool Push(uint32_t cycle, int arm, const char* category, const char* motion,
              const char* reason, int code, double x, double y, double z, double extra0,
              double extra1, double fk_x, double fk_y, double fk_z);

    std::size_t Size() const { return samples_.size(); }
    bool Full() const { return samples_.size() >= capacity_; }
    const DiagEventSample& operator[](std::size_t i) const { return samples_[i]; }

private:
    std::vector<DiagEventSample> samples_;
    std::size_t capacity_ = 0;
};

bool ExportDiagEventsCsv(const DiagEventRecorder& recorder, const char* dir);
bool ExportDiagSummary(const DiagEventRecorder& recorder, const char* dir);
bool ExportTeleopDiagReport(const PicoRecorder& pico, const char* dir);
