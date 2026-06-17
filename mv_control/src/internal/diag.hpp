#pragma once

// 诊断日志：使能/错误类始终打印；细节需 export MV_DIAG=1

struct DiagEvent {
    int arm = -1;
    const char* category = "";
    const char* motion = "";
    const char* reason = "";
    int code = 0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double extra0 = 0.0;
    double extra1 = 0.0;
    double fk_x = 0.0;
    double fk_y = 0.0;
    double fk_z = 0.0;
};

using DiagEventCallback = void (*)(const DiagEvent& e, void* user);

namespace MvDiag {

bool Verbose();

void LogEnable(int arm_serial, const char* fmt, ...);
void LogMotion(const char* fmt, ...);
void LogVerbose(int arm_serial, const char* fmt, ...);

// 测试/遥操作：注册后 IK 等事件仅回调，不 printf
void SetDiagEventCallback(DiagEventCallback cb, void* user);
void ClearDiagEventCallback();

void EmitIkError(int arm, const char* reason, int code, double x, double y, double z,
                 double extra0 = 0.0, double extra1 = 0.0, double fk_x = 0.0,
                 double fk_y = 0.0, double fk_z = 0.0);
void EmitMotionError(int arm, const char* motion, const char* reason);

}  // namespace MvDiag
