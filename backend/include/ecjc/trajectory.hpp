// trajectory.hpp —— 轨迹发生器。
//
// 约定：所有轨迹在 start() 时拿到当前关节状态 q0，以它为起点，
// 避免 Run 瞬间目标值从 0 跳到当前位置造成的冲击。
// CsvFile 在 start() 里完成全部磁盘读取，eval() 只做二分 + 插值——
// RT 线程内绝不碰磁盘。
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "ecjc/types.hpp"

namespace ecjc {

enum class TrajType : uint8_t {
    Constant, Sine, Ramp, Triangle, Trapezoidal, CsvFile
};
const char* toString(TrajType t);
bool parseTrajType(const std::string& s, TrajType* out);

struct TrajParams {
    TrajType type = TrajType::Constant;

    double constant_value = 0.0;   // 单位随模式：CSP=deg / CSV=rpm / CST=Nm

    double offset = 0.0, amplitude = 0.0, frequency_hz = 0.0;
    double phase_deg = 0.0, duration_s = -1.0;

    double ramp_initial = 0.0, ramp_final = 0.0, ramp_duration_s = 1.0;

    double trapz_target_deg = 0.0, trapz_vmax_rpm = 0.0;
    double trapz_acc = 0.0, trapz_dec = 0.0;

    std::string csv_path;
    bool csv_loop = false;
};

class TrajectoryBase {
public:
    virtual ~TrajectoryBase() = default;
    /// mode 决定 Setpoint 的哪一个字段被填充
    virtual void   start(const JointState& q0, OpMode mode) = 0;
    virtual void   eval(double t, Setpoint* out) = 0;
    virtual double duration() const = 0;      // <0 = 无限
    virtual const char* name() const = 0;

protected:
    /// 把标量值按当前模式塞进 Setpoint 的正确字段
    static void fill(Setpoint* s, OpMode mode, double v) {
        switch (mode) {
            case OpMode::CSP: case OpMode::PP:  s->pos_deg = v; break;
            case OpMode::CSV: case OpMode::PV:  s->vel_rpm = v; break;
            case OpMode::CST: case OpMode::PT:  s->trq_Nm  = v; break;
            default: break;
        }
    }
};

std::unique_ptr<TrajectoryBase> makeTrajectory(const TrajParams& p, std::string* err);

}  // namespace ecjc
