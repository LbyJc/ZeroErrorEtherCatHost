// scaling.hpp —— 原始计数 ↔ 真实物理量。
//
// 纯计算、无副作用、可单元测试（任务书第五十二节第 15 条）。
// 所有系数来自 scaling.yaml，**一个都不许写死在别处**（第三十五节）。
//
// 本驱动器的一个事实：0x6064 / 0x607A / 0x60FF / 0x606C 全部是**输出侧**计数，
// 实测 Δ0x6064 与 Δ0x20A0（输出侧编码器）逐拍相等，而电机侧 0x2240 是它的 30.24 倍。
// 所以位置/速度的换算基准是输出侧编码器，电机侧数值由减速比换算得到。
#pragma once

#include <cstdint>
#include "ecjc/types.hpp"

namespace ecjc {

struct ScalingConfig {
    double motor_counts_per_rev  = 131072.0;   // 2^17
    double output_counts_per_rev = 524288.0;   // 2^19
    bool   resolution_verified   = false;
    double gear_ratio            = 121.0;

    double velocity_gain_correction = 1.0;

    double rated_current_mA = 6300.0;
    double rated_torque_mNm = 31000.0;
    double current_scale    = 0.001;
    double torque_scale     = 0.001;

    int    position_direction = 1;
    int    velocity_direction = 1;
    int    current_direction  = 1;

    // 速度目标以哪一侧为准（GUI 输入框的标签随之变化）
    bool   target_velocity_is_motor_side = true;

    // 编码器回绕模数。0 = 32 位累加多圈量（本机硬件）；
    // 若换成单圈回绕的编码器，填 counts_per_rev。见 PositionUnwrapper 注释。
    int64_t motor_position_modulus = 0;
    int64_t output_position_modulus = 0;

    double motor_velocity_rpm_max  = 3000.0;
    double output_velocity_rpm_max = 25.0;
    double torque_Nm_max           = 20.0;
    double current_A_max           = 6.3;
};

/// 多圈展开器。
///
/// 两种编码器行为都要支持：
///
///  A) **32 位累加多圈量**（本驱动器的 0x6064 / 0x2240 实测就是这种，
///     跑到 573956 也不回绕）→ 用 int32 差值累加即可，
///     二补数减法天然处理 2^32 处的溢出。设 modulus = 0。
///
///  B) **单圈回绕**（值域 0..CPR-1，转满一圈跳回 0）→ 差值法在这里**不成立**：
///     524000 → 288 的真实位移是 +576，但按 int32 差值算是 -523712。
///     所以需要按模数把超过半圈的跳变折回来。设 modulus = CPR。
///
/// modulus 由 scaling.yaml 配置，默认 0（本机硬件适用）。
class PositionUnwrapper {
public:
    void setModulus(int64_t m) { modulus_ = m; }

    void reset(int32_t raw) { last_ = raw; acc_ = raw; primed_ = true; }

    int64_t update(int32_t raw) {
        if (!primed_) { reset(raw); return acc_; }
        int64_t delta = static_cast<int32_t>(
            static_cast<uint32_t>(raw) - static_cast<uint32_t>(last_));
        if (modulus_ > 0) {
            // 位移不可能超过半圈/周期，超过就是回绕，折回来
            const int64_t half = modulus_ / 2;
            while (delta >  half) delta -= modulus_;
            while (delta < -half) delta += modulus_;
        }
        acc_ += delta;
        last_ = raw;
        return acc_;
    }
    int64_t counts() const { return acc_; }
    bool primed() const { return primed_; }

private:
    int32_t last_ = 0;
    int64_t acc_  = 0;
    int64_t modulus_ = 0;
    bool    primed_ = false;
};

/// [0,360) 归一化。用于 GUI 显示，底层始终保留 unwrapped。
double wrapDeg360(double deg);

class Scaling {
public:
    explicit Scaling(const ScalingConfig& c) : c_(c) { reset(); }
    const ScalingConfig& config() const { return c_; }

    void reset() {
        motor_uw_ = PositionUnwrapper{};
        output_uw_ = PositionUnwrapper{};
        motor_uw_.setModulus(c_.motor_position_modulus);
        output_uw_.setModulus(c_.output_position_modulus);
    }

    /// 把一拍原始 PDO 转成物理量。
    void toPhysical(const RawIo& raw, JointState* out);

    // ── 单向换算（目标值下发时用）────────────────────────────────────────
    /// 输出侧角度 → 0x607A 原始计数
    int32_t degToTargetPosition(double deg) const;
    /// 转速 → 0x60FF 原始值。is_motor_side 决定输入是电机侧还是输出侧 rpm。
    int32_t rpmToTargetVelocity(double rpm, bool is_motor_side) const;
    /// 力矩 Nm → 0x6071 千分比
    int16_t nmToTargetTorque(double nm) const;

    // ── 反向换算 ────────────────────────────────────────────────────────
    double targetVelocityToRpm(int32_t raw, bool motor_side) const;
    double outputCountsToDeg(int64_t counts) const;
    double motorCountsToDeg(int64_t counts) const;

    /// 电机侧 rpm ↔ 0x60FF 原始值 的比例，GUI 用来显示"当前 1 rpm = ? 计数"
    double countsPerMotorRpm() const;

private:
    ScalingConfig c_;
    PositionUnwrapper motor_uw_, output_uw_;
};

}  // namespace ecjc
