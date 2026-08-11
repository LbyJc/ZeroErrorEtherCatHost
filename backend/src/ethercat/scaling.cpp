#include "ecjc/scaling.hpp"

#include <cmath>

namespace ecjc {

double wrapDeg360(double deg) {
    double r = std::fmod(deg, 360.0);
    if (r < 0.0) r += 360.0;
    return r;
}

void Scaling::toPhysical(const RawIo& raw, JointState* o) {
    const double dir_p = static_cast<double>(c_.position_direction);
    const double dir_v = static_cast<double>(c_.velocity_direction);
    const double dir_i = static_cast<double>(c_.current_direction);

    // ── 位置 ──────────────────────────────────────────────────────────
    // 0x6064 是输出侧计数。0x2240 是电机侧，走异步 SDO 取得。
    const int64_t out_counts   = output_uw_.update(raw.position_actual);
    const int64_t motor_counts = motor_uw_.update(raw.motor_position);

    o->output_pos_raw = raw.position_actual;
    o->motor_pos_raw  = raw.motor_position;

    o->output_pos_unwrapped_deg = dir_p * outputCountsToDeg(out_counts);
    o->motor_pos_unwrapped_deg  = dir_p * motorCountsToDeg(motor_counts);
    o->output_pos_deg = wrapDeg360(o->output_pos_unwrapped_deg);
    o->motor_pos_deg  = wrapDeg360(o->motor_pos_unwrapped_deg);

    // ── 速度 ──────────────────────────────────────────────────────────
    // 0x606C 单位 = 0x6064 的 counts/s（实测：写 0x60FF=7207 → 回读 7211.6）
    const double out_cps = dir_v * static_cast<double>(raw.velocity_actual) *
                           c_.velocity_gain_correction;
    o->output_vel_rpm = out_cps / c_.output_counts_per_rev * 60.0;
    o->motor_vel_rpm  = o->output_vel_rpm * c_.gear_ratio;

    // ── 电流 / 力矩 ───────────────────────────────────────────────────
    // 0x6078 / 0x6077 都是额定值的千分比
    o->current_A = dir_i * static_cast<double>(raw.current_actual) *
                   c_.current_scale * (c_.rated_current_mA / 1000.0);
    o->torque_Nm = static_cast<double>(raw.torque_actual) *
                   c_.torque_scale * (c_.rated_torque_mNm / 1000.0);
}

double Scaling::outputCountsToDeg(int64_t counts) const {
    return static_cast<double>(counts) / c_.output_counts_per_rev * 360.0;
}

double Scaling::motorCountsToDeg(int64_t counts) const {
    return static_cast<double>(counts) / c_.motor_counts_per_rev * 360.0;
}

int32_t Scaling::degToTargetPosition(double deg) const {
    // 0x607A 与 0x6064 同尺度（输出侧计数）
    const double counts = deg / 360.0 * c_.output_counts_per_rev *
                          static_cast<double>(c_.position_direction);
    return static_cast<int32_t>(std::lround(counts));
}

int32_t Scaling::rpmToTargetVelocity(double rpm, bool is_motor_side) const {
    // 先统一折算到输出侧 rpm，再换成输出侧 counts/s
    const double out_rpm = is_motor_side ? (rpm / c_.gear_ratio) : rpm;
    const double cps = out_rpm / 60.0 * c_.output_counts_per_rev /
                       c_.velocity_gain_correction *
                       static_cast<double>(c_.velocity_direction);
    return static_cast<int32_t>(std::lround(cps));
}

double Scaling::targetVelocityToRpm(int32_t raw, bool motor_side) const {
    const double out_rpm = static_cast<double>(raw) * c_.velocity_gain_correction /
                           c_.output_counts_per_rev * 60.0 *
                           static_cast<double>(c_.velocity_direction);
    return motor_side ? out_rpm * c_.gear_ratio : out_rpm;
}

int16_t Scaling::nmToTargetTorque(double nm) const {
    // 千分比，先按软限位裁剪
    double t = nm;
    if (t >  c_.torque_Nm_max) t =  c_.torque_Nm_max;
    if (t < -c_.torque_Nm_max) t = -c_.torque_Nm_max;
    const double per_mille = t / (c_.rated_torque_mNm / 1000.0) / c_.torque_scale;
    long v = std::lround(per_mille);
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;
    return static_cast<int16_t>(v);
}

double Scaling::countsPerMotorRpm() const {
    // 电机侧 1 rpm 对应多少 0x60FF 计数
    return c_.output_counts_per_rev / 60.0 / c_.gear_ratio / c_.velocity_gain_correction;
}

}  // namespace ecjc
