#include "ecjc/controller.hpp"

#include <algorithm>
#include <cmath>

namespace ecjc {
namespace {

// ── 直通 ──────────────────────────────────────────────────────────────────
// 轨迹给定原样下发。用于验证链路、做标定实验。
class PassthroughController : public ControllerBase {
public:
    void reset() override {}
    void update(const ControlContext& c, ControlOutput* o) override {
        o->target_pos_deg = c.ref.pos_deg;
        o->target_vel_rpm = c.ref.vel_rpm;
        o->target_trq_Nm  = c.ref.trq_Nm;
    }
    void declareParams(ParameterRegistry&) override {}
    const char* id() const override { return "passthrough"; }
    const char* name() const override { return "直通 (Passthrough)"; }
    std::vector<OpMode> supportedModes() const override {
        return {OpMode::CSP, OpMode::CSV, OpMode::CST, OpMode::PP, OpMode::PV, OpMode::PT};
    }
};

/// PID 公共部分。积分带抗饱和（输出限幅时停止积分），
/// 微分对**测量值**求导而不是对误差求导——给定阶跃时不会产生微分冲击。
struct Pid {
    double integ = 0, last_meas = 0;
    bool   primed = false;
    void reset() { integ = 0; last_meas = 0; primed = false; }
    double step(double err, double meas, double kp, double ki, double kd,
                double dt, double out_min, double out_max) {
        double d = 0;
        if (primed && dt > 0) d = -(meas - last_meas) / dt;   // 负号：对测量求导
        last_meas = meas; primed = true;

        double u = kp * err + ki * integ + kd * d;
        if (u > out_max)      u = out_max;
        else if (u < out_min) u = out_min;
        else                  integ += err * dt;              // 仅未饱和时积分
        return u;
    }
};

// ── 位置环 PID（CST 输出力矩）─────────────────────────────────────────────
class PidPositionController : public ControllerBase {
public:
    void reset() override { pid_.reset(); }
    void declareParams(ParameterRegistry& r) override {
        r.add("pos_kp", "位置 Kp", "Nm/deg", 0.8, 0.0, 50.0, 0.01);
        r.add("pos_ki", "位置 Ki", "Nm/(deg·s)", 0.0, 0.0, 50.0, 0.01);
        r.add("pos_kd", "位置 Kd", "Nm·s/deg", 0.02, 0.0, 5.0, 0.001);
        r.add("trq_lim", "力矩限幅", "Nm", 5.0, 0.0, 20.0, 0.1);
    }
    void update(const ControlContext& c, ControlOutput* o) override {
        const double kp = c.p->get(0), ki = c.p->get(1), kd = c.p->get(2);
        const double lim = c.p->get(3);
        const double meas = c.act.output_pos_unwrapped_deg;
        const double err  = c.ref.pos_deg - meas;
        o->target_trq_Nm = pid_.step(err, meas, kp, ki, kd, c.dt, -lim, lim);
        o->target_pos_deg = c.ref.pos_deg;
    }
    const char* id() const override { return "pid_position"; }
    const char* name() const override { return "位置环 PID"; }
    std::vector<OpMode> supportedModes() const override { return {OpMode::CST}; }
private:
    Pid pid_;
};

// ── 速度环 PID（CST 输出力矩）─────────────────────────────────────────────
class PidVelocityController : public ControllerBase {
public:
    void reset() override { pid_.reset(); }
    void declareParams(ParameterRegistry& r) override {
        r.add("vel_kp", "速度 Kp", "Nm/rpm", 0.05, 0.0, 10.0, 0.001);
        r.add("vel_ki", "速度 Ki", "Nm/(rpm·s)", 0.5, 0.0, 50.0, 0.01);
        r.add("vel_kd", "速度 Kd", "Nm·s/rpm", 0.0, 0.0, 1.0, 0.001);
        r.add("trq_lim", "力矩限幅", "Nm", 5.0, 0.0, 20.0, 0.1);
    }
    void update(const ControlContext& c, ControlOutput* o) override {
        const double kp = c.p->get(0), ki = c.p->get(1), kd = c.p->get(2);
        const double lim = c.p->get(3);
        const double meas = c.act.motor_vel_rpm;
        const double err  = c.ref.vel_rpm - meas;
        o->target_trq_Nm = pid_.step(err, meas, kp, ki, kd, c.dt, -lim, lim);
        o->target_vel_rpm = c.ref.vel_rpm;
    }
    const char* id() const override { return "pid_velocity"; }
    const char* name() const override { return "速度环 PID"; }
    std::vector<OpMode> supportedModes() const override { return {OpMode::CST}; }
private:
    Pid pid_;
};

// ── 自研 CST 控制器骨架 ───────────────────────────────────────────────────
// 这是任务书第三十八节说的"未来在 CST 模式下实现自研控制算法"的接入点。
// 当前实现 = 带摩擦补偿项的速度环，参数已按自适应/神经网络控制的常用命名预留。
// 要换成 RBFNN / Backstepping / 预设性能控制，只改这个类的 update()，
// 声明好参数，GUI 会自动长出对应的调参控件。
class CstCustomController : public ControllerBase {
public:
    void reset() override { pid_.reset(); theta_ = 0.0; }
    void declareParams(ParameterRegistry& r) override {
        r.add("vel_kp",  "速度 Kp",     "Nm/rpm",     0.05, 0.0, 10.0, 0.001);
        r.add("vel_ki",  "速度 Ki",     "Nm/(rpm·s)", 0.30, 0.0, 50.0, 0.01);
        r.add("gamma",   "自适应增益 γ", "-",          1.0,  0.0, 100.0, 0.01);
        r.add("lambda",  "滑模系数 λ",   "-",          5.0,  0.0, 100.0, 0.01);
        r.add("nn_lr",   "NN 学习率",    "-",          0.01, 0.0, 1.0,  0.001);
        r.add("fric_c",  "库仑摩擦补偿", "Nm",         0.0,  0.0, 5.0,  0.01);
        r.add("trq_lim", "力矩限幅",     "Nm",         5.0,  0.0, 20.0, 0.1);
    }
    void update(const ControlContext& c, ControlOutput* o) override {
        const double kp = c.p->get(0), ki = c.p->get(1);
        const double gamma = c.p->get(2);
        const double fric_c = c.p->get(5), lim = c.p->get(6);

        const double meas = c.act.motor_vel_rpm;
        const double err  = c.ref.vel_rpm - meas;

        // 基础反馈
        double u = pid_.step(err, meas, kp, ki, 0.0, c.dt, -lim, lim);

        // 库仑摩擦前馈：按运动方向补一个常值
        if (std::fabs(meas) > 1.0) u += fric_c * (meas > 0 ? 1.0 : -1.0);

        // 一阶自适应项 θ̇ = γ·e（示意；真正的自适应律在这里替换）
        theta_ += gamma * err * c.dt * 1e-4;
        theta_ = std::clamp(theta_, -lim, lim);
        u += theta_;

        o->target_trq_Nm = std::clamp(u, -lim, lim);
        o->target_vel_rpm = c.ref.vel_rpm;
    }
    const char* id() const override { return "cst_custom"; }
    const char* name() const override { return "自研 CST 控制器"; }
    std::vector<OpMode> supportedModes() const override { return {OpMode::CST}; }
private:
    Pid pid_;
    double theta_ = 0;
};

}  // namespace

std::unique_ptr<ControllerBase> makeController(const std::string& id) {
    if (id == "passthrough")   return std::make_unique<PassthroughController>();
    if (id == "pid_position")  return std::make_unique<PidPositionController>();
    if (id == "pid_velocity")  return std::make_unique<PidVelocityController>();
    if (id == "cst_custom")    return std::make_unique<CstCustomController>();
    return nullptr;
}

std::vector<std::string> availableControllers() {
    return {"passthrough", "pid_position", "pid_velocity", "cst_custom"};
}

}  // namespace ecjc
