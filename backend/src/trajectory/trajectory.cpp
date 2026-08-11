#include "ecjc/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace ecjc {

const char* toString(TrajType t) {
    switch (t) {
        case TrajType::Constant:    return "constant";
        case TrajType::Sine:        return "sine";
        case TrajType::Ramp:        return "ramp";
        case TrajType::Triangle:    return "triangle";
        case TrajType::Trapezoidal: return "trapezoidal";
        case TrajType::CsvFile:     return "csv";
    }
    return "constant";
}

bool parseTrajType(const std::string& s, TrajType* out) {
    if (s == "constant")    { *out = TrajType::Constant;    return true; }
    if (s == "sine")        { *out = TrajType::Sine;        return true; }
    if (s == "ramp")        { *out = TrajType::Ramp;        return true; }
    if (s == "triangle")    { *out = TrajType::Triangle;    return true; }
    if (s == "trapezoidal") { *out = TrajType::Trapezoidal; return true; }
    if (s == "csv")         { *out = TrajType::CsvFile;     return true; }
    return false;
}

namespace {

// ── Constant ──────────────────────────────────────────────────────────────
class ConstantTraj : public TrajectoryBase {
public:
    explicit ConstantTraj(const TrajParams& p) : p_(p) {}
    void start(const JointState&, OpMode m) override { mode_ = m; }
    void eval(double, Setpoint* s) override {
        *s = Setpoint{};
        fill(s, mode_, p_.constant_value);
    }
    double duration() const override { return -1.0; }
    const char* name() const override { return "Constant"; }
private:
    TrajParams p_; OpMode mode_ = OpMode::CSV;
};

// ── Sine：q(t) = offset + A·sin(2πft + φ) ────────────────────────────────
class SineTraj : public TrajectoryBase {
public:
    explicit SineTraj(const TrajParams& p) : p_(p) {}
    void start(const JointState& q0, OpMode m) override {
        mode_ = m;
        // CSP 下以当前位置为基准叠加，避免 Run 瞬间位置突跳
        base_ = (m == OpMode::CSP) ? q0.output_pos_unwrapped_deg : 0.0;
    }
    void eval(double t, Setpoint* s) override {
        *s = Setpoint{};
        const double w = 2.0 * M_PI * p_.frequency_hz;
        const double ph = p_.phase_deg * M_PI / 180.0;
        fill(s, mode_, base_ + p_.offset + p_.amplitude * std::sin(w * t + ph));
        s->finished = (p_.duration_s > 0 && t >= p_.duration_s);
    }
    double duration() const override { return p_.duration_s; }
    const char* name() const override { return "Sine"; }
private:
    TrajParams p_; OpMode mode_ = OpMode::CSV; double base_ = 0;
};

// ── Ramp ──────────────────────────────────────────────────────────────────
class RampTraj : public TrajectoryBase {
public:
    explicit RampTraj(const TrajParams& p) : p_(p) {}
    void start(const JointState&, OpMode m) override { mode_ = m; }
    void eval(double t, Setpoint* s) override {
        *s = Setpoint{};
        const double T = p_.ramp_duration_s > 0 ? p_.ramp_duration_s : 1e-9;
        const double a = std::clamp(t / T, 0.0, 1.0);
        fill(s, mode_, p_.ramp_initial + (p_.ramp_final - p_.ramp_initial) * a);
        s->finished = (t >= T);
    }
    double duration() const override { return p_.ramp_duration_s; }
    const char* name() const override { return "Ramp"; }
private:
    TrajParams p_; OpMode mode_ = OpMode::CSV;
};

// ── Triangle ──────────────────────────────────────────────────────────────
class TriangleTraj : public TrajectoryBase {
public:
    explicit TriangleTraj(const TrajParams& p) : p_(p) {}
    void start(const JointState& q0, OpMode m) override {
        mode_ = m;
        base_ = (m == OpMode::CSP) ? q0.output_pos_unwrapped_deg : 0.0;
    }
    void eval(double t, Setpoint* s) override {
        *s = Setpoint{};
        const double f = p_.frequency_hz > 0 ? p_.frequency_hz : 1e-9;
        // 相位归一到 [0,1)，再折成 [-1,1] 的三角波
        double ph = std::fmod(t * f, 1.0);
        if (ph < 0) ph += 1.0;
        const double tri = (ph < 0.5) ? (4.0 * ph - 1.0) : (3.0 - 4.0 * ph);
        fill(s, mode_, base_ + p_.offset + p_.amplitude * tri);
        s->finished = (p_.duration_s > 0 && t >= p_.duration_s);
    }
    double duration() const override { return p_.duration_s; }
    const char* name() const override { return "Triangle"; }
private:
    TrajParams p_; OpMode mode_ = OpMode::CSP; double base_ = 0;
};

// ── Trapezoidal（梯形速度剖面，仅位置模式）────────────────────────────────
class TrapezoidTraj : public TrajectoryBase {
public:
    explicit TrapezoidTraj(const TrajParams& p) : p_(p) {}
    void start(const JointState& q0, OpMode m) override {
        mode_ = m;
        q0_ = q0.output_pos_unwrapped_deg;
        const double dist = p_.trapz_target_deg - q0_;
        sign_ = (dist >= 0) ? 1.0 : -1.0;
        D_ = std::fabs(dist);
        // 输出侧 rpm → deg/s
        vmax_ = std::fabs(p_.trapz_vmax_rpm) * 6.0;
        acc_  = std::fabs(p_.trapz_acc) > 1e-9 ? std::fabs(p_.trapz_acc) : 1e-9;
        dec_  = std::fabs(p_.trapz_dec) > 1e-9 ? std::fabs(p_.trapz_dec) : 1e-9;

        // 判断能否达到 vmax：加速段 + 减速段距离是否超过总距离
        const double d_acc = vmax_ * vmax_ / (2 * acc_);
        const double d_dec = vmax_ * vmax_ / (2 * dec_);
        if (d_acc + d_dec > D_) {
            // 三角形剖面：峰值速度由总距离反解
            vpk_ = std::sqrt(2 * D_ * acc_ * dec_ / (acc_ + dec_));
            t1_ = vpk_ / acc_;
            t2_ = t1_;                       // 无匀速段
            t3_ = t2_ + vpk_ / dec_;
        } else {
            vpk_ = vmax_;
            t1_ = vpk_ / acc_;
            const double d_const = D_ - d_acc - d_dec;
            t2_ = t1_ + d_const / vpk_;
            t3_ = t2_ + vpk_ / dec_;
        }
    }
    void eval(double t, Setpoint* s) override {
        *s = Setpoint{};
        double d;
        if (t <= 0)        d = 0;
        else if (t < t1_)  d = 0.5 * acc_ * t * t;
        else if (t < t2_)  d = 0.5 * vpk_ * t1_ + vpk_ * (t - t1_);
        else if (t < t3_) {
            const double td = t - t2_;
            d = 0.5 * vpk_ * t1_ + vpk_ * (t2_ - t1_) + vpk_ * td - 0.5 * dec_ * td * td;
        } else d = D_;
        s->pos_deg = q0_ + sign_ * d;
        s->finished = (t >= t3_);
        (void)mode_;
    }
    double duration() const override { return t3_; }
    const char* name() const override { return "Trapezoidal"; }
private:
    TrajParams p_; OpMode mode_ = OpMode::CSP;
    double q0_=0, sign_=1, D_=0, vmax_=0, acc_=1, dec_=1, vpk_=0, t1_=0, t2_=0, t3_=0;
};

// ── CSV 文件 ──────────────────────────────────────────────────────────────
// 支持两种表头：
//   time,target
//   time,target_position,target_velocity,target_torque
class CsvTraj : public TrajectoryBase {
public:
    CsvTraj(const TrajParams& p, std::vector<double> t, std::vector<Setpoint> v)
        : p_(p), t_(std::move(t)), v_(std::move(v)) {}
    void start(const JointState&, OpMode m) override { mode_ = m; }
    void eval(double t, Setpoint* s) override {
        *s = Setpoint{};
        if (t_.empty()) return;
        double tt = t;
        const double T = t_.back();
        if (p_.csv_loop && T > 0) { tt = std::fmod(t, T); if (tt < 0) tt += T; }
        if (tt <= t_.front()) { *s = v_.front(); }
        else if (tt >= T)     { *s = v_.back(); s->finished = !p_.csv_loop; }
        else {
            // 二分查找 + 线性插值，RT 安全：无分配、无磁盘
            const size_t i = static_cast<size_t>(
                std::upper_bound(t_.begin(), t_.end(), tt) - t_.begin());
            const size_t a = i - 1, b = i;
            const double w = (tt - t_[a]) / (t_[b] - t_[a]);
            s->pos_deg = v_[a].pos_deg + w * (v_[b].pos_deg - v_[a].pos_deg);
            s->vel_rpm = v_[a].vel_rpm + w * (v_[b].vel_rpm - v_[a].vel_rpm);
            s->trq_Nm  = v_[a].trq_Nm  + w * (v_[b].trq_Nm  - v_[a].trq_Nm);
        }
        (void)mode_;
    }
    double duration() const override { return p_.csv_loop ? -1.0 : (t_.empty() ? 0 : t_.back()); }
    const char* name() const override { return "CSV File"; }
private:
    TrajParams p_; OpMode mode_ = OpMode::CSP;
    std::vector<double> t_;
    std::vector<Setpoint> v_;
};

bool loadCsv(const std::string& path, OpMode mode,
             std::vector<double>* ts, std::vector<Setpoint>* vs, std::string* err) {
    std::ifstream f(path);
    if (!f) { *err = "无法打开轨迹文件: " + path; return false; }

    std::string line;
    bool header_done = false;
    int ncol = 0;
    size_t lineno = 0;
    while (std::getline(f, line)) {
        ++lineno;
        if (line.empty() || line[0] == '#') continue;
        // 表头：首字段不是数字就当表头跳过
        if (!header_done) {
            header_done = true;
            if (!line.empty() && (std::isalpha(static_cast<unsigned char>(line[0])))) {
                ncol = 1 + static_cast<int>(std::count(line.begin(), line.end(), ','));
                continue;
            }
        }
        std::stringstream ss(line);
        std::string cell;
        double col[4] = {0, 0, 0, 0};
        int n = 0;
        while (n < 4 && std::getline(ss, cell, ',')) {
            try { col[n] = std::stod(cell); }
            catch (...) { *err = "轨迹文件第 " + std::to_string(lineno) + " 行无法解析: " + cell; return false; }
            ++n;
        }
        if (n < 2) { *err = "轨迹文件第 " + std::to_string(lineno) + " 行列数不足（至少 time,target）"; return false; }
        if (ncol == 0) ncol = n;

        Setpoint sp{};
        if (n >= 4) { sp.pos_deg = col[1]; sp.vel_rpm = col[2]; sp.trq_Nm = col[3]; }
        else {
            switch (mode) {
                case OpMode::CSP: case OpMode::PP: sp.pos_deg = col[1]; break;
                case OpMode::CSV: case OpMode::PV: sp.vel_rpm = col[1]; break;
                case OpMode::CST: case OpMode::PT: sp.trq_Nm  = col[1]; break;
                default: sp.pos_deg = col[1]; break;
            }
        }
        if (!ts->empty() && col[0] < ts->back()) {
            *err = "轨迹文件时间列必须单调递增（第 " + std::to_string(lineno) + " 行）";
            return false;
        }
        ts->push_back(col[0]);
        vs->push_back(sp);
    }
    if (ts->size() < 2) { *err = "轨迹文件至少需要 2 个数据点"; return false; }
    return true;
}

}  // namespace

std::unique_ptr<TrajectoryBase> makeTrajectory(const TrajParams& p, std::string* err) {
    switch (p.type) {
        case TrajType::Constant:    return std::make_unique<ConstantTraj>(p);
        case TrajType::Sine:        return std::make_unique<SineTraj>(p);
        case TrajType::Ramp:        return std::make_unique<RampTraj>(p);
        case TrajType::Triangle:    return std::make_unique<TriangleTraj>(p);
        case TrajType::Trapezoidal: return std::make_unique<TrapezoidTraj>(p);
        case TrajType::CsvFile: {
            std::vector<double> ts; std::vector<Setpoint> vs;
            // 磁盘读取发生在这里 —— Run 之前，不在 RT 线程里
            if (!loadCsv(p.csv_path, OpMode::CSP, &ts, &vs, err)) return nullptr;
            return std::make_unique<CsvTraj>(p, std::move(ts), std::move(vs));
        }
    }
    *err = "未知轨迹类型";
    return nullptr;
}

}  // namespace ecjc
