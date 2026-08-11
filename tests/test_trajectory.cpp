#include "test_framework.hpp"
#include "ecjc/trajectory.hpp"

#include <cmath>

using namespace ecjc;

TEST(常值轨迹) {
    TrajParams p;
    p.type = TrajType::Constant;
    p.constant_value = 100.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    CHECK(t != nullptr);

    JointState q0{};
    t->start(q0, OpMode::CSV);
    Setpoint s{};
    t->eval(0.0, &s);
    CHECK_NEAR(s.vel_rpm, 100.0, 1e-9);
    t->eval(1000.0, &s);
    CHECK_NEAR(s.vel_rpm, 100.0, 1e-9);
    CHECK(t->duration() < 0);          // 无限
}

TEST(常值轨迹_按模式填对字段) {
    TrajParams p;
    p.type = TrajType::Constant;
    p.constant_value = 30.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    JointState q0{};
    Setpoint s{};

    t->start(q0, OpMode::CSP);
    t->eval(0, &s);
    CHECK_NEAR(s.pos_deg, 30.0, 1e-9);
    CHECK_NEAR(s.vel_rpm, 0.0, 1e-9);

    t->start(q0, OpMode::CST);
    t->eval(0, &s);
    CHECK_NEAR(s.trq_Nm, 30.0, 1e-9);
}

TEST(正弦轨迹) {
    TrajParams p;
    p.type = TrajType::Sine;
    p.offset = 0; p.amplitude = 10.0; p.frequency_hz = 1.0;
    p.phase_deg = 0; p.duration_s = 2.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    JointState q0{};
    t->start(q0, OpMode::CSV);

    Setpoint s{};
    t->eval(0.0, &s);   CHECK_NEAR(s.vel_rpm, 0.0, 1e-9);
    t->eval(0.25, &s);  CHECK_NEAR(s.vel_rpm, 10.0, 1e-6);    // 1Hz 的 1/4 周期 = 峰值
    t->eval(0.75, &s);  CHECK_NEAR(s.vel_rpm, -10.0, 1e-6);
    t->eval(2.5, &s);   CHECK(s.finished);
}

TEST(正弦轨迹_CSP以当前位置为基准) {
    // 避免 Run 瞬间从当前位置跳到 0 造成冲击
    TrajParams p;
    p.type = TrajType::Sine;
    p.amplitude = 5.0; p.frequency_hz = 1.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    JointState q0{};
    q0.output_pos_unwrapped_deg = 123.0;
    t->start(q0, OpMode::CSP);
    Setpoint s{};
    t->eval(0.0, &s);
    CHECK_NEAR(s.pos_deg, 123.0, 1e-6);     // 起点 = 当前位置，不是 0
}

TEST(斜坡轨迹) {
    TrajParams p;
    p.type = TrajType::Ramp;
    p.ramp_initial = 0; p.ramp_final = 100.0; p.ramp_duration_s = 4.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    JointState q0{};
    t->start(q0, OpMode::CSV);

    Setpoint s{};
    t->eval(0.0, &s);  CHECK_NEAR(s.vel_rpm, 0.0, 1e-9);
    t->eval(2.0, &s);  CHECK_NEAR(s.vel_rpm, 50.0, 1e-9);
    t->eval(4.0, &s);  CHECK_NEAR(s.vel_rpm, 100.0, 1e-9);
    CHECK(s.finished);
    t->eval(10.0, &s); CHECK_NEAR(s.vel_rpm, 100.0, 1e-9);   // 结束后保持终值
}

TEST(三角波轨迹) {
    TrajParams p;
    p.type = TrajType::Triangle;
    p.offset = 0; p.amplitude = 10.0; p.frequency_hz = 1.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    JointState q0{};
    t->start(q0, OpMode::CSV);

    Setpoint s{};
    t->eval(0.0, &s);   CHECK_NEAR(s.vel_rpm, -10.0, 1e-9);
    t->eval(0.25, &s);  CHECK_NEAR(s.vel_rpm, 0.0, 1e-9);
    t->eval(0.5, &s);   CHECK_NEAR(s.vel_rpm, 10.0, 1e-9);
    t->eval(0.75, &s);  CHECK_NEAR(s.vel_rpm, 0.0, 1e-9);
}

TEST(梯形轨迹_有匀速段) {
    TrajParams p;
    p.type = TrajType::Trapezoidal;
    p.trapz_target_deg = 360.0;
    p.trapz_vmax_rpm = 10.0;          // = 60 deg/s
    p.trapz_acc = 60.0;               // deg/s^2 → 1 秒到达 vmax
    p.trapz_dec = 60.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    JointState q0{};
    t->start(q0, OpMode::CSP);

    Setpoint s{};
    t->eval(0.0, &s);
    CHECK_NEAR(s.pos_deg, 0.0, 1e-6);

    // 终点必须精确到达目标，不能差一点
    t->eval(t->duration(), &s);
    CHECK_NEAR(s.pos_deg, 360.0, 1e-6);
    CHECK(s.finished);

    // 单调不回头
    double prev = -1e9;
    for (double x = 0; x <= t->duration(); x += 0.01) {
        t->eval(x, &s);
        CHECK(s.pos_deg >= prev - 1e-9);
        prev = s.pos_deg;
    }
}

TEST(梯形轨迹_距离太短退化成三角形) {
    TrajParams p;
    p.type = TrajType::Trapezoidal;
    p.trapz_target_deg = 1.0;         // 很短，够不到 vmax
    p.trapz_vmax_rpm = 100.0;
    p.trapz_acc = 100.0;
    p.trapz_dec = 100.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    JointState q0{};
    t->start(q0, OpMode::CSP);
    Setpoint s{};
    t->eval(t->duration(), &s);
    CHECK_NEAR(s.pos_deg, 1.0, 1e-6);   // 依然精确到位
}

TEST(梯形轨迹_反向) {
    TrajParams p;
    p.type = TrajType::Trapezoidal;
    p.trapz_target_deg = -90.0;
    p.trapz_vmax_rpm = 10.0;
    p.trapz_acc = 60.0; p.trapz_dec = 60.0;
    std::string err;
    auto t = makeTrajectory(p, &err);
    JointState q0{};
    q0.output_pos_unwrapped_deg = 0.0;
    t->start(q0, OpMode::CSP);
    Setpoint s{};
    t->eval(t->duration(), &s);
    CHECK_NEAR(s.pos_deg, -90.0, 1e-6);
}

TEST(CSV轨迹_文件不存在时报人话) {
    TrajParams p;
    p.type = TrajType::CsvFile;
    p.csv_path = "/nonexistent/nope.csv";
    std::string err;
    auto t = makeTrajectory(p, &err);
    CHECK(t == nullptr);
    CHECK(err.find("无法打开") != std::string::npos);
}
