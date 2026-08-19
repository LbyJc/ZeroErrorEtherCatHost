#include "test_framework.hpp"
#include "ecjc/trajectory.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

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

namespace {
// 在临时目录写一个轨迹 CSV，返回路径。与 test_config.cpp 同款做法。
std::string writeTrajCsv(const std::string& tag, const std::string& body) {
    auto p = std::filesystem::temp_directory_path() / ("ecjc_traj_test_" + tag + ".csv");
    FILE* f = fopen(p.c_str(), "w");
    fwrite(body.data(), 1, body.size(), f);
    fclose(f);
    return p.string();
}
}  // namespace

TEST(CSV轨迹_CSP起点平移到当前实测位置) {
    // 重复定位精度工况就地起测：CSV 里的位置是相对波形，
    // start() 时整体平移到当前实测位置，否则第一拍就是位置阶跃。
    TrajParams p;
    p.type = TrajType::CsvFile;
    p.csv_path = writeTrajCsv("csp_shift",
                              "time,target\n0,0\n1,-10\n2,-10\n3,0\n");
    std::string err;
    auto t = makeTrajectory(p, &err);
    CHECK(t != nullptr);

    JointState q0{};
    q0.output_pos_unwrapped_deg = 123.0;
    t->start(q0, OpMode::CSP);
    Setpoint s{};
    t->eval(0.0, &s);  CHECK_NEAR(s.pos_deg, 123.0, 1e-9);   // 起点 = 当前位置
    t->eval(1.5, &s);  CHECK_NEAR(s.pos_deg, 113.0, 1e-9);   // -10° 相对平移
    t->eval(3.0, &s);  CHECK_NEAR(s.pos_deg, 123.0, 1e-9);   // 回到测试点
    CHECK(s.finished);

    // 重新 start 要按新位置重新锁存
    q0.output_pos_unwrapped_deg = -7.0;
    t->start(q0, OpMode::CSP);
    t->eval(0.0, &s);  CHECK_NEAR(s.pos_deg, -7.0, 1e-9);
}

TEST(CSV轨迹_首点非零也平移到当前位置) {
    // 平移量 = q0 - 首点值：不管文件首点写多少，eval(0) 都等于当前位置
    TrajParams p;
    p.type = TrajType::CsvFile;
    p.csv_path = writeTrajCsv("csp_shift_nonzero",
                              "time,target\n0,5\n1,15\n");
    std::string err;
    auto t = makeTrajectory(p, &err);
    CHECK(t != nullptr);
    JointState q0{};
    q0.output_pos_unwrapped_deg = 100.0;
    t->start(q0, OpMode::CSP);
    Setpoint s{};
    t->eval(0.0, &s);  CHECK_NEAR(s.pos_deg, 100.0, 1e-9);
    t->eval(1.0, &s);  CHECK_NEAR(s.pos_deg, 110.0, 1e-9);
}

TEST(CSV轨迹_速度模式不平移) {
    // 四列速度工况（现有节点周期测试）行为必须原样：速度列照发，
    // 位置列不做任何平移（速度模式下 RT 也不读它）。
    TrajParams p;
    p.type = TrajType::CsvFile;
    p.csv_path = writeTrajCsv("csv_vel",
                              "time,target_position,target_velocity,target_torque\n"
                              "0,0,0,0\n4,0,5,0\n24,0,5,0\n");
    std::string err;
    auto t = makeTrajectory(p, &err);
    CHECK(t != nullptr);
    JointState q0{};
    q0.output_pos_unwrapped_deg = 55.0;
    t->start(q0, OpMode::CSV);
    Setpoint s{};
    t->eval(4.0, &s);
    CHECK_NEAR(s.vel_rpm, 5.0, 1e-9);
    CHECK_NEAR(s.pos_deg, 0.0, 1e-9);    // 不平移
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
