// 物理量换算。用例里的数字都是 2026-08-10 真机实测值，
// 这样这些测试同时起到"标定结果回归保护"的作用：
// 谁改坏了换算，这里立刻红。
#include "test_framework.hpp"
#include "ecjc/scaling.hpp"

using namespace ecjc;

static ScalingConfig realCfg() {
    ScalingConfig c;
    c.motor_counts_per_rev = 131072.0;    // 2^17
    c.output_counts_per_rev = 524288.0;   // 2^19
    c.gear_ratio = 121.0;
    c.rated_current_mA = 6300.0;
    c.rated_torque_mNm = 31000.0;
    return c;
}

TEST(wrap到0_360) {
    CHECK_NEAR(wrapDeg360(0.0), 0.0, 1e-9);
    CHECK_NEAR(wrapDeg360(359.9), 359.9, 1e-9);
    CHECK_NEAR(wrapDeg360(360.0), 0.0, 1e-9);
    CHECK_NEAR(wrapDeg360(361.5), 1.5, 1e-9);
    CHECK_NEAR(wrapDeg360(-1.0), 359.0, 1e-9);      // 负角必须落在 [0,360)
    CHECK_NEAR(wrapDeg360(-721.0), 359.0, 1e-9);
}

TEST(多圈展开_不丢圈数) {
    PositionUnwrapper u;
    u.reset(0);
    CHECK_EQ(u.update(1000), 1000);
    CHECK_EQ(u.update(524288), 524288);        // 转满一圈
    CHECK_EQ(u.update(1048576), 1048576);      // 两圈
}

TEST(多圈展开_扛得住int32回绕) {
    // 这是用差值累加而不是直接取绝对值的理由：
    // 编码器计数溢出 int32 时，绝对值会跳变，差值不会。
    PositionUnwrapper u;
    u.reset(2147483000);                       // 接近 INT32_MAX
    const int64_t a = u.update(-2147483296);   // 溢出后变成负数，实际只前进了 1000
    CHECK_EQ(a, 2147484000LL);
}

TEST(多圈展开_单圈回绕编码器需要设模数) {
    // 不设模数时，差值法只处理 2^32 回绕，对 524288 模数的单圈编码器是**错的**
    PositionUnwrapper naive;
    naive.reset(524000);
    CHECK_EQ(naive.update(288), 288);          // 被当成倒退了 52 万计数

    // 设了模数才正确：524000 → 288 的真实位移是 +576
    PositionUnwrapper u;
    u.setModulus(524288);
    u.reset(524000);
    CHECK_EQ(u.update(288), 524576);           // 524000 + 576
    // 反向回绕同样要处理
    CHECK_EQ(u.update(524000), 524000);
}

TEST(多圈展开_本机硬件用零模数) {
    // 本驱动器 0x6064/0x2240 是 32 位累加量，跑到 573956 也不回绕，
    // 所以模数必须是 0——设成 524288 反而会把正常的连续计数折断
    PositionUnwrapper u;                        // 默认 modulus = 0
    u.reset(500000);
    CHECK_EQ(u.update(573956), 573956);         // 越过 524288 依然线性
}

TEST(目标速度换算_对齐实测值) {
    Scaling s(realCfg());
    // 实测：0x60FF = 7207 → 电机侧 99.89 rpm
    const double rpm = s.targetVelocityToRpm(7207, /*motor_side=*/true);
    CHECK_NEAR(rpm, 99.8, 0.5);

    // 反向：电机侧 100 rpm 应该落在 7207 附近
    const int32_t raw = s.rpmToTargetVelocity(100.0, true);
    CHECK(raw > 7150 && raw < 7290);

    // 每 rpm 对应的计数：524288/60/121 ≈ 72.2
    CHECK_NEAR(s.countsPerMotorRpm(), 72.2, 0.2);
}

TEST(输出侧转速换算) {
    Scaling s(realCfg());
    // 输出侧 1 rpm = 524288/60 = 8738.1 counts/s
    const int32_t raw = s.rpmToTargetVelocity(1.0, /*motor_side=*/false);
    CHECK_NEAR(raw, 8738, 2);
}

TEST(位置换算_一圈等于360度) {
    Scaling s(realCfg());
    CHECK_NEAR(s.outputCountsToDeg(524288), 360.0, 1e-6);
    CHECK_NEAR(s.motorCountsToDeg(131072), 360.0, 1e-6);
    CHECK_EQ(s.degToTargetPosition(360.0), 524288);
    CHECK_EQ(s.degToTargetPosition(90.0), 131072);
}

TEST(力矩换算_千分比) {
    Scaling s(realCfg());
    // 额定 31 Nm，千分比：1000 → 31 Nm
    CHECK_EQ(s.nmToTargetTorque(31.0 * 0.5), 500);
    CHECK_EQ(s.nmToTargetTorque(0.0), 0);
    // 超过软限位 20 Nm 要被裁掉
    const int16_t big = s.nmToTargetTorque(1000.0);
    CHECK_NEAR(big, 20.0 / 31.0 * 1000.0, 2.0);
}

TEST(PDO到物理量_全链路) {
    Scaling s(realCfg());
    RawIo raw{};
    raw.position_actual = 131072;      // 输出侧 1/4 圈
    raw.motor_position  = 0;
    raw.velocity_actual = 7211;        // 实测 0x60FF=7207 时的回读值
    raw.current_actual  = 100;         // 千分比 → 0.63 A
    raw.torque_actual   = 40;          // 千分比 → 1.24 Nm

    JointState j{};
    s.toPhysical(raw, &j);

    CHECK_NEAR(j.output_pos_deg, 90.0, 1e-6);
    CHECK_NEAR(j.output_vel_rpm, 7211.0 / 524288.0 * 60.0, 1e-6);
    CHECK_NEAR(j.motor_vel_rpm, j.output_vel_rpm * 121.0, 1e-6);
    CHECK_NEAR(j.motor_vel_rpm, 99.85, 0.3);      // ← 与真机一致
    CHECK_NEAR(j.current_A, 0.63, 1e-6);
    CHECK_NEAR(j.torque_Nm, 1.24, 1e-6);
}

// CSP 位置阶跃兜底：偏差在阈值内原样放行，不触发。
TEST(csp阶跃兜底_阈值内不动作) {
    const auto r = cspTargetJumpGuard(/*target=*/93.0, /*current=*/90.0, /*max=*/5.0);
    CHECK(!r.triggered);
    CHECK_NEAR(r.safe_target_deg, 93.0, 1e-9);
    CHECK_NEAR(r.err_deg, 3.0, 1e-9);
}

// 偏差超过阈值：必须钳到当前实测位置，绝不能把原目标放行下去。
TEST(csp阶跃兜底_超阈值钳到实测位置) {
    const auto r = cspTargetJumpGuard(/*target=*/170.0, /*current=*/90.0, /*max=*/5.0);
    CHECK(r.triggered);
    CHECK_NEAR(r.safe_target_deg, 90.0, 1e-9);   // 钳到 current，不是 target
    CHECK_NEAR(r.err_deg, 80.0, 1e-9);
}

// 负方向偏差同样要按幅值判断，不能只查符号。
TEST(csp阶跃兜底_负方向偏差按幅值判断) {
    const auto r = cspTargetJumpGuard(/*target=*/10.0, /*current=*/90.0, /*max=*/5.0);
    CHECK(r.triggered);
    CHECK_NEAR(r.safe_target_deg, 90.0, 1e-9);
    CHECK_NEAR(r.err_deg, -80.0, 1e-9);
}

TEST(方向取反) {
    ScalingConfig c = realCfg();
    c.velocity_direction = -1;
    Scaling s(c);
    RawIo raw{};
    raw.velocity_actual = 8738;
    JointState j{};
    s.toPhysical(raw, &j);
    CHECK(j.output_vel_rpm < 0);
}

// 力矩必须和电流一样受 current_direction 影响，否则 P = τ·ω 算出来符号是错的
TEST(torque_respects_direction_sign) {
    ScalingConfig c;
    c.rated_torque_mNm = 31000.0;
    c.torque_scale = 0.001;
    c.current_direction = -1;
    Scaling s(c);
    RawIo raw{};
    raw.torque_actual = 500;              // 千分之 500 = 半额定
    JointState j{};
    s.toPhysical(raw, &j);
    CHECK_NEAR(j.torque_Nm, -15.5, 1e-6);  // 方向为负 ⇒ 力矩为负
}

// 0x2241 换算：1 count = 21600 / (131072 × 121) 角分（输出侧）
TEST(twist_counts_to_arcmin) {
    ScalingConfig c;                        // 默认 131072 / 524288 / 121
    Scaling s(c);
    CHECK_NEAR(s.twistCountsToArcmin(1), 1.3619417e-3, 1e-9);
    CHECK_NEAR(s.twistCountsToArcmin(267), 0.36363, 1e-4);  // 手册表22-2 的零扭矩开口量级
}

// 恒等式 Δ = C_m − 30.25 × C_o，30.25 = 121/4 是精确有理数
TEST(expected_twist_identity_uses_exact_30p25) {
    ScalingConfig c;
    Scaling s(c);
    CHECK_NEAR(s.expectedTwistFromPositions(30250, 1000), 0.0, 1e-9);
    CHECK_NEAR(s.expectedTwistFromPositions(30251, 1000), 1.0, 1e-9);
}

// rad 换算：计划附录 A.1 要求 theta_out_rad
TEST(output_counts_to_rad) {
    ScalingConfig c;
    Scaling s(c);
    CHECK_NEAR(s.outputCountsToRad(1), 1.198422e-5, 1e-11);
    CHECK_NEAR(s.outputCountsToRad(524288), 6.283185307, 1e-6);  // 整圈 = 2π
}

TEST(arcmin_to_rad) {
    CHECK_NEAR(Scaling::arcminToRad(1.0), 2.908882e-4, 1e-10);
}

// 力矩指令/回读往返恒等式：与 degToTargetPosition/rpmToTargetVelocity 一致，
// nmToTargetTorque（下行）必须和 toPhysical（上行）的方向系数对称，
// 否则 current_direction=-1 时下发 +X Nm 会回读成 -X Nm。
// 用 15.5 Nm（= 500 千分比，31000mNm × 0.001 精确整除）避免 int16 千分比量化
// 噪声（分辨率 31000mNm×0.001/1000 = 0.031 Nm/count）掩盖或误报方向 bug。
TEST(torque_command_readback_roundtrip_respects_direction) {
    ScalingConfig c = realCfg();
    c.current_direction = -1;
    Scaling s(c);

    const int16_t raw_target = s.nmToTargetTorque(15.5);
    RawIo raw{};
    raw.torque_actual = raw_target;   // 假设驱动器原样执行并回读
    JointState j{};
    s.toPhysical(raw, &j);

    CHECK_NEAR(j.torque_Nm, 15.5, 1e-6);  // 往返恒等：下发 +15.5 Nm，回读也应是 +15.5 Nm
}
