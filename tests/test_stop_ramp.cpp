#include "test_framework.hpp"
#include "ecjc/config.hpp"
#include "ecjc/scaling.hpp"

using namespace ecjc;

// 默认配置下，CSP 停止必须保持"当前位置"，不得跳回 Run 起始位置。
TEST(csp_stop_holds_current_position_by_default) {
    StopRampCfg cfg;                       // 使用默认值
    const double run_start = 0.0;          // Run 开始时在 0°
    const double now       = 90.0;         // 现在走到了 90°
    CHECK_NEAR(cspStopTarget(cfg, run_start, now), 90.0, 1e-9);
}

// 首次 Run 之前 hold 仍是初值 0，此时也不得把目标打到绝对零位。
TEST(csp_stop_before_first_run_does_not_command_absolute_zero) {
    StopRampCfg cfg;
    const double hold_uninitialised = 0.0;
    const double now = 137.5;
    CHECK_NEAR(cspStopTarget(cfg, hold_uninitialised, now), 137.5, 1e-9);
}

// 显式要求保持 Run 起始位置时，行为仍然可用（保留该选项，但不再是默认）。
TEST(csp_stop_can_still_hold_run_start_when_explicitly_asked) {
    StopRampCfg cfg;
    cfg.csp_hold_position = true;
    CHECK_NEAR(cspStopTarget(cfg, 10.0, 90.0), 10.0, 1e-9);
}

// ── 复合行为：safeStopRamp 的输出还要经过 realtime_task.cpp 里的
//    CSP 位置阶跃兜底（cspTargetJumpGuard），这两步按实际调用顺序串起来测。────

// csp_hold_position=true 时，若"Run 起始位置"与当前实测偏差超过
// csp_target_jump_deg_max，兜底会在下一拍把它钳回当前实测位置——
// 这不是 bug（失效到安全侧），但意味着该配置项"保持 Run 起始位置"的
// 字面语义在偏差过大时不会被真正履行，而是静默降级为"保持当前位置"。
TEST(csp_stop_explicit_hold_run_start_gets_overridden_beyond_jump_threshold) {
    StopRampCfg cfg;
    cfg.csp_hold_position = true;
    const double run_start = 10.0;
    const double now       = 90.0;              // 与 run_start 相差 80°，远超阈值

    const double stop_target = cspStopTarget(cfg, run_start, now);
    CHECK_NEAR(stop_target, 10.0, 1e-9);         // safeStopRamp 这一步确实想保持 Run 起始位置

    const auto guard = cspTargetJumpGuard(stop_target, now, /*max_jump_deg=*/5.0);
    CHECK(guard.triggered);
    CHECK_NEAR(guard.safe_target_deg, 90.0, 1e-9);   // 但兜底把它钳回了当前实测位置
}

// 对照组：偏差在阈值内时，"保持 Run 起始位置" 的承诺确实被履行，不会被兜底覆盖。
TEST(csp_stop_explicit_hold_run_start_honored_within_jump_threshold) {
    StopRampCfg cfg;
    cfg.csp_hold_position = true;
    const double run_start = 10.0;
    const double now       = 12.0;               // 偏差 2°，在阈值 5° 内

    const double stop_target = cspStopTarget(cfg, run_start, now);
    const auto guard = cspTargetJumpGuard(stop_target, now, /*max_jump_deg=*/5.0);
    CHECK(!guard.triggered);
    CHECK_NEAR(guard.safe_target_deg, 10.0, 1e-9);   // 这次是真的保持了 Run 起始位置
}

// ── CSP 空闲保持锁存：cspIdleHoldTarget ─────────────────────────────────
// 2026-08-13 真机：CSP + 已使能 + 未运行时，目标每拍跟随实测位置，
// 位置环零刚度，摆臂重力以粘滑蠕动把关节持续拖走（电机侧 0~20 rpm 波动下坠）。
// 修复：保持位置只在进入"可保持"状态的第一拍锁存一次，之后持续下发。

TEST(csp_idle_hold_latches_once_and_does_not_follow_feedback) {
    CspIdleHold h;
    CHECK_NEAR(cspIdleHoldTarget(&h, true, 195.95), 195.95, 1e-9);  // 第一拍锁存
    CHECK(h.latched);
    // 关节被重力拖动了，目标必须钉住不跟随——这正是本次要修的 bug
    CHECK_NEAR(cspIdleHoldTarget(&h, true, 195.60), 195.95, 1e-9);
    CHECK_NEAR(cspIdleHoldTarget(&h, true, 196.30), 195.95, 1e-9);
}

TEST(csp_idle_hold_tracks_current_when_cannot_hold) {
    CspIdleHold h;
    // 未使能/模式未生效时不锁存，目标跟随实测（此时无力矩，跟随是安全的，
    // 且保证使能瞬间目标 = 当前位置，不会跳向一个陈旧的锁存值）
    CHECK_NEAR(cspIdleHoldTarget(&h, false, 50.0), 50.0, 1e-9);
    CHECK(!h.latched);
    CHECK_NEAR(cspIdleHoldTarget(&h, false, 51.0), 51.0, 1e-9);
}

TEST(csp_idle_hold_relatches_at_new_position_after_disable_enable) {
    CspIdleHold h;
    cspIdleHoldTarget(&h, true, 10.0);                       // 使能，锁存在 10°
    cspIdleHoldTarget(&h, false, 20.0);                      // 撤使能，清锁存
    CHECK(!h.latched);
    CHECK_NEAR(cspIdleHoldTarget(&h, true, 30.0), 30.0, 1e-9);  // 重新使能，在新位置锁存
}

// ── slewLimit：CSP 运行期位置斜坡 ────────────────────────────────────────
// 2026-08-13 真机：CSP 常值目标 202°、当前 109°，点开始运行第一拍就被
// 阶跃兜底拒绝并软停，GUI 看起来"没反应"。绝对角度目标必须在软件里
// 做成斜坡，每拍步进 = csp_position_rate_deg_per_s × dt。

TEST(slew_limit_caps_step_toward_far_target) {
    // 1 kHz、15 °/s → 每拍 0.015°
    CHECK_NEAR(slewLimit(202.0, 109.0, 0.015), 109.015, 1e-9);
    CHECK_NEAR(slewLimit(-50.0, 109.0, 0.015), 108.985, 1e-9);   // 反向同样限
}

TEST(slew_limit_passes_reachable_target_through) {
    CHECK_NEAR(slewLimit(1.0, 0.9, 0.5), 1.0, 1e-9);    // 一步内可达就直达，不过冲
    CHECK_NEAR(slewLimit(0.9, 0.9, 0.5), 0.9, 1e-9);    // 已到位保持不动
}

// ── 撤使能安全门限：isSafeToDisableAt ────────────────────────────────────
// 依据手册 §7.1：制动器只许在 <10% 最大转速（输出最大 25 rpm）下承受动态制动。

// 撤使能的安全门限：输出侧转速必须降到 2.5 rpm 以下。
TEST(safe_to_disable_requires_output_speed_below_2p5_rpm) {
    CHECK(!isSafeToDisableAt(10.5, /*stopping=*/true));   // 摆臂峰值，绝不允许
    CHECK(!isSafeToDisableAt(2.6,  /*stopping=*/true));
    CHECK( isSafeToDisableAt(2.4,  /*stopping=*/false));
    CHECK( isSafeToDisableAt(0.0,  /*stopping=*/false));
}

// 斜坡还没走完（stopping 仍为真）时，即使转速已经很低也要等斜坡置位完成
TEST(safe_to_disable_waits_for_ramp_completion) {
    CHECK(!isSafeToDisableAt(0.5, /*stopping=*/true));
}

// ── 撤使能门控的超时兜底：realtime_task.cpp 里"等太久就强制撤使能"这条
//    安全分支必须能脱离 RT 循环单独测试（上一轮评审因内联未测被打回）。
TEST(disable_wait_not_timed_out_within_limit) {
    CHECK(!disableWaitTimedOut(0,     15000));
    CHECK(!disableWaitTimedOut(15000, 15000));   // 恰好等于上限，尚未超出
}

TEST(disable_wait_times_out_beyond_limit) {
    CHECK(disableWaitTimedOut(15001, 15000));
}

// ── shouldHoldEnableForDisableGate：终审 finding I2 ─────────────────────
// 只有驱动器当前确已带着力矩（OperationEnabled）且还不安全时，门控才应该
// 扣住目标、维持 EnableOperation；否则会造成没有操作员动作的"自励磁"——
// 驱动器已经掉到 Switch On Disabled 等更低状态，外力反驱超过安全转速时，
// 门控绝不能把它拉回 EnableOperation。

TEST(disable_gate_holds_when_operation_enabled_and_unsafe) {
    CHECK(shouldHoldEnableForDisableGate(Cia402State::OperationEnabled,
                                          /*output_rpm=*/10.0, /*stopping=*/true));
}

TEST(disable_gate_passes_through_once_safe_even_if_operation_enabled) {
    CHECK(!shouldHoldEnableForDisableGate(Cia402State::OperationEnabled,
                                           /*output_rpm=*/0.5, /*stopping=*/false));
}

// 这是 I2 的核心场景：驱动器已经不在 OperationEnabled（比如已经是
// Switch On Disabled），即使外力把它反驱到很高的转速，也绝不能因为这条
// 门控又把它拉回 EnableOperation——那是没有操作员动作的"自励磁"。
TEST(disable_gate_never_holds_when_not_operation_enabled_regardless_of_speed) {
    CHECK(!shouldHoldEnableForDisableGate(Cia402State::SwitchOnDisabled,
                                           /*output_rpm=*/10.0, /*stopping=*/true));
    CHECK(!shouldHoldEnableForDisableGate(Cia402State::ReadyToSwitchOn,
                                           /*output_rpm=*/10.0, /*stopping=*/true));
    CHECK(!shouldHoldEnableForDisableGate(Cia402State::SwitchedOn,
                                           /*output_rpm=*/10.0, /*stopping=*/true));
    CHECK(!shouldHoldEnableForDisableGate(Cia402State::Fault,
                                           /*output_rpm=*/10.0, /*stopping=*/true));
}
