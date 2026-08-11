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
