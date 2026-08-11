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
