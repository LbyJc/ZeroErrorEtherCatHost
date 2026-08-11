#include "test_framework.hpp"
#include "ecjc/config.hpp"

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
