// cia402.hpp —— CiA402 控制字/状态字解析与状态机。
//
// 本文件**不依赖任何 EtherCAT 头文件**，是纯逻辑。
// 这是能给它写单元测试的前提（任务书第五十二节第 12~14 条）。
#pragma once

#include <cstdint>
#include "ecjc/types.hpp"

namespace ecjc {

// ── 控制字位 (0x6040) ─────────────────────────────────────────────────────
namespace cw {
constexpr uint16_t kSwitchOn        = 1u << 0;
constexpr uint16_t kEnableVoltage   = 1u << 1;
constexpr uint16_t kQuickStop       = 1u << 2;   // 低有效：0 = 触发快停
constexpr uint16_t kEnableOperation = 1u << 3;
constexpr uint16_t kModeSpecific1   = 1u << 4;
constexpr uint16_t kModeSpecific2   = 1u << 5;
constexpr uint16_t kModeSpecific3   = 1u << 6;
constexpr uint16_t kFaultReset      = 1u << 7;   // 上升沿有效
constexpr uint16_t kHalt            = 1u << 8;

// 标准命令
constexpr uint16_t kCmdShutdown        = 0x0006;  // → Ready to Switch On
constexpr uint16_t kCmdSwitchOn        = 0x0007;  // → Switched On
constexpr uint16_t kCmdEnableOperation = 0x000F;  // → Operation Enabled
constexpr uint16_t kCmdDisableVoltage  = 0x0000;
constexpr uint16_t kCmdQuickStop       = 0x0002;
constexpr uint16_t kCmdDisableOperation= 0x0007;
}  // namespace cw

struct ControlwordBits {
    bool switch_on, enable_voltage, quick_stop, enable_operation;
    bool ms1, ms2, ms3, fault_reset, halt;
};
ControlwordBits decodeControlword(uint16_t cw);

// ── 状态字位 (0x6041) ─────────────────────────────────────────────────────
struct StatuswordBits {
    bool ready_to_switch_on;   // bit0
    bool switched_on;          // bit1
    bool operation_enabled;    // bit2
    bool fault;                // bit3
    bool voltage_enabled;      // bit4
    bool quick_stop;           // bit5 低有效
    bool switch_on_disabled;   // bit6
    bool warning;              // bit7
    bool remote;               // bit9
    bool target_reached;       // bit10
    bool internal_limit;       // bit11
    bool ms1;                  // bit12
    bool ms2;                  // bit13
};
StatuswordBits decodeStatusword(uint16_t sw);

/// 按 CiA402 掩码规则解码状态。注意 Not Ready / Switch On Disabled
/// 用的是通配掩码（bit4/5/6 不参与比较），不能简单用 0x006F 一刀切。
Cia402State decodeState(uint16_t sw);

// ── 状态机 ────────────────────────────────────────────────────────────────
enum class Cia402Target : uint8_t {
    Idle,            // 不主动推进，维持当前
    Shutdown,        // → Ready to Switch On
    SwitchOn,        // → Switched On
    EnableOperation, // → Operation Enabled（逐级 06→07→0F）
    DisableOperation,// → Switched On
    DisableVoltage,  // → Switch On Disabled
    QuickStop,
};

/// 状态机本身不碰总线，只回答"这一周期该发什么控制字"。
class Cia402StateMachine {
public:
    void     setTarget(Cia402Target t) { target_ = t; }
    Cia402Target target() const { return target_; }

    /// 请求一次 Fault Reset。bit7 需要**上升沿**，所以内部分三段：
    /// 先保证 bit7=0 至少 1 周期 → 拉高 hold 周期 → 再拉低。
    /// 直接常置 1 是清不掉故障的，这是常见错误。
    void requestFaultReset(uint32_t hold_cycles = 20);
    bool faultResetInProgress() const { return fr_phase_ != FrPhase::None; }

    /// 每个控制周期调用一次，返回本周期应写入 0x6040 的值。
    uint16_t update(uint16_t statusword);

    Cia402State state() const { return state_; }
    /// 目标是否已达成（GUI 用它判断"使能完成"）
    bool targetReached() const;

private:
    enum class FrPhase : uint8_t { None, Low, High };

    Cia402Target target_ = Cia402Target::Idle;
    Cia402State  state_  = Cia402State::Unknown;
    uint16_t     last_cw_ = 0;

    FrPhase  fr_phase_ = FrPhase::None;
    uint32_t fr_counter_ = 0;
    uint32_t fr_hold_ = 20;
};

}  // namespace ecjc
