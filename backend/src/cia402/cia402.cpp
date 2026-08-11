#include "ecjc/cia402.hpp"

namespace ecjc {

const char* toString(Cia402State s) {
    switch (s) {
        case Cia402State::NotReadyToSwitchOn: return "Not Ready to Switch On";
        case Cia402State::SwitchOnDisabled:   return "Switch On Disabled";
        case Cia402State::ReadyToSwitchOn:    return "Ready to Switch On";
        case Cia402State::SwitchedOn:         return "Switched On";
        case Cia402State::OperationEnabled:   return "Operation Enabled";
        case Cia402State::QuickStopActive:    return "Quick Stop Active";
        case Cia402State::FaultReactionActive:return "Fault Reaction Active";
        case Cia402State::Fault:              return "Fault";
        default:                              return "Unknown";
    }
}

const char* toString(OpMode m) {
    switch (m) {
        case OpMode::PP:     return "PP";
        case OpMode::PV:     return "PV";
        case OpMode::PT:     return "PT";
        case OpMode::Homing: return "Homing";
        case OpMode::CSP:    return "CSP";
        case OpMode::CSV:    return "CSV";
        case OpMode::CST:    return "CST";
        default:             return "None";
    }
}

bool parseOpMode(const char* n, OpMode* out) {
    struct { const char* s; OpMode m; } tbl[] = {
        {"PP", OpMode::PP}, {"PV", OpMode::PV}, {"PT", OpMode::PT},
        {"Homing", OpMode::Homing}, {"CSP", OpMode::CSP},
        {"CSV", OpMode::CSV}, {"CST", OpMode::CST}, {"None", OpMode::None},
    };
    for (auto& e : tbl) {
        const char* a = e.s; const char* b = n;
        while (*a && *b && *a == *b) { ++a; ++b; }
        if (!*a && !*b) { *out = e.m; return true; }
    }
    return false;
}

const char* toString(EcState s) {
    switch (s) {
        case EcState::Init:   return "INIT";
        case EcState::PreOp:  return "PREOP";
        case EcState::Boot:   return "BOOT";
        case EcState::SafeOp: return "SAFEOP";
        case EcState::Op:     return "OP";
        default:              return "UNKNOWN";
    }
}

const char* toString(AppState s) {
    switch (s) {
        case AppState::Disconnected:  return "DISCONNECTED";
        case AppState::MasterReady:   return "MASTER_READY";
        case AppState::EtherCatReady: return "ETHERCAT_READY";
        case AppState::ServoDisabled: return "SERVO_DISABLED";
        case AppState::ServoEnabled:  return "SERVO_ENABLED";
        case AppState::ReadyToRun:    return "READY_TO_RUN";
        case AppState::Running:       return "RUNNING";
        case AppState::Stopping:      return "STOPPING";
        case AppState::Fault:         return "FAULT";
    }
    return "UNKNOWN";
}

ControlwordBits decodeControlword(uint16_t c) {
    ControlwordBits b{};
    b.switch_on        = c & cw::kSwitchOn;
    b.enable_voltage   = c & cw::kEnableVoltage;
    b.quick_stop       = c & cw::kQuickStop;
    b.enable_operation = c & cw::kEnableOperation;
    b.ms1              = c & cw::kModeSpecific1;
    b.ms2              = c & cw::kModeSpecific2;
    b.ms3              = c & cw::kModeSpecific3;
    b.fault_reset      = c & cw::kFaultReset;
    b.halt             = c & cw::kHalt;
    return b;
}

StatuswordBits decodeStatusword(uint16_t s) {
    StatuswordBits b{};
    b.ready_to_switch_on = s & (1u << 0);
    b.switched_on        = s & (1u << 1);
    b.operation_enabled  = s & (1u << 2);
    b.fault              = s & (1u << 3);
    b.voltage_enabled    = s & (1u << 4);
    b.quick_stop         = s & (1u << 5);
    b.switch_on_disabled = s & (1u << 6);
    b.warning            = s & (1u << 7);
    b.remote             = s & (1u << 9);
    b.target_reached     = s & (1u << 10);
    b.internal_limit     = s & (1u << 11);
    b.ms1                = s & (1u << 12);
    b.ms2                = s & (1u << 13);
    return b;
}

Cia402State decodeState(uint16_t sw) {
    // 掩码规则见 CiA402 / IEC 61800-7-201。
    // 前两个用 0x004F（bit4/5 不比较），其余用 0x006F。顺序不能颠倒：
    // 先匹配窄掩码会误判，所以按标准表从上到下逐条比。
    if ((sw & 0x004F) == 0x0000) return Cia402State::NotReadyToSwitchOn;
    if ((sw & 0x004F) == 0x0040) return Cia402State::SwitchOnDisabled;
    if ((sw & 0x006F) == 0x0021) return Cia402State::ReadyToSwitchOn;
    if ((sw & 0x006F) == 0x0023) return Cia402State::SwitchedOn;
    if ((sw & 0x006F) == 0x0027) return Cia402State::OperationEnabled;
    if ((sw & 0x006F) == 0x0007) return Cia402State::QuickStopActive;
    if ((sw & 0x004F) == 0x000F) return Cia402State::FaultReactionActive;
    if ((sw & 0x004F) == 0x0008) return Cia402State::Fault;
    return Cia402State::Unknown;
}

void Cia402StateMachine::requestFaultReset(uint32_t hold_cycles) {
    fr_hold_ = hold_cycles ? hold_cycles : 1;
    fr_phase_ = FrPhase::Low;      // 先保证一个周期的低电平，制造干净的上升沿
    fr_counter_ = 0;
}

bool Cia402StateMachine::targetReached() const {
    switch (target_) {
        case Cia402Target::EnableOperation:  return state_ == Cia402State::OperationEnabled;
        case Cia402Target::SwitchOn:
        case Cia402Target::DisableOperation: return state_ == Cia402State::SwitchedOn;
        case Cia402Target::Shutdown:         return state_ == Cia402State::ReadyToSwitchOn;
        case Cia402Target::DisableVoltage:   return state_ == Cia402State::SwitchOnDisabled;
        case Cia402Target::QuickStop:        return state_ == Cia402State::QuickStopActive;
        case Cia402Target::Idle:             return true;
    }
    return false;
}

uint16_t Cia402StateMachine::update(uint16_t statusword) {
    state_ = decodeState(statusword);

    // Fault Reset 的 bit7 脉冲优先于一切：它是从 Fault 里爬出来的唯一出路。
    if (fr_phase_ != FrPhase::None) {
        uint16_t out;
        if (fr_phase_ == FrPhase::Low) {
            out = cw::kCmdDisableVoltage;             // bit7 = 0
            if (++fr_counter_ >= 1) { fr_phase_ = FrPhase::High; fr_counter_ = 0; }
            last_cw_ = out;
        } else {
            out = cw::kCmdDisableVoltage | cw::kFaultReset;   // bit7 = 1
            if (++fr_counter_ >= fr_hold_) { fr_phase_ = FrPhase::None; fr_counter_ = 0; }
            // ⚠ 保持值里不能留 bit7：否则脉冲结束后若目标是 Idle，
            //   控制字会一直挂着 bit7=1，下一次故障就再也产生不了上升沿，
            //   Fault Reset 变成一次性功能。
            last_cw_ = cw::kCmdDisableVoltage;
        }
        return out;
    }

    uint16_t out = last_cw_;

    switch (target_) {
        case Cia402Target::Idle:
            break;

        case Cia402Target::DisableVoltage:
            out = cw::kCmdDisableVoltage;
            break;

        case Cia402Target::QuickStop:
            // bit2 = 0 触发快停，其余位保持
            out = cw::kEnableVoltage;   // 0x0002
            break;

        case Cia402Target::Shutdown:
            out = cw::kCmdShutdown;
            break;

        case Cia402Target::SwitchOn:
        case Cia402Target::DisableOperation:
            // 只有从 Ready to Switch On 才能到 Switched On；
            // 若还在 Switch On Disabled，得先 Shutdown。
            out = (state_ == Cia402State::SwitchOnDisabled) ? cw::kCmdShutdown
                                                            : cw::kCmdSwitchOn;
            break;

        case Cia402Target::EnableOperation:
            // 严格逐级：Switch On Disabled → 0x06 → 0x07 → 0x0F。
            // 绝不无条件发 0x0F（任务书第十五节）——有的驱动器会因此拒绝使能，
            // 有的会直接报错，行为不可移植。
            switch (state_) {
                case Cia402State::SwitchOnDisabled:  out = cw::kCmdShutdown;        break;
                case Cia402State::ReadyToSwitchOn:   out = cw::kCmdSwitchOn;        break;
                case Cia402State::SwitchedOn:        out = cw::kCmdEnableOperation; break;
                case Cia402State::OperationEnabled:  out = cw::kCmdEnableOperation; break;
                case Cia402State::Fault:
                case Cia402State::FaultReactionActive:
                    // 有故障时不推进使能，等 GUI 显式 Fault Reset
                    out = cw::kCmdDisableVoltage;
                    break;
                default:                             out = cw::kCmdDisableVoltage;  break;
            }
            break;
    }

    last_cw_ = out;
    return out;
}

}  // namespace ecjc
