// CiA402 状态字解码 + 控制字生成 + 使能序列 + Fault Reset 脉冲
#include "test_framework.hpp"
#include "ecjc/cia402.hpp"

using namespace ecjc;

TEST(状态字解码_八个状态) {
    CHECK(decodeState(0x0000) == Cia402State::NotReadyToSwitchOn);
    CHECK(decodeState(0x0250) == Cia402State::SwitchOnDisabled);
    CHECK(decodeState(0x0231) == Cia402State::ReadyToSwitchOn);
    CHECK(decodeState(0x0233) == Cia402State::SwitchedOn);
    CHECK(decodeState(0x0237) == Cia402State::OperationEnabled);
    CHECK(decodeState(0x0217) == Cia402State::QuickStopActive);
    CHECK(decodeState(0x021F) == Cia402State::FaultReactionActive);
    CHECK(decodeState(0x0208) == Cia402State::Fault);
}

TEST(状态字解码_现场实测值) {
    // 这些是 2026-08-10 从真机上抓到的状态字，必须都能正确归类
    CHECK(decodeState(0x0208) == Cia402State::Fault);            // 上电锁存故障(0x730F)
    CHECK(decodeState(0x1208) == Cia402State::Fault);            // bit12 置位不影响判定
    CHECK(decodeState(0x0250) == Cia402State::SwitchOnDisabled); // 清故障后
    CHECK(decodeState(0x1637) == Cia402State::OperationEnabled); // 运行中
    CHECK(decodeState(0x1250) == Cia402State::SwitchOnDisabled); // 撤使能后
}

TEST(状态字解码_通配掩码不能用0x6F一刀切) {
    // Switch On Disabled 的掩码是 0x004F 而不是 0x006F：bit5(quick stop) 不参与比较。
    // 0x0250 与 0x0270 只差 bit5，必须判成同一个状态。
    CHECK(decodeState(0x0250) == Cia402State::SwitchOnDisabled);
    CHECK(decodeState(0x0270) == Cia402State::SwitchOnDisabled);
    // 但 bit6 是参与比较的：清掉 bit6 就不再是 Switch On Disabled
    CHECK(decodeState(0x0210) != Cia402State::SwitchOnDisabled);
    // Fault 同理，bit5 不参与
    CHECK(decodeState(0x0208) == Cia402State::Fault);
    CHECK(decodeState(0x0228) == Cia402State::Fault);
}

TEST(状态字位解析) {
    const auto b = decodeStatusword(0x1637);
    CHECK(b.ready_to_switch_on);
    CHECK(b.switched_on);
    CHECK(b.operation_enabled);
    CHECK(!b.fault);
    CHECK(b.voltage_enabled);
    CHECK(b.quick_stop);
    CHECK(!b.switch_on_disabled);
    CHECK(b.remote);
    CHECK(b.target_reached);      // bit10
    CHECK(!b.internal_limit);     // bit11 在 0x1637 里是 0
    CHECK(b.ms1);                 // bit12 = 1（CSV 下表示速度给定已被接受）
}

TEST(控制字位解析) {
    const auto b = decodeControlword(0x000F);
    CHECK(b.switch_on && b.enable_voltage && b.quick_stop && b.enable_operation);
    CHECK(!b.fault_reset && !b.halt);
    CHECK(decodeControlword(0x0080).fault_reset);
    CHECK(decodeControlword(0x0100).halt);
}

TEST(使能序列必须逐级_06_07_0F) {
    Cia402StateMachine sm;
    sm.setTarget(Cia402Target::EnableOperation);

    // 从 Switch On Disabled 出发，第一步只能是 0x06，绝不能直接 0x0F
    CHECK_EQ(sm.update(0x0250), 0x0006);
    // 驱动器进入 Ready to Switch On 后才发 0x07
    CHECK_EQ(sm.update(0x0231), 0x0007);
    // 进入 Switched On 后才发 0x0F
    CHECK_EQ(sm.update(0x0233), 0x000F);
    // 已使能则保持 0x0F
    CHECK_EQ(sm.update(0x0237), 0x000F);
    CHECK(sm.targetReached());
}

TEST(有故障时不推进使能) {
    Cia402StateMachine sm;
    sm.setTarget(Cia402Target::EnableOperation);
    // 故障态下必须发 DisableVoltage 而不是继续往 0x0F 冲
    CHECK_EQ(sm.update(0x0208), 0x0000);
    CHECK(!sm.targetReached());
}

TEST(FaultReset是bit7上升沿而不是常置1) {
    Cia402StateMachine sm;
    sm.setTarget(Cia402Target::Idle);
    sm.requestFaultReset(3);
    CHECK(sm.faultResetInProgress());

    // 第一拍必须是低电平，才能制造出干净的上升沿
    const uint16_t c0 = sm.update(0x0208);
    CHECK((c0 & cw::kFaultReset) == 0);

    // 随后拉高并保持 hold 个周期
    for (int i = 0; i < 3; ++i) {
        const uint16_t c = sm.update(0x0208);
        CHECK((c & cw::kFaultReset) != 0);
    }
    // 保持结束后自动落回
    CHECK(!sm.faultResetInProgress());
    const uint16_t cend = sm.update(0x0250);
    CHECK((cend & cw::kFaultReset) == 0);
}

TEST(快停与撤使能) {
    Cia402StateMachine sm;
    sm.setTarget(Cia402Target::QuickStop);
    CHECK_EQ(sm.update(0x0237) & 0x0004, 0x0000);   // bit2 = 0 触发快停

    sm.setTarget(Cia402Target::DisableVoltage);
    CHECK_EQ(sm.update(0x0237), 0x0000);
}

TEST(SwitchOn目标从SwitchOnDisabled出发要先Shutdown) {
    Cia402StateMachine sm;
    sm.setTarget(Cia402Target::SwitchOn);
    CHECK_EQ(sm.update(0x0250), 0x0006);   // 先 Shutdown
    CHECK_EQ(sm.update(0x0231), 0x0007);   // 再 Switch On
}

TEST(模式名解析) {
    OpMode m;
    CHECK(parseOpMode("CSV", &m) && m == OpMode::CSV);
    CHECK(parseOpMode("CST", &m) && m == OpMode::CST);
    CHECK(parseOpMode("CSP", &m) && m == OpMode::CSP);
    CHECK(!parseOpMode("NOPE", &m));
    CHECK_EQ(static_cast<int>(OpMode::CSV), 9);
    CHECK_EQ(static_cast<int>(OpMode::CST), 10);
}
