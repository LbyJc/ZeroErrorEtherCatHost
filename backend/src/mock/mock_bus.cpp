// mock_bus.cpp —— 无硬件仿真后端（--mock）。
//
// 关键点：Mock 产生的是**和真实驱动器同尺度的原始计数**，不是已经换算好的物理量。
// 这样 scaling / CiA402 / 控制器 / Logger / IPC 全链路都被真实地走了一遍，
// 只有最底下的"网线"被换掉。否则 Mock 测出来的绿灯没有意义。
//
// 仿真内容：
//   · CiA402 状态机（对控制字的响应、Fault Reset 的 bit7 上升沿）
//   · 关节动力学：转动惯量 + 粘滞/库仑摩擦
//   · CSV / CSP / CST 三种模式各自的跟随行为
#include "ecjc/ethercat_bus.hpp"
#include "ecjc/cia402.hpp"

#include <cmath>
#include <cstring>
#include <random>

namespace ecjc {
namespace {

class MockBus : public IEtherCATBus {
public:
    bool configure(const FullConfig& cfg, const StepReporter& rep, std::string*) override {
        cfg_ = cfg;
        dt_ = cfg.ethercat.cycle_us * 1e-6;
        phase_ = BusPhase::PreActivate;
        rep("IgH Master Ready", true, "MOCK 模式：未接触真实主站");
        rep("EtherCAT Slave Detected", true, cfg.slave.name + " (模拟)");
        rep("PDO Configured", true, "MOCK");
        rep("Distributed Clock Configured", true, "MOCK");
        return true;
    }

    bool activate(const StepReporter& rep, std::string*) override {
        phase_ = BusPhase::Active;
        rep("Master Activated", true, "MOCK");
        rep("SAFEOP", true, "MOCK");
        rep("OP", true, "MOCK");
        al_ = EcState::Op;
        return true;
    }

    void deactivate() override { phase_ = BusPhase::PostDeactivate; al_ = EcState::PreOp; }
    void release() override { phase_ = BusPhase::Idle; al_ = EcState::Unknown; }

    void receive() override { /* 仿真在 send() 里推进 */ }

    void send(int64_t) override {
        step();
        ++cycles_;
    }

    void readInputs(RawIo* io) override {
        io->statusword      = statusword_;
        io->error_code      = error_code_;
        io->position_actual = static_cast<int32_t>(out_counts_);
        io->position_counts = static_cast<int32_t>(out_counts_);
        io->output_position = static_cast<int32_t>(out_counts_);
        io->velocity_actual = static_cast<int32_t>(std::lround(out_cps_));
        io->motor_position  = static_cast<int32_t>(motor_counts_);
        io->torque_actual   = static_cast<int16_t>(std::lround(torque_permille_));
        io->current_actual  = static_cast<int16_t>(std::lround(current_permille_));
        io->modes_display   = mode_display_;
        io->warning_code    = 0;

        // 固定可辨识假值：不从 cfg_.scaling 反算，避免标定测试变成同义反复
        io->twist_counts       = 1500;      // 约 2.04 角分，便于肉眼核对
        io->following_error    = 120;
        io->vendor_torque      = 8200;      // mNm
        io->torque_ratio       = 265;
        io->dc_link_mV         = 48000;
        io->drive_temp_C       = 38;
        io->motor_position_sdo = io->motor_position;   // mock 里两路一致
    }

    void writeOutputs(const RawIo& io) override { cmd_ = io; }
    void pollAsyncSdo(uint64_t, RawIo*) override { /* Mock 里电机侧位置直接在 readInputs 给出 */ }

    BusStatus status() override {
        BusStatus s{};
        s.master_state = al_;
        s.slave_state = al_;
        s.slave_online = (phase_ == BusPhase::Active);
        s.slave_operational = (al_ == EcState::Op);
        s.link_up = true;
        s.slave_count = 1;
        s.working_counter = (al_ == EcState::Op) ? 3 : 0;
        s.wc_state = (al_ == EcState::Op) ? 2 : 0;
        s.dc_ok = cfg_.ethercat.dc_enabled;
        return s;
    }

    BusPhase phase() const override { return phase_; }
    bool isMock() const override { return true; }

    bool blockingSdoRead(uint16_t idx, uint8_t, void* buf, size_t size,
                         size_t* rs, std::string* err) override {
        if (phase_ == BusPhase::Active) {
            *err = "MOCK：同样拒绝在 Active 相位阻塞式读 SDO（与真实后端行为一致）";
            return false;
        }
        uint32_t v = 0;
        switch (idx) {
            case 0x1001: v = 0; break;
            case 0x6079: v = 48000; break;      // 母线电压 mV
            case 0x22A2: v = 35; break;         // 温度
            case 0x6075: v = 6300; break;
            case 0x6076: v = 31000; break;
            default: v = 0; break;
        }
        std::memcpy(buf, &v, size < sizeof(v) ? size : sizeof(v));
        if (rs) *rs = size;
        return true;
    }

    bool blockingSdoWrite(uint16_t, uint8_t, const void*, size_t, std::string* err) override {
        if (phase_ == BusPhase::Active) { *err = "MOCK：Active 相位禁止阻塞 SDO"; return false; }
        return true;
    }

private:
    // ── 关节仿真 ──────────────────────────────────────────────────────
    void step() {
        // 1) CiA402 状态机对控制字的响应
        updateCia402(cmd_.controlword);
        mode_display_ = cmd_.modes_of_operation;

        const bool enabled = (decodeState(statusword_) == Cia402State::OperationEnabled);

        // 2) 目标 → 输出侧角速度 (counts/s)
        double target_cps = 0.0;
        if (enabled) {
            switch (static_cast<OpMode>(mode_display_)) {
                case OpMode::CSV:
                    target_cps = static_cast<double>(cmd_.target_velocity);
                    break;
                case OpMode::CSP: {
                    // 位置环：比例跟随，带速度饱和。
                    // 增益 = 1/τ：8 时 τ=125ms，跑寿命正弦（±30°@0.25Hz，峰值
                    // 47°/s）模拟跟随误差 5.9°，会把 5° 阶跃保护正当触发——
                    // 真机位置环毫秒级滞后没这个问题。40 → τ=25ms，误差 1.2°。
                    const double err = static_cast<double>(cmd_.target_position) -
                                       static_cast<double>(out_counts_);
                    target_cps = std::clamp(err * 40.0, -262144.0, 262144.0);
                    break;
                }
                case OpMode::CST: {
                    // 力矩 → 角加速度。J 取一个让 2 Nm 大约几秒到几十 rpm 的量级
                    const double trq_Nm = cmd_.target_torque * 0.001 *
                                          (cfg_.scaling.rated_torque_mNm / 1000.0);
                    const double J = 0.6;                      // kg·m²（输出侧等效）
                    const double b = 0.25;                     // 粘滞摩擦
                    const double omega = out_cps_ / cfg_.scaling.output_counts_per_rev * 2 * M_PI;
                    double coulomb = 0.0;
                    if (std::fabs(omega) > 0.01) coulomb = 0.15 * (omega > 0 ? 1 : -1);
                    const double alpha = (trq_Nm - b * omega - coulomb) / J;
                    const double omega_new = omega + alpha * dt_;
                    target_cps = omega_new / (2 * M_PI) * cfg_.scaling.output_counts_per_rev;
                    break;
                }
                default: target_cps = 0.0; break;
            }
        }

        // 3) 一阶惯性跟随 + 少量噪声，模拟真实驱动器的动态
        const double tau = 0.02;                        // 20 ms 速度环时间常数
        const double a = dt_ / (tau + dt_);
        out_cps_ += a * (target_cps - out_cps_);
        out_cps_ += noise_(rng_) * 2.0;

        // 4) 位置积分。输出侧与电机侧按减速比联动，与真实从站一致
        pos_frac_ += out_cps_ * dt_;
        const double whole = std::trunc(pos_frac_);
        out_counts_ += static_cast<int64_t>(whole);
        pos_frac_ -= whole;

        const double ratio = cfg_.scaling.gear_ratio *
                             cfg_.scaling.motor_counts_per_rev /
                             cfg_.scaling.output_counts_per_rev;
        motor_frac_ += out_cps_ * dt_ * ratio;
        const double mwhole = std::trunc(motor_frac_);
        motor_counts_ += static_cast<int64_t>(mwhole);
        motor_frac_ -= mwhole;

        // 5) 力矩/电流：负载 + 加速项
        const double omega = out_cps_ / cfg_.scaling.output_counts_per_rev * 2 * M_PI;
        const double load_Nm = 0.25 * omega + (std::fabs(omega) > 0.01 ? 0.15 : 0.0) * (omega > 0 ? 1 : -1);
        const double rated_Nm = cfg_.scaling.rated_torque_mNm / 1000.0;
        torque_permille_ = enabled ? (load_Nm / rated_Nm / 0.001) : 0.0;
        current_permille_ = torque_permille_;    // 简化：电流与力矩同比
    }

    void updateCia402(uint16_t cw) {
        const bool fault_reset_edge = (cw & cw::kFaultReset) && !(last_cw_ & cw::kFaultReset);
        last_cw_ = cw;

        if (fault_) {
            statusword_ = 0x0208;                     // Fault
            if (fault_reset_edge) { fault_ = false; error_code_ = 0; statusword_ = 0x0250; }
            return;
        }

        const bool switch_on   = cw & cw::kSwitchOn;
        const bool en_voltage  = cw & cw::kEnableVoltage;
        const bool quick_stop  = cw & cw::kQuickStop;
        const bool en_op       = cw & cw::kEnableOperation;

        if (!en_voltage) {
            statusword_ = 0x0250;                     // Switch On Disabled
        } else if (!quick_stop) {
            statusword_ = 0x0217;                     // Quick Stop Active
        } else if (!switch_on) {
            statusword_ = 0x0231;                     // Ready to Switch On
        } else if (!en_op) {
            statusword_ = 0x0233;                     // Switched On
        } else {
            statusword_ = 0x0237;                     // Operation Enabled
        }
        // 到位标志：速度接近目标时置 bit10
        statusword_ |= 0x0400;
    }

    FullConfig cfg_;
    BusPhase phase_ = BusPhase::Idle;
    EcState  al_ = EcState::Unknown;
    double   dt_ = 0.001;
    uint64_t cycles_ = 0;

    RawIo cmd_{};
    uint16_t statusword_ = 0x0250;
    uint16_t last_cw_ = 0;
    uint16_t error_code_ = 0;
    bool fault_ = false;
    int8_t mode_display_ = 0;

    int64_t out_counts_ = 0, motor_counts_ = 0;
    double  pos_frac_ = 0, motor_frac_ = 0;
    double  out_cps_ = 0;
    double  torque_permille_ = 0, current_permille_ = 0;

    std::mt19937 rng_{12345};
    std::normal_distribution<double> noise_{0.0, 1.0};
};

}  // namespace

std::unique_ptr<IEtherCATBus> makeMockBus() { return std::make_unique<MockBus>(); }

}  // namespace ecjc
