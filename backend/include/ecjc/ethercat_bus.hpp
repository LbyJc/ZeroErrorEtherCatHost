// ethercat_bus.hpp —— 总线抽象。
//
// IghBus（真实 IgH EtherCAT Master）与 MockBus（无硬件仿真）实现同一个接口，
// 上层 RealtimeTask / IPC / Logger 完全不知道自己在跟谁说话。
// 这是任务书第四十四节"Mock 和真实 EtherCAT 必须使用相同 IPC 数据结构"的落地方式：
// 不是"数据结构相同"，而是**整条链路只有最底下一层被替换**。
#pragma once

#include <functional>
#include <memory>
#include <string>

#include "ecjc/config.hpp"
#include "ecjc/types.hpp"

namespace ecjc {

/// 启动过程的分步进度回报（任务书第八节）。
/// ok=false 时 msg 必须是人能看懂的原因，不能只有错误码。
using StepReporter = std::function<void(const std::string& step, bool ok,
                                        const std::string& msg)>;

struct BusStatus {
    EcState  master_state = EcState::Unknown;
    EcState  slave_state  = EcState::Unknown;
    bool     slave_online = false;
    bool     slave_operational = false;
    bool     link_up = false;
    unsigned slave_count = 0;
    uint32_t working_counter = 0;
    int      wc_state = 0;          // 0=ZERO 1=INCOMPLETE 2=COMPLETE
    bool     dc_ok = false;
    uint64_t lost_frames = 0;
    /// 所有异步 SDO 请求（0x2240/0x2241/0x22A2...）累计的失败次数之和。
    /// 逐个对象的计数仍在 IghBus::AsyncReq 里；这里只汇总一个总数给 GUI 看。
    uint64_t async_sdo_errors = 0;
};

/// 阻塞式 SDO 的调用相位。见 ARCHITECTURE.md §0.1：
/// 在 Active 相位调用阻塞 SDO 会导致进程 D 状态死锁，只能重启机器。
enum class BusPhase : uint8_t { Idle, PreActivate, Active, PostDeactivate };

class IEtherCATBus {
public:
    virtual ~IEtherCATBus() = default;

    /// 请求主站 → 从站配置 → PDO → DC → domain。不激活。
    virtual bool configure(const FullConfig& cfg, const StepReporter& rep,
                           std::string* err) = 0;
    /// 激活主站并等待从站进入 OP
    virtual bool activate(const StepReporter& rep, std::string* err) = 0;
    virtual void deactivate() = 0;
    virtual void release() = 0;

    // ── RT 循环内调用，必须无阻塞 ──────────────────────────────────────
    virtual void receive() = 0;
    virtual void send(int64_t app_time_ns) = 0;
    virtual void readInputs(RawIo* io) = 0;
    virtual void writeOutputs(const RawIo& io) = 0;
    /// 异步 SDO 轮询（0x2240 电机侧位置只能这么读）
    virtual void pollAsyncSdo(uint64_t cycle, RawIo* io) = 0;

    virtual BusStatus status() = 0;
    virtual BusPhase  phase() const = 0;

    /// ⚠ 只允许在 PreActivate / PostDeactivate 相位调用。
    /// 其它相位实现里会**直接返回 false 而不是真的去调**——
    /// 宁可读不到诊断值，也不能把机器锁死到必须重启。
    virtual bool blockingSdoRead(uint16_t index, uint8_t sub, void* buf,
                                 size_t size, size_t* result_size,
                                 std::string* err) = 0;
    virtual bool blockingSdoWrite(uint16_t index, uint8_t sub, const void* buf,
                                  size_t size, std::string* err) = 0;

    virtual bool isMock() const = 0;
};

std::unique_ptr<IEtherCATBus> makeIghBus();
std::unique_ptr<IEtherCATBus> makeMockBus();

}  // namespace ecjc
