// realtime_task.hpp —— 实时控制任务。
//
// 线程模型见 ARCHITECTURE.md §2。本类拥有 RT 线程，
// 所有来自 IPC 线程的命令都通过原子量 / 指针交接进来，RT 线程内不加锁。
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "ecjc/cia402.hpp"
#include "ecjc/config.hpp"
#include "ecjc/controller.hpp"
#include "ecjc/ethercat_bus.hpp"
#include "ecjc/parameter.hpp"
#include "ecjc/ring_buffer.hpp"
#include "ecjc/scaling.hpp"
#include "ecjc/trajectory.hpp"
#include "ecjc/types.hpp"

namespace ecjc {

/// 给 GUI 的状态快照。RT 线程写、IPC 线程读，走 seqlock。
/// ⚠ 因此本结构体必须**可平凡复制**：不许放 std::string / vector /
///   任何带析构或堆指针的成员。撕裂读到一个半拷贝的 std::string 会直接段错误。
struct StatusSnapshot {
    BusStatus  bus;
    AppState   app_state = AppState::Disconnected;
    Cia402State cia_state = Cia402State::Unknown;
    uint16_t   controlword = 0, statusword = 0, error_code = 0;
    uint32_t   warning_code = 0;      // 0x3B68，厂商警告码（状态字 bit7 置位时看这里）
    int8_t     mode_display = 0;
    OpMode     mode_selected = OpMode::CSV;
    bool       running = false;
    bool       mode_matched = false;
    RtStats    stats;
    JointState joint;
    Setpoint   ref;
    double     run_time_s = 0;
    char       last_error[128] = {0};
};
static_assert(std::is_trivially_copyable<StatusSnapshot>::value,
              "StatusSnapshot 必须可平凡复制，否则 seqlock 会撕裂读出非法对象");

class RealtimeTask {
public:
    RealtimeTask(FullConfig cfg, IEtherCATBus* bus);
    ~RealtimeTask();

    bool start(std::string* err);      ///< 启动 RT 线程（此时还不碰总线）
    /// activate 完成后调用，RT 循环立刻接管总线收发。
    /// 分两步是为了消除 activate 循环与 RT 循环之间的空窗（SM 看门狗会因此打回 SAFEOP）。
    void setBusActive(bool v) { bus_active_.store(v, std::memory_order_release); }
    void requestStop();                ///< 请求退出（安全停机后再退）
    void join();

    // ── 以下方法由 IPC 线程调用，线程安全 ────────────────────────────
    void servoEnable();
    void servoDisable();
    void faultReset();
    void quickStop();
    bool setMode(OpMode m, std::string* err);
    bool setController(const std::string& id, std::string* err);
    bool setTrajectory(const TrajParams& p, std::string* err);
    void setTargetValue(double v);     ///< Constant 轨迹的目标值
    bool startRun(std::string* err);
    void stopRun();

    ParameterBlock& params() { return params_; }
    StatusSnapshot  snapshot();
    SpscRing<Sample>& logRing() { return log_ring_; }
    SpscRing<Sample>& guiRing() { return gui_ring_; }

    const FullConfig& config() const { return cfg_; }
    const Scaling& scaling() const { return scaling_; }

    void setRecording(bool on) { recording_.store(on, std::memory_order_relaxed); }
    void setRecordEpoch(int64_t ns) { record_epoch_ns_.store(ns, std::memory_order_relaxed); }

private:
    void threadMain();
    void applyRtScheduling();
    void cycle(int64_t now_ns);
    void buildSample(Sample* s, int64_t now_ns);
    void safeStopRamp(ControlOutput* o);
    bool checkRunPreconditions(std::string* err);
    void swapPending();

    FullConfig cfg_;
    IEtherCATBus* bus_;
    Scaling scaling_;
    Cia402StateMachine cia_;
    ParameterBlock params_;
    ParameterView param_view_;

    std::thread th_;
    std::atomic<bool> quit_{false};
    std::atomic<bool> thread_running_{false};
    std::atomic<bool> bus_active_{false};   ///< 主站已 activate，RT 可以泵总线

    // ── 命令（IPC 线程写，RT 线程读）───────────────────────────────
    std::atomic<int>  desired_target_{static_cast<int>(Cia402Target::Idle)};
    std::atomic<bool> fault_reset_req_{false};
    std::atomic<int>  desired_mode_{static_cast<int>(OpMode::CSV)};
    std::atomic<bool> run_req_{false};
    std::atomic<bool> stopping_{false};
    std::atomic<double> target_value_{0.0};
    /// 仅 Constant 轨迹允许被 GUI 的【目标值】实时覆盖
    std::atomic<bool> traj_is_constant_{true};
    std::atomic<bool> recording_{false};
    std::atomic<int64_t> record_epoch_ns_{0};

    // 轨迹/控制器热替换。
    // IPC 线程构造好新对象 → 原子放进 pending_；
    // RT 线程 exchange 取走，把旧指针原子塞进 retired_；
    // IPC 线程随后取走 retired_ 并 delete。
    // 全程只有原子指针交换，**RT 线程既不加锁也不分配/释放内存**。
    std::atomic<TrajectoryBase*> pending_traj_{nullptr};
    std::atomic<TrajectoryBase*> retired_traj_{nullptr};
    std::atomic<ControllerBase*> pending_ctrl_{nullptr};
    std::atomic<ControllerBase*> retired_ctrl_{nullptr};
    /// IPC 线程调用：回收 RT 交还的旧对象
    void reapRetired();

    TrajectoryBase* traj_ = nullptr;   // 所有权在 RT 线程手里
    ControllerBase* ctrl_ = nullptr;
    std::mutex ctrl_id_mu_;
    std::string ctrl_id_;

    // ── RT 线程私有状态 ────────────────────────────────────────────
    RawIo raw_{};
    JointState joint_{};
    Setpoint ref_{};
    ControlOutput out_{};
    ControlOutput last_out_{};
    uint64_t cycle_count_ = 0;
    int64_t  run_start_ns_ = 0;
    double   jitter_sum_ = 0;
    RtStats  stats_{};
    AppState app_state_ = AppState::Disconnected;
    bool     mode_matched_ = false;
    double   hold_position_deg_ = 0;

    SpscRing<Sample> log_ring_, gui_ring_;

    // 快照用 seqlock：RT 线程是写者，**永不阻塞**（这是不能用 mutex 的原因）。
    // 写：seq 变奇数 → 写数据 → seq 变偶数
    // 读：读 seq → 拷贝 → 再读 seq，若变了或为奇数就重试
    std::atomic<uint64_t> snap_seq_{0};
    StatusSnapshot snap_;
};

}  // namespace ecjc
