// config.hpp —— 加载 config/*.yaml。
// 任务书第三十五节：转换系数统一放配置文件，不要散落在代码里。
#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "ecjc/scaling.hpp"
#include "ecjc/trajectory.hpp"
#include "ecjc/types.hpp"

namespace ecjc {

struct PdoEntryCfg {
    uint16_t index = 0;
    uint8_t  sub = 0;
    uint8_t  bits = 0;
    std::string type;
    std::string name;
};
struct PdoCfg {
    uint16_t index = 0;
    bool watchdog = false;
    std::vector<PdoEntryCfg> entries;
};
struct AsyncSdoCfg {
    uint16_t index = 0;
    uint8_t  sub = 0;
    std::string type;
    std::string name;
    int poll_divisor = 1;
};

struct EthercatCfg {
    unsigned master_index = 0;
    std::string interface = "enp3s0";
    unsigned cycle_us = 1000;
    bool dc_enabled = true;
    uint32_t dc_assign_activate = 0x0300;
    int32_t dc_sync0_shift_ns = 0;
    unsigned op_timeout_ms = 10000;
    unsigned wc_error_tolerance = 5;
};

struct RealtimeCfg {
    int sched_priority = 80;
    std::vector<int> cpu_affinity;
    int stack_prefault_kb = 8192;
    bool lock_memory = true;
    size_t log_ring_capacity = 16384;
    size_t gui_ring_capacity = 4096;
};

/// 启动时下发给驱动器的参数。
/// 在 activate **之前**用阻塞式 SDO 写（此时相位是 PreActivate，允许）。
/// 好处：驱动器里的关键阈值由配置文件说了算，不依赖某台驱动器碰巧被人手工设过。
struct StartupSdoCfg {
    uint16_t index = 0;
    uint8_t  sub = 0;
    std::string type;      // u8/u16/u32/i8/i16/i32
    int64_t  value = 0;
    std::string name;
    std::string comment;
};

/// activate 前的诊断只读 SDO（Task 13 消费）：上线前把这些读一遍，
/// 异常直接在日志/GUI 里报出来，而不是等到运行中才发现驱动器状态不对。
struct DiagnosticSdoCfg {
    uint16_t index = 0;
    uint8_t  sub   = 0;
    std::string type;    // "u8" | "u16" | "u32" | "i16" | "i32"
    std::string name;
};

struct SlaveCfg {
    uint16_t alias = 0, position = 0;
    uint32_t vendor_id = 0, product_code = 0;
    std::string name;
    uint32_t supported_modes_raw = 0;
    bool supports_homing = false;
    /// 从站支持的最小通信周期（µs）。0 = 未声明，不校验。
    unsigned min_cycle_us = 0;
    std::vector<DiagnosticSdoCfg> diagnostic_sdos;
};

struct AppCfg {
    std::string name = "EtherCAT Joint Control";
    std::string version = "0.1.0";
    bool auto_start_master = false;
    std::string socket_path = "/run/ethercat-joint-control/control.sock";
    std::string socket_path_dev = "/tmp/ecjc-control.sock";
    std::string data_dir = "data";
    std::string log_dir = "logs";
    std::string log_level = "INFO";

    // ── Task 13：运行期计算，不来自 yaml ──────────────────────────────
    // main.cpp 在 loadConfig() 之后、装配 RealtimeTask/DataLogger/IpcServer
    // 之前填充；ipc_server.cpp 的 record_start 处理器从这里读出写进
    // RecordingMeta，让数据文件能绑回代码版本与配置内容。
    std::string git_commit = "unknown";
    std::string config_sha256;
};

struct GuiCfg {
    int telemetry_publish_hz = 100;
    bool decimate_keep_extremes = true;
};

struct StopRampCfg {
    double csv_decel_rpm_per_s = 200.0;
    double cst_decel_Nm_per_s = 5.0;
    bool   csp_hold_position = false;   // CSP 停止 = 保持"当前实测位置"。
                                        // 置 true 会保持"本次 Run 开始时"的位置——但这个承诺
                                        // 只在该位置与当前实测偏差 ≤ scaling.limits.
                                        // csp_target_jump_deg_max（默认 5°）时才被履行：
                                        // 超过阈值，realtime_task.cpp 里的 CSP 位置阶跃兜底
                                        // 会在下一拍把它钳回当前实测位置并强制软停
                                        // （失效到安全侧，但不再是"Run 起始位置"）。
    uint64_t disable_timeout_cycles = 15000;   ///< 软停等待上限。1 kHz 下 15 s
};

/// CSP 停止时应下发的目标位置。
/// hold_deg  = 本次 Run 开始时记录的位置
/// current_deg = 当前实测位置
inline double cspStopTarget(const StopRampCfg& cfg, double hold_deg, double current_deg) {
    return cfg.csp_hold_position ? hold_deg : current_deg;
}

/// 撤使能安全门限。手册 §7.1：制动器只许在 <10% 最大转速下承受动态制动，
/// eRob80H120 输出端最大 25 rpm ⇒ 门限 2.5 rpm。
constexpr double kSafeDisableOutputRpm = 2.5;

inline bool isSafeToDisableAt(double output_rpm, bool stopping) {
    return !stopping && std::fabs(output_rpm) < kSafeDisableOutputRpm;
}

/// 撤使能门控的超时兜底判定：等待软停走完超过 disable_timeout_cycles 拍后，
/// 即使转速仍未压到安全门限也强制放行撤使能，避免速度传感异常等极端情况下
/// 进程永远卡在"等软停"、连退出都退不出去（失效到"允许"而不是"永久悬挂"）。
inline bool disableWaitTimedOut(uint64_t wait_cycles, uint64_t timeout_cycles) {
    return wait_cycles > timeout_cycles;
}

/// 撤使能门控是否应该"扣住"目标不放（继续维持 EnableOperation），
/// 而不是让 DisableVoltage 直通。
///
/// 终审 finding I2：只看转速/stopping 不够——驱动器已经处于 Switch On
/// Disabled（或任何非 OperationEnabled 状态）时，外力反驱关节超过安全转速
/// （手扳摆臂、抱闸滑动）不该触发这条门控去抢发 EnableOperation。制动器只在
/// **带着力矩**（OperationEnabled）时才可能被动态制动伤到，此时才值得等；
/// 否则就是没有操作员动作的"自励磁"——驱动器自己重新上电闭环。
///
/// current_state 是"上一拍"解码出的状态（调用方在本拍 cia_.update() 之前
/// 读取 cia_.state()），在 1kHz 下滞后不到 1ms，足够用。
inline bool shouldHoldEnableForDisableGate(Cia402State current_state,
                                            double output_rpm,
                                            bool stopping) {
    return current_state == Cia402State::OperationEnabled &&
           !isSafeToDisableAt(output_rpm, stopping);
}

struct ControllerCfg {
    std::string default_id = "passthrough";
    double torque_Nm_limit = 20.0;
    double torque_rate_Nm_per_s = 100.0;
    /// 目标速度的变化率上限（电机侧 rpm/s）。
    /// 不限的话，Constant 轨迹一开跑就是 0→100 rpm 的阶跃，
    /// 驱动器实际速度跟不上，会报 0x3B68 = 0xFF00 软速度误差警告。
    double velocity_rate_rpm_per_s = 200.0;
};

struct FullConfig {
    AppCfg        app;
    EthercatCfg   ethercat;
    RealtimeCfg   realtime;
    SlaveCfg      slave;
    ScalingConfig scaling;
    GuiCfg        gui;
    TrajParams    trajectory;
    StopRampCfg   stop_ramp;
    ControllerCfg controller;
    std::vector<PdoCfg> rx_pdos, tx_pdos;
    std::vector<AsyncSdoCfg> async_sdos;
    std::vector<StartupSdoCfg> startup_sdos;

    std::string config_dir;
};

/// 加载失败时 err 里是**人能看懂的原因**（任务书第四十三节），
/// 例如 "配置文件 config/ethercat.yaml 不存在" 而不是 "error -1"。
bool loadConfig(const std::string& dir, FullConfig* out, std::string* err);

}  // namespace ecjc
