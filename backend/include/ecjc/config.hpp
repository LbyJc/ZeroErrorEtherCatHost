// config.hpp —— 加载 config/*.yaml。
// 任务书第三十五节：转换系数统一放配置文件，不要散落在代码里。
#pragma once

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

struct SlaveCfg {
    uint16_t alias = 0, position = 0;
    uint32_t vendor_id = 0, product_code = 0;
    std::string name;
    uint32_t supported_modes_raw = 0;
    bool supports_homing = false;
    /// 从站支持的最小通信周期（µs）。0 = 未声明，不校验。
    unsigned min_cycle_us = 0;
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
};

struct GuiCfg {
    int telemetry_publish_hz = 100;
    bool decimate_keep_extremes = true;
};

struct StopRampCfg {
    double csv_decel_rpm_per_s = 200.0;
    double cst_decel_Nm_per_s = 5.0;
    bool   csp_hold_position = false;   // CSP 停止 = 保持"当前实测位置"。
                                        // 置 true 会保持"本次 Run 开始时"的位置，
                                        // 那是一次位置阶跃，CSP 下驱动器不做 profile 限制。
};

/// CSP 停止时应下发的目标位置。
/// hold_deg  = 本次 Run 开始时记录的位置
/// current_deg = 当前实测位置
inline double cspStopTarget(const StopRampCfg& cfg, double hold_deg, double current_deg) {
    return cfg.csp_hold_position ? hold_deg : current_deg;
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
