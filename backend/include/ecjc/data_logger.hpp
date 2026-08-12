// data_logger.hpp —— 长时间高频数据记录（HDF5 列存）。
//
// 任务书第三十一节：禁止"全部存 RAM、结束统一写盘"，
// 必须 RT 线程 → 无锁 ring → Logger 线程 → 磁盘。本类实现的是链路的后半段。
//
// 采用列存（一个字段一个 dataset）而不是行存：
//   · 分析时通常只取两三列画图，列存可以只读需要的部分
//   · 同列数据相邻，压缩率远高于行存
#pragma once

#include <atomic>
#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ecjc/config.hpp"
#include "ecjc/ring_buffer.hpp"
#include "ecjc/types.hpp"

namespace ecjc {

struct RecordingMeta {
    std::string test_name;
    std::string description;
    std::string operation_mode;
    std::string controller;
    std::string control_params_json;
    unsigned    cycle_us = 1000;
    double      sampling_hz = 1000;
    std::string slave_name;
    uint32_t    vendor_id = 0, product_code = 0;
    double      motor_counts_per_rev = 0, output_counts_per_rev = 0, gear_ratio = 0;
    bool        encoder_resolution_verified = false;
    std::string software_version;
    std::string start_time;
    std::string end_time;

    // ── Task 13：数据能绑回代码版本与驱动器标定状态 ──────────────────
    std::string git_commit;      ///< main.cpp 用 CMake 注入的宏填充；非 git 构建环境降级为 "unknown"
    std::string config_sha256;   ///< config/ 目录下全部 yaml 拼接后的 sha256，启动时算一次
    /// activate 前一次性读到的诊断 SDO（IEtherCATBus::diagnostics()）。
    /// 读失败的条目值为 INT64_MIN，写 metadata 时转成 "read_failed" 字符串属性。
    std::map<std::string, int64_t> diagnostics;
};

struct RecordingStatus {
    bool     active = false;
    std::string file;
    uint64_t samples = 0;
    uint64_t dropped = 0;
    uint64_t bytes = 0;
    double   elapsed_s = 0;
    double   disk_free_gb = 0;
    double   buffer_usage = 0;
    int64_t  start_time_ns = 0;
};

// 列定义唯一数据源。加字段只改这一处，列名/类型/取值表达式绑在一起，
// 位置耦合从根上消除。参数：(列名, HDF5 类型, C++ 缓冲类型标签, 取值表达式)
//
// 前 23 列的名称/顺序/HDF5 dtype 与重构前完全一致（既有 HDF5 文件按位置或按名读取
// 都不受影响）；seq 与 flags 是本次新增的两列，追加在尾部。
#define ECJC_SAMPLE_COLUMNS(X)                                                    \
    X(system_time_ns,               H5T_NATIVE_INT64,  I64, s[i].system_time_ns)  \
    X(elapsed_time_s,               H5T_NATIVE_DOUBLE, D,   s[i].elapsed_time_s)  \
    X(motor_position_raw,           H5T_NATIVE_INT32,  I32, s[i].motor_position_raw) \
    X(motor_position_unwrapped_deg, H5T_NATIVE_DOUBLE, D,   s[i].motor_position_unwrapped_deg) \
    X(motor_position_deg,           H5T_NATIVE_DOUBLE, D,   s[i].motor_position_deg) \
    X(motor_velocity_rpm,           H5T_NATIVE_DOUBLE, D,   s[i].motor_velocity_rpm) \
    X(output_position_raw,          H5T_NATIVE_INT32,  I32, s[i].output_position_raw) \
    X(output_position_unwrapped_deg,H5T_NATIVE_DOUBLE, D,   s[i].output_position_unwrapped_deg) \
    X(output_position_deg,          H5T_NATIVE_DOUBLE, D,   s[i].output_position_deg) \
    X(output_velocity_rpm,          H5T_NATIVE_DOUBLE, D,   s[i].output_velocity_rpm) \
    X(motor_current_A,              H5T_NATIVE_DOUBLE, D,   s[i].motor_current_A)  \
    X(actual_torque_Nm,             H5T_NATIVE_DOUBLE, D,   s[i].actual_torque_Nm) \
    X(target_position_deg,          H5T_NATIVE_DOUBLE, D,   s[i].target_position_deg) \
    X(target_velocity_rpm,          H5T_NATIVE_DOUBLE, D,   s[i].target_velocity_rpm) \
    X(target_torque_Nm,             H5T_NATIVE_DOUBLE, D,   s[i].target_torque_Nm) \
    X(position_error_deg,           H5T_NATIVE_DOUBLE, D,   s[i].position_error_deg) \
    X(velocity_error_rpm,           H5T_NATIVE_DOUBLE, D,   s[i].velocity_error_rpm) \
    X(controlword,                  H5T_NATIVE_UINT16, U16, s[i].controlword)     \
    X(statusword,                   H5T_NATIVE_UINT16, U16, s[i].statusword)      \
    X(operation_mode,               H5T_NATIVE_INT8,   I8,  s[i].operation_mode)  \
    X(cia402_state,                 H5T_NATIVE_UINT8,  U8,  s[i].cia402_state)    \
    X(ethercat_state,               H5T_NATIVE_UINT8,  U8,  s[i].ethercat_state)  \
    X(working_counter,              H5T_NATIVE_UINT32, U32, s[i].working_counter) \
    X(seq,                          H5T_NATIVE_UINT32, U32, s[i].seq)             \
    X(flags,                        H5T_NATIVE_UINT8,  U8,  s[i].flags)           \
    X(twist_counts,           H5T_NATIVE_INT32,  I32, s[i].twist_counts)           \
    X(following_error_counts, H5T_NATIVE_INT32,  I32, s[i].following_error_counts) \
    X(torque_est_mNm,         H5T_NATIVE_INT32,  I32, s[i].torque_est_mNm)         \
    X(aux_position_raw,       H5T_NATIVE_INT32,  I32, s[i].aux_position_raw)       \
    X(position_counts_raw,    H5T_NATIVE_INT32,  I32, s[i].position_counts_raw)    \
    X(motor_position_sdo,     H5T_NATIVE_INT32,  I32, s[i].motor_position_sdo)     \
    X(dc_link_voltage_mV,     H5T_NATIVE_UINT32, U32, s[i].dc_link_voltage_mV)     \
    X(warning_code,           H5T_NATIVE_UINT32, U32, s[i].warning_code)           \
    X(error_code,             H5T_NATIVE_UINT16, U16, s[i].error_code)             \
    X(temperature_drive_C,    H5T_NATIVE_UINT16, U16, s[i].temperature_drive_C)    \
    X(torque_actual_permille, H5T_NATIVE_INT16,  I16, s[i].torque_actual_permille) \
    X(torque_ratio,           H5T_NATIVE_INT16,  I16, s[i].torque_ratio)

/// 列总数（等于 kCols[] 的元素个数）
size_t sampleColumnCount();
/// 写入序列实际写出的列数——与 sampleColumnCount() 用同一份宏展开，恒等
size_t sampleWriterCount();
/// 按建列顺序返回列名
std::vector<std::string> sampleColumnNames();

class DataLogger {
public:
    DataLogger(const FullConfig& cfg, SpscRing<Sample>* ring);
    ~DataLogger();

    bool start(const RecordingMeta& meta, std::string* err);
    void stop();
    RecordingStatus status() const;

    /// 磁盘剩余低于此值时自动停止采集并告警
    void setMinFreeGb(double v) { min_free_gb_ = v; }

private:
    void threadMain();
    bool openFile(std::string* err);
    void closeFile();
    bool writeBatch(const Sample* s, size_t n, std::string* err);

    FullConfig cfg_;
    SpscRing<Sample>* ring_;
    std::thread th_;
    std::atomic<bool> quit_{false};
    std::atomic<bool> active_{false};

    RecordingMeta meta_;
    std::string cur_path_;
    double min_free_gb_ = 5.0;

    struct Impl;
    std::unique_ptr<Impl> impl_;

    mutable std::mutex st_mu_;
    RecordingStatus st_;
};

}  // namespace ecjc
