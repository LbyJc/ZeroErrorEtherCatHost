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
