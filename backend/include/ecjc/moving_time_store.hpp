// moving_time_store.hpp —— 关节累计转动时长的落盘（2026-08-18）。
//
// 450h 寿命实验要求累计值跨后端重启续算。文件内容是一个十进制整数（纳秒）。
// 写盘走 tmp+rename 原子替换：断电最多丢最后一个写盘周期（main.cpp 每 30s
// 写一次），绝不会留下半截文件把整个累计弄丢。
// 只在非实时线程使用（main 的看护循环与 IPC 线程）——RT 线程不做文件 IO。
#pragma once

#include <cstdint>
#include <string>

namespace ecjc {

class MovingTimeStore {
public:
    explicit MovingTimeStore(std::string path) : path_(std::move(path)) {}

    /// 读回累计纳秒。文件缺失/内容非法/负数一律返回 0（首启即从零累计）。
    int64_t load() const;

    /// 原子写盘。父目录不存在时会逐级创建（部署首启 log_dir 可能还没有）。
    bool save(int64_t total_ns, std::string* err) const;

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

}  // namespace ecjc
