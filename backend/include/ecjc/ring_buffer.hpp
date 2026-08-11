// ring_buffer.hpp —— 单生产者单消费者(SPSC)无锁环形队列。
//
// 用于 RT 线程 → Logger 线程、RT 线程 → IPC 线程。
// 设计要点：
//   · 容量来自配置文件（不硬编码），但**只在 init() 时分配一次**，
//     RT 循环内零分配。
//   · RT 侧 push() 永不阻塞。队列满时丢弃并计数——
//     宁可丢遥测也不能拖慢实时循环。丢了多少会如实上报 GUI。
//   · head/tail 各占一条 cache line，避免伪共享导致的跨核 cache 乒乓。
//   · 容量向上取整到 2 的幂，用位与代替取模。
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>

namespace ecjc {

inline size_t roundUpPow2(size_t v) {
    if (v < 2) return 2;
    size_t p = 1;
    while (p < v) p <<= 1;
    return p;
}

template <typename T>
class SpscRing {
    static_assert(std::is_trivially_copyable<T>::value,
                  "环形队列元素必须可平凡复制（RT 线程内不允许调用构造函数）");

public:
    /// 只在启动时调用一次。RT 线程跑起来之后不得再调。
    void init(size_t capacity) {
        cap_ = roundUpPow2(capacity);
        mask_ = cap_ - 1;
        buf_ = std::make_unique<T[]>(cap_);
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        dropped_.store(0, std::memory_order_relaxed);
    }

    /// 生产者侧（RT 线程）。返回 false 表示队列满、本条已丢弃。
    bool push(const T& v) noexcept {
        const size_t h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) >= cap_) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buf_[h & mask_] = v;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    /// 消费者侧批量取，返回实际取到的条数。Logger 用它做批量写盘。
    size_t popBatch(T* out, size_t max_n) noexcept {
        const size_t h = head_.load(std::memory_order_acquire);
        size_t t = tail_.load(std::memory_order_relaxed);
        size_t n = 0;
        while (t != h && n < max_n) out[n++] = buf_[t++ & mask_];
        tail_.store(t, std::memory_order_release);
        return n;
    }

    bool pop(T* out) noexcept { return popBatch(out, 1) == 1; }

    size_t size() const noexcept {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }
    size_t capacity() const noexcept { return cap_; }
    double usage() const noexcept { return cap_ ? double(size()) / double(cap_) : 0.0; }
    uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

private:
    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
    alignas(64) std::atomic<uint64_t> dropped_{0};
    std::unique_ptr<T[]> buf_;
    size_t cap_ = 0, mask_ = 0;
};

}  // namespace ecjc
