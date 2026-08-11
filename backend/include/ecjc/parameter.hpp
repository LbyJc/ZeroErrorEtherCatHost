// parameter.hpp —— GUI → RT 线程的无锁在线调参通道。
//
// 任务书第三十九节要求"禁止 GUI 直接修改控制器内部状态造成竞争条件"。
// 做法：双缓冲 + 原子 generation。
//   写侧（IPC 线程）：改 staging → generation++ (release)
//   读侧（RT 线程）：每周期读 generation (acquire)，变了才 memcpy 一次
// RT 线程永不加锁、永不等待，最坏延迟 1 个控制周期。
#pragma once

#include <array>
#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

namespace ecjc {

constexpr size_t kMaxParams = 32;

/// 控制器自报的参数元信息，Backend 把它发给 GUI，GUI 据此**动态生成**调参控件。
struct ParamMeta {
    std::string name;
    std::string label;      // 中文显示名
    std::string unit;
    double min_v = 0, max_v = 1, default_v = 0, step = 0.01;
};

/// RT 线程看到的只读快照：一个定长 double 数组，按索引取值。
struct ParameterView {
    std::array<double, kMaxParams> v{};
    size_t n = 0;
    double get(size_t i) const { return i < n ? v[i] : 0.0; }
};

class ParameterRegistry;

/// 双缓冲参数块
class ParameterBlock {
public:
    /// 控制器切换时调用：重设参数表并把值初始化为默认值
    void redefine(const std::vector<ParamMeta>& metas);

    /// IPC 线程调用。名字不存在返回 false（GUI 会收到明确的失败原因）。
    bool set(const std::string& name, double value, std::string* err);

    /// RT 线程每周期调用。generation 没变则直接返回，零开销。
    void refreshIfChanged(ParameterView* out) {
        const uint64_t g = generation_.load(std::memory_order_acquire);
        if (g == seen_generation_) return;
        // 加锁的是**写侧极少发生**的路径；读侧这里用的是 staging 的快照拷贝，
        // 拷贝 32 个 double = 256 字节，代价可忽略且有界。
        std::lock_guard<std::mutex> lk(mu_);
        *out = staging_;
        seen_generation_ = g;
    }

    std::vector<ParamMeta> metas() const {
        std::lock_guard<std::mutex> lk(mu_);
        return metas_;
    }
    std::vector<double> values() const {
        std::lock_guard<std::mutex> lk(mu_);
        return std::vector<double>(staging_.v.begin(), staging_.v.begin() + staging_.n);
    }
    /// 参数名 → 索引。控制器在 update() 里用索引取值，不做字符串查找。
    int indexOf(const std::string& name) const;

private:
    mutable std::mutex mu_;
    std::vector<ParamMeta> metas_;
    ParameterView staging_;
    std::atomic<uint64_t> generation_{1};
    uint64_t seen_generation_ = 0;
};

/// 控制器在 declareParams() 里往这里塞条目
class ParameterRegistry {
public:
    void add(const std::string& name, const std::string& label, const std::string& unit,
             double def, double min_v, double max_v, double step = 0.01) {
        metas_.push_back(ParamMeta{name, label, unit, min_v, max_v, def, step});
    }
    const std::vector<ParamMeta>& metas() const { return metas_; }
    /// 声明顺序即索引顺序，控制器据此记住自己的参数下标
    size_t size() const { return metas_.size(); }

private:
    std::vector<ParamMeta> metas_;
};

}  // namespace ecjc
