#include "ecjc/parameter.hpp"

#include <algorithm>

namespace ecjc {

void ParameterBlock::redefine(const std::vector<ParamMeta>& metas) {
    std::lock_guard<std::mutex> lk(mu_);
    metas_ = metas;
    if (metas_.size() > kMaxParams) metas_.resize(kMaxParams);
    staging_ = ParameterView{};
    staging_.n = metas_.size();
    for (size_t i = 0; i < metas_.size(); ++i) staging_.v[i] = metas_[i].default_v;
    generation_.fetch_add(1, std::memory_order_release);
}

int ParameterBlock::indexOf(const std::string& name) const {
    std::lock_guard<std::mutex> lk(mu_);
    for (size_t i = 0; i < metas_.size(); ++i)
        if (metas_[i].name == name) return static_cast<int>(i);
    return -1;
}

bool ParameterBlock::set(const std::string& name, double value, std::string* err) {
    std::lock_guard<std::mutex> lk(mu_);
    for (size_t i = 0; i < metas_.size(); ++i) {
        if (metas_[i].name != name) continue;
        const auto& m = metas_[i];
        if (value < m.min_v || value > m.max_v) {
            if (err) *err = "参数 " + name + " = " + std::to_string(value) +
                            " 超出允许范围 [" + std::to_string(m.min_v) + ", " +
                            std::to_string(m.max_v) + "]";
            return false;
        }
        staging_.v[i] = value;
        generation_.fetch_add(1, std::memory_order_release);
        return true;
    }
    if (err) *err = "当前控制器没有名为 \"" + name + "\" 的参数";
    return false;
}

}  // namespace ecjc
