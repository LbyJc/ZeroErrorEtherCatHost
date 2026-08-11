// 极简测试框架。不引入 gtest，保持工程离线可构建。
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace tf {

struct Case { std::string name; std::function<void()> fn; };
std::vector<Case>& registry();
int  failures();
void fail(const std::string& file, int line, const std::string& msg);

struct Reg {
    Reg(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};

}  // namespace tf

#define TEST(name)                                                        \
    static void name();                                                   \
    static tf::Reg reg_##name(#name, name);                               \
    static void name()

#define CHECK(cond)                                                       \
    do { if (!(cond)) tf::fail(__FILE__, __LINE__, "CHECK 失败: " #cond); } while (0)

#define CHECK_EQ(a, b)                                                    \
    do { auto va_ = (a); auto vb_ = (b);                                  \
         if (!(va_ == vb_)) {                                             \
             char m_[512];                                                \
             snprintf(m_, sizeof m_, "CHECK_EQ 失败: %s != %s (左=%lld 右=%lld)", \
                      #a, #b, (long long)(va_), (long long)(vb_));        \
             tf::fail(__FILE__, __LINE__, m_);                            \
         } } while (0)

#define CHECK_NEAR(a, b, tol)                                             \
    do { double va_ = (double)(a), vb_ = (double)(b);                     \
         if (std::fabs(va_ - vb_) > (tol)) {                              \
             char m_[512];                                                \
             snprintf(m_, sizeof m_, "CHECK_NEAR 失败: %s vs %s (左=%.6f 右=%.6f 容差=%.6f)", \
                      #a, #b, va_, vb_, (double)(tol));                   \
             tf::fail(__FILE__, __LINE__, m_);                            \
         } } while (0)
