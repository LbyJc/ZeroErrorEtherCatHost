// controller.hpp —— 控制算法统一接口（任务书第三十八节）。
//
// 设计意图：加一个新控制器 = 新写一个 .cpp + 在工厂里注册一行，
// **GUI 一行都不用改**。因为控制器在 declareParams() 里自报参数表，
// Backend 把这张表通过 IPC 发给 GUI，调参页面据此动态生成控件。
//
// 控制器运行在 RT 线程内，因此 update() 里：
//   禁止分配内存、禁止磁盘 IO、禁止 printf、禁止加锁。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "ecjc/parameter.hpp"
#include "ecjc/types.hpp"

namespace ecjc {

struct ControlContext {
    double t   = 0;    // Run 开始至今 (s)
    double dt  = 0;    // 控制周期 (s)
    OpMode mode = OpMode::CSV;
    Setpoint   ref;    // 轨迹给定
    JointState act;    // 实测
    const ParameterView* p = nullptr;
};

class ControllerBase {
public:
    virtual ~ControllerBase() = default;

    /// 每次点【开始运行】时调用，清积分项等内部状态
    virtual void reset() = 0;

    /// 每个控制周期调用一次
    virtual void update(const ControlContext& ctx, ControlOutput* out) = 0;

    /// 自报可调参数。声明顺序即 ParameterView 的索引顺序。
    virtual void declareParams(ParameterRegistry& reg) = 0;

    virtual const char* id() const = 0;
    virtual const char* name() const = 0;
    /// 该控制器支持哪些运行模式，GUI 用它禁用不合法的组合
    virtual std::vector<OpMode> supportedModes() const = 0;
};

/// 工厂。新增控制器只需在 controller.cpp 的表里加一行。
std::unique_ptr<ControllerBase> makeController(const std::string& id);
std::vector<std::string> availableControllers();

}  // namespace ecjc
