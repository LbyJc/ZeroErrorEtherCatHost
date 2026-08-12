# P0 止血 + P1 总线采集层 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 消除会损坏硬件的既有缺陷，并把 EtherCAT 总线上这只关节能读到的物理量全部端到端落盘，使角分级的退化测量在数据链路上成立。

**Architecture:** 分两期。P0 是纯代码止血，不碰总线：修复 CSP 停止时的位置阶跃、把所有停机路径串行化到软停斜坡之后、给 IPC 发送加超时、建立版本控制。P1 扩展总线采集层：先消除 `data_logger` 的位置耦合与构建切分障碍，再扩 `RawIo`/`Sample`、补齐被丢弃的 PDO 对象、修异步 SDO 的类型分派，最后做 `0x1A00` PDO 重映射并在真机上过 J1~J6 门禁。

**Tech Stack:** C++17 / CMake / IgH EtherCAT Master 1.5.4 / HDF5 / yaml-cpp / PySide6（GUI）/ 自研极简测试框架（`tests/test_framework.hpp`）

## Global Constraints

以下为全局约束，每个任务的要求都隐含包含本节。数值逐字来自 spec v2。

- **实际传动比 = 121**（手册 §12 `n_out = n_motor/(X+1)`）。不得从型号名 eRob80H**120** 推断。
- **电机端编码器 131072 counts/rev（2^17）；输出端 524288 counts/rev（2^19）**。
- **扭转角换算**：`θ_twist[arcmin] = Δ(0x2241) × 21600 / (131072 × 121) = Δ × 1.3619417e-3`
- **恒等式**：`Δ(0x2241) = C_m(0x2240) − 30.25 × C_o(0x6064)`，其中 `30.25 = 121/4` 是精确有理数。
- **rad 换算**：`1.198422e-5 rad/count`（输出侧）；`2.908882e-4 rad/arcmin`。
- **TxPDO 字节上限 76**（手册 §16.1）。当前 36 字节，本计划完成后 52 字节。
- **绝不写 `0x1010:01 = 0x65766173`**（Store Parameters）。本计划任何步骤都不得把参数固化进驱动器 Flash。
- **改动只动 TxPDO（读方向）**，RxPDO 的 `0x1605` 一个字节都不碰。
- **真机验证第一步必须伺服不使能。**
- **制动器只许在 < 2.5 rpm（输出侧）下承受动态制动**（手册 §7.1；输出最大 25 rpm 的 10%）。任何停机路径都必须先减速到该门限以下再撤使能。
- 编译：`cmake --build build -j` ；测试：`ctest --test-dir build --output-on-failure`
- 需要 root 的命令一律用 `pkexec`（本机 sudo 无 TTY）。
- 清理进程用 `pkill -x <exact-name>` 或 `pgrep | xargs kill`，**不要**在同一条命令里再提到该进程名（会误杀执行它的 bash）。

---

# P0 · 止血

P0 全部是纯代码改动，**不需要上电、不需要总线**。做完 P0 才允许把默认运行模式切到 CSP。

---

### Task 1: 建立版本控制基线

**Files:**
- Create: `.gitignore`
- Create: （git 仓库本身）

**Interfaces:**
- Consumes: 无
- Produces: 后续每个任务都以 `git commit` 结束；Task 13 会把 `git rev-parse HEAD` 写进 HDF5 metadata

- [ ] **Step 1: 确认当前不是 git 仓库**

```bash
cd /home/tyy/Desktop/ethercat_joint_control
git status 2>&1 | head -2
```

Expected: `fatal: 不是 git 仓库`

- [ ] **Step 2: 写 .gitignore**

```
build/
data/
logs/
config.bak.*/
__pycache__/
*.pyc
*.h5
```

- [ ] **Step 3: 初始化并提交基线**

```bash
cd /home/tyy/Desktop/ethercat_joint_control
git init
git add -A
git commit -m "chore: 导入现有工程作为基线

四轮对抗审核后的起点。此提交之前的所有代码无版本记录。
审核结论见 docs/superpowers/specs/2026-08-11-bus-data-coverage-and-plan-alignment-design.md"
```

- [ ] **Step 4: 验证**

```bash
git log --oneline
git status --short
```

Expected: 一条提交记录；`git status --short` 无输出（工作区干净）

---

### Task 2: 修复 CSP 停止时的位置阶跃

**Files:**
- Modify: `backend/src/realtime/realtime_task.cpp:411-417`（`safeStopRamp` 的 CSP 分支）
- Modify: `backend/include/ecjc/config.hpp:96`（`csp_hold_position` 默认值）
- Modify: `config/trajectory.yaml:45`（配置值与注释）
- Test: `tests/test_stop_ramp.cpp`（新建）
- Modify: `tests/CMakeLists.txt:6`（加入新测试）

**Interfaces:**
- Consumes: `ecjc::StopRampCfg`（`config.hpp`）、`ecjc::OpMode`（`types.hpp`）
- Produces: `ecjc::cspStopTarget(const StopRampCfg&, double hold_deg, double current_deg) -> double` —— 提纯出来的可测函数

**背景**：`hold_position_deg_` 全工程只有一处赋值（`realtime_task.cpp:247`，Run 启动那一拍），初值 0。默认 `csp_hold_position = true`，于是 CSP 停止时目标位置跳回"本次 Run 开始时的位置"，而 `trajectory.yaml:45` 的注释写的是"保持当前位置"。`degToTargetPosition` 是绝对换算，0° → 计数 0。

- [ ] **Step 1: 把决策逻辑提纯为可测函数**

在 `backend/include/ecjc/config.hpp` 的 `StopRampCfg` 定义之后加：

```cpp
/// CSP 停止时应下发的目标位置。
/// hold_deg  = 本次 Run 开始时记录的位置
/// current_deg = 当前实测位置
inline double cspStopTarget(const StopRampCfg& cfg, double hold_deg, double current_deg) {
    return cfg.csp_hold_position ? hold_deg : current_deg;
}
```

- [ ] **Step 2: 写失败测试**

新建 `tests/test_stop_ramp.cpp`：

```cpp
#include "test_framework.hpp"
#include "ecjc/config.hpp"

using namespace ecjc;

// 默认配置下，CSP 停止必须保持"当前位置"，不得跳回 Run 起始位置。
TEST(csp_stop_holds_current_position_by_default) {
    StopRampCfg cfg;                       // 使用默认值
    const double run_start = 0.0;          // Run 开始时在 0°
    const double now       = 90.0;         // 现在走到了 90°
    CHECK_NEAR(cspStopTarget(cfg, run_start, now), 90.0, 1e-9);
}

// 首次 Run 之前 hold 仍是初值 0，此时也不得把目标打到绝对零位。
TEST(csp_stop_before_first_run_does_not_command_absolute_zero) {
    StopRampCfg cfg;
    const double hold_uninitialised = 0.0;
    const double now = 137.5;
    CHECK_NEAR(cspStopTarget(cfg, hold_uninitialised, now), 137.5, 1e-9);
}

// 显式要求保持 Run 起始位置时，行为仍然可用（保留该选项，但不再是默认）。
TEST(csp_stop_can_still_hold_run_start_when_explicitly_asked) {
    StopRampCfg cfg;
    cfg.csp_hold_position = true;
    CHECK_NEAR(cspStopTarget(cfg, 10.0, 90.0), 10.0, 1e-9);
}
```

- [ ] **Step 3: 把新测试加进构建**

`tests/CMakeLists.txt` 第 6 行改为：

```cmake
foreach(t test_cia402 test_scaling test_trajectory test_ring_buffer test_stop_ramp)
```

- [ ] **Step 4: 运行，确认前两个用例失败**

```bash
cmake --build build -j && ctest --test-dir build -R test_stop_ramp --output-on-failure
```

Expected: FAIL —— `csp_stop_holds_current_position_by_default` 与 `csp_stop_before_first_run_...` 均报 `CHECK_NEAR 失败`（左=0.000000 右=90.000000）

- [ ] **Step 5: 改默认值**

`backend/include/ecjc/config.hpp:96`：

```cpp
    bool   csp_hold_position = false;   // CSP 停止 = 保持"当前实测位置"。
                                        // 置 true 会保持"本次 Run 开始时"的位置，
                                        // 那是一次位置阶跃，CSP 下驱动器不做 profile 限制。
```

`config/trajectory.yaml:45`：

```yaml
    csp_hold_position: false    # CSP 停止 = 保持当前实测位置。
                                # 置 true 表示保持"本次 Run 开始时"的位置——那是位置阶跃，
                                # CSP 下 0x6081/0x6083 不生效，会全速冲向该位置。
```

- [ ] **Step 6: 运行，确认三个用例全过**

```bash
cmake --build build -j && ctest --test-dir build -R test_stop_ramp --output-on-failure
```

Expected: PASS，3/3

- [ ] **Step 7: 让 `safeStopRamp` 调用该函数**

`backend/src/realtime/realtime_task.cpp:411-417` 的 CSP 分支改为：

```cpp
        case OpMode::CSP:
        default:
            o->target_pos_deg = cspStopTarget(cfg_.stop_ramp,
                                              hold_position_deg_,
                                              joint_.output_pos_unwrapped_deg);
            done = true;
            break;
```

- [ ] **Step 8: 加位置阶跃兜底**

在 `realtime_task.cpp` 中，紧邻 `raw_.target_position = scaling_.degToTargetPosition(out_.target_pos_deg);`（约 `:315`）之前插入：

```cpp
    // 兜底：CSP 下驱动器不做 profile 限制，目标与实测差得太远就是一次位置阶跃。
    // 宁可拒绝下发并报错，也不要让它冲过去。
    if (want_mode == OpMode::CSP) {
        const double err_deg = out_.target_pos_deg - joint_.output_pos_unwrapped_deg;
        if (std::fabs(err_deg) > cfg_.limits.csp_target_jump_deg_max) {
            out_.target_pos_deg = joint_.output_pos_unwrapped_deg;
            stopping_.store(true, std::memory_order_relaxed);
            run_req_.store(false, std::memory_order_relaxed);
            snprintf(snap_.last_error, sizeof snap_.last_error,
                     "CSP 目标位置阶跃 %.3f° 超过上限 %.3f°，已拒绝下发并软停",
                     err_deg, cfg_.limits.csp_target_jump_deg_max);
        }
    }
```

在 `config.hpp` 的 limits 结构里加字段（默认 5°）：

```cpp
    double csp_target_jump_deg_max = 5.0;   ///< CSP 目标与实测的最大允许偏差
```

在 `backend/src/config.cpp` 的 limits 解析块里加：

```cpp
            o->limits.csp_target_jump_deg_max =
                get<double>(lim, "csp_target_jump_deg_max", 5.0);
```

在 `config/scaling.yaml` 的 `limits:` 段加：

```yaml
    csp_target_jump_deg_max: 5.0   # CSP 目标位置与实测的最大允许偏差，超过即拒绝下发并软停
```

- [ ] **Step 9: 全量测试 + mock 端到端**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
```

Expected: ctest 全绿；integration_test 34 项全过

- [ ] **Step 10: 提交**

```bash
git add -A
git commit -m "fix(rt): CSP 停止改为保持当前实测位置，并加位置阶跃兜底

原实现保持的是"本次 Run 开始时"的位置（hold_position_deg_ 只在 Run 启动那一拍赋值，
初值 0），而 trajectory.yaml 的注释写的是"保持当前位置"——注释、配置、变量名三者
都在描述一个不存在的行为。CSP 下驱动器不做 profile 限制（0x6081/0x6083 不生效），
这是一次真正的位置阶跃。

触发路径：有限时长轨迹正常跑完 / 手动停止 / 安全联锁跌出（前两者伺服仍保持使能）。
至今未暴露是因为默认模式是 CSV；实验计划要求摆臂用 CSP，一切换即从潜伏变必然。

mock_bus 的 CSP 是钳在 30 rpm 的软 P 跟随器，把这个缺陷表现成"慢慢滑回去"。"
```

---

### Task 3: 停机路径串行化到软停之后

**Files:**
- Modify: `backend/src/realtime/realtime_task.cpp`（`servoDisable`、主循环消费 `desired_target_` 处）
- Modify: `backend/include/ecjc/realtime_task.hpp`（新增成员）
- Modify: `backend/main.cpp:161-166`（停主站序列）
- Modify: `config/slave.yaml`（`startup_sdo` 增加 `0x605A`）
- Test: `tests/test_stop_ramp.cpp`（追加用例）

**Interfaces:**
- Consumes: Task 2 的 `cspStopTarget`
- Produces:
  - `ecjc::kSafeDisableOutputRpm`（constexpr double = 2.5）
  - `ecjc::isSafeToDisableAt(double output_rpm, bool stopping) -> bool` —— 自由函数，定义在 `config.hpp`
  - `StopRampCfg::disable_timeout_cycles`（uint64_t，默认 15000）
  - `StatusSnapshot::stopping`（bool，供 `main.cpp` 的等待循环读）

> `RealtimeTask::requestSafeDisable()` 由 **Task 4 Step 1** 实现，不属于本任务。

**背景**：`servoDisable()` 同时置 `stopping_ = true` 与 `desired_target_ = DisableVoltage`，而 `desired_target_` 在主循环里**无条件消费**（无 `if (!stopping_)` 门）⇒ 控制字下一拍就撤使能，**斜坡是装饰品**。`main.cpp` 的停主站序列只等 300 ms，而按 `csv_decel_rpm_per_s = 200` 从摆臂峰值（1270 电机 rpm）减到零需 6.4 秒。手册 §7.1：>2.5 rpm（输出侧）动态制动造成**永久性损坏**。

- [ ] **Step 1: 写失败测试**

在 `tests/test_stop_ramp.cpp` 追加：

```cpp
#include "ecjc/scaling.hpp"

// 撤使能的安全门限：输出侧转速必须降到 2.5 rpm 以下。
// 依据手册 §7.1：制动器只许在 <10% 最大转速（输出最大 25 rpm）下承受动态制动。
TEST(safe_to_disable_requires_output_speed_below_2p5_rpm) {
    CHECK(!isSafeToDisableAt(10.5, /*stopping=*/true));   // 摆臂峰值，绝不允许
    CHECK(!isSafeToDisableAt(2.6,  /*stopping=*/true));
    CHECK( isSafeToDisableAt(2.4,  /*stopping=*/false));
    CHECK( isSafeToDisableAt(0.0,  /*stopping=*/false));
}

// 斜坡还没走完（stopping 仍为真）时，即使转速已经很低也要等斜坡置位完成
TEST(safe_to_disable_waits_for_ramp_completion) {
    CHECK(!isSafeToDisableAt(0.5, /*stopping=*/true));
}
```

在 `backend/include/ecjc/config.hpp` 加自由函数（放在 `StopRampCfg` 之后）：

```cpp
/// 撤使能安全门限。手册 §7.1：制动器只许在 <10% 最大转速下承受动态制动，
/// eRob80H120 输出端最大 25 rpm ⇒ 门限 2.5 rpm。
constexpr double kSafeDisableOutputRpm = 2.5;

inline bool isSafeToDisableAt(double output_rpm, bool stopping) {
    return !stopping && std::fabs(output_rpm) < kSafeDisableOutputRpm;
}
```

（`config.hpp` 顶部需 `#include <cmath>`）

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build build -j 2>&1 | tail -5
```

Expected: 编译失败 —— `isSafeToDisableAt` 未定义（若先写测试后写函数）；写完函数后 `ctest -R test_stop_ramp` 应 PASS

- [ ] **Step 3: 让 `desired_target_` 的消费受 `stopping_` 约束**

`realtime_task.cpp` 中 `cia_.setTarget(...)` 那一行（约 `:210`）改为：

```cpp
    // 撤使能类目标必须等软停真正完成。否则控制字下一拍就切电，
    // 而斜坡还在数——手册 §7.1：>2.5 rpm 抱闸会永久损坏运动组件。
    {
        const auto want = static_cast<Cia402Target>(desired_target_.load(std::memory_order_relaxed));
        const bool is_disable = (want == Cia402Target::DisableVoltage ||
                                 want == Cia402Target::SwitchOnDisabled);
        if (is_disable && !isSafeToDisableAt(joint_.output_vel_rpm,
                                             stopping_.load(std::memory_order_relaxed))) {
            cia_.setTarget(Cia402Target::EnableOperation);   // 维持使能，让斜坡把速度压下来
            if (++disable_wait_cycles_ > cfg_.stop_ramp.disable_timeout_cycles) {
                cia_.setTarget(want);                        // 超时兜底，避免永远停不下来
                snprintf(snap_.last_error, sizeof snap_.last_error,
                         "软停超时（%llu 拍），强制撤使能，转速 %.2f rpm",
                         (unsigned long long)disable_wait_cycles_, joint_.output_vel_rpm);
            }
        } else {
            disable_wait_cycles_ = 0;
            cia_.setTarget(want);
        }
    }
```

`realtime_task.hpp` 的 RT 私有状态区加：

```cpp
    uint64_t disable_wait_cycles_ = 0;
```

`config.hpp` 的 `StopRampCfg` 加：

```cpp
    uint64_t disable_timeout_cycles = 15000;   ///< 软停等待上限。1 kHz 下 15 s
```

`config.cpp` 的 stop_ramp 解析块加：

```cpp
        o->stop_ramp.disable_timeout_cycles =
            static_cast<uint64_t>(get<double>(sr, "disable_timeout_cycles", 15000.0));
```

`config/trajectory.yaml` 的 `stop_ramp:` 段加：

```yaml
    disable_timeout_cycles: 15000   # 软停等待上限。1 kHz 下 15 s，够从 3000 电机 rpm 按 200 rpm/s 减到零
```

- [ ] **Step 4: 修 `main.cpp` 的停主站序列**

`backend/main.cpp:161-166` 改为：

```cpp
    auto disconnect = [&](std::string* e) -> bool {
        (void)e;
        rt.stopRun();
        // 等软停真正走完，而不是拍一个固定的 300ms。
        // 按 csv_decel_rpm_per_s=200，从 3000 电机 rpm 减到零需 15 s。
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (std::chrono::steady_clock::now() < deadline) {
            const auto st = rt.snapshot();
            if (isSafeToDisableAt(st.output_vel_rpm, st.stopping)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        rt.servoDisable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 手册：制动器动作约 150ms
        rt.requestStop();
        rt.join();
        rt.setBusActive(false);
        bus->deactivate();
```

（若 `StatusSnapshot` 没有 `stopping` 字段，在 `realtime_task.hpp` 的 `StatusSnapshot` 里加 `bool stopping;`，并在 seqlock 写入处赋值。该结构有 `is_trivially_copyable` 静态断言，`bool` 不违反。）

- [ ] **Step 5: 配置 `0x605A` quick stop option code**

`config/slave.yaml` 的 `startup_sdo` 段追加：

```yaml
  - index: 0x605A
    sub: 0
    type: i16
    value: 2
    name: "Quick stop option code"
    comment: >
      2 = 按 quick stop deceleration 减速后进入 Switch On Disabled。
      不设置时快停行为取决于驱动器 NVM 里碰巧存了什么值——这正是本工程
      在 slave.yaml 开头反对的那种依赖。
      手册 §7.1：制动器只许在 <2.5 rpm（输出侧）下承受动态制动，
      而摆臂峰值 10.5 rpm，快停必须是受控减速而不是切电。
```

- [ ] **Step 6: 全量测试**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
```

Expected: 全绿

- [ ] **Step 7: 提交**

```bash
git add -A
git commit -m "fix(rt): 撤使能必须等软停完成，停主站不再拍 300ms

servoDisable() 同时置 stopping_ 与 desired_target_=DisableVoltage，而后者在主循环里
无条件消费，控制字下一拍就切电——斜坡是装饰品。main.cpp 只等 300ms，而从摆臂峰值
（1270 电机 rpm）按 200 rpm/s 减到零需 6.4 秒。

手册 §7.1：制动器只许在 <10% 最大转速（输出 25 rpm 的 10% = 2.5 rpm）下承受动态制动，
'高转速、重载荷下触发故障、意外、急停事件会导致制动器闭合，并对运动组件造成永久性损坏'。
摆臂峰值 10.5 rpm 是该门限的 4.2 倍。

同时补 startup_sdo 的 0x605A——此前从未配置，快停行为取决于驱动器 NVM 残留值。"
```

---

### Task 4: GUI 危险操作加门控与确认，新增减速急停

**Files:**
- Modify: `gui/widgets/cia402_panel.py`（Servo Disable 按钮）
- Modify: `gui/widgets/system_panel.py`（停止主站按钮）
- Modify: `gui/widgets/mode_panel.py`（新增急停按钮）
- Modify: `backend/src/communication/ipc_server.cpp`（新增 `safe_stop` 命令）
- Modify: `backend/include/ecjc/realtime_task.hpp` / `.cpp`（`requestSafeDisable()`）

**Interfaces:**
- Consumes: Task 3 的 `isSafeToDisableAt`
- Produces: IPC 命令 `{"cmd":"safe_stop"}` —— 先软停到 2.5 rpm 以下再撤使能

- [ ] **Step 1: 后端加 `safe_stop` 命令**

`realtime_task.hpp` 声明、`.cpp` 实现：

```cpp
void RealtimeTask::requestSafeDisable() {
    run_req_.store(false);
    stopping_.store(true);
    // 注意：这里**不**直接下 DisableVoltage。
    // 由主循环在 isSafeToDisableAt() 成立后自行推进（见 Task 3）。
    desired_target_.store(static_cast<int>(Cia402Target::DisableVoltage));
}
```

`ipc_server.cpp` 的命令分发处加分支：

```cpp
    } else if (cmd == "safe_stop") {
        rt_->requestSafeDisable();
        log("INFO", "已请求安全停机：软停至 2.5 rpm 以下后撤使能");
```

- [ ] **Step 2: GUI 的 Servo Disable 加 running 门控与确认**

`gui/widgets/cia402_panel.py` 中该按钮的槽函数改为：

```python
    def _on_servo_disable(self):
        if self._running:
            box = QMessageBox(self)
            box.setIcon(QMessageBox.Warning)
            box.setWindowTitle("正在运行中")
            box.setText("关节正在运行。直接撤使能会在高速下抱闸。")
            box.setInformativeText(
                "手册 §7.1：制动器只许在 2.5 rpm（输出侧）以下承受动态制动，"
                "高转速下触发会对运动组件造成永久性损坏。\n\n"
                "建议改用【安全停机】：先软停到 2.5 rpm 以下再撤使能。"
            )
            safe = box.addButton("安全停机", QMessageBox.AcceptRole)
            box.addButton("取消", QMessageBox.RejectRole)
            box.exec()
            if box.clickedButton() is safe:
                self.command.emit({"cmd": "safe_stop"})
            return
        self.command.emit({"cmd": "servo_disable"})
```

并在遥测回调里维护 `self._running = bool(sample.flags & 0x01)`。

- [ ] **Step 3: 停止主站按钮同样加门控**

`gui/widgets/system_panel.py` 的停主站槽函数开头加：

```python
        if self._running:
            QMessageBox.warning(
                self, "正在运行中",
                "请先【停止运行】或【安全停机】，再停止主站。\n"
                "停主站会等待软停完成（最长 20 s），运行中直接停会拉长该等待。")
            return
```

- [ ] **Step 4: 新增【安全停机】按钮**

`gui/widgets/mode_panel.py` 在【停止运行】旁加：

```python
        self.btn_safe_stop = QPushButton("安全停机")
        self.btn_safe_stop.setToolTip(
            "软停至 2.5 rpm 以下后自动撤使能。\n"
            "手册 §7.1：高于该转速抱闸会永久损坏运动组件。")
        self.btn_safe_stop.clicked.connect(
            lambda: self.command.emit({"cmd": "safe_stop"}))
        row.addWidget(self.btn_safe_stop)
```

- [ ] **Step 5: mock 下验证**

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python gui/main.py --mock
```

手动检查：运行中点 Servo Disable 弹确认框；点"安全停机"后日志出现"已请求安全停机"；停主站按钮在运行中被拦下。

- [ ] **Step 6: 提交**

```bash
git add -A
git commit -m "feat(gui): 危险停机操作加门控与确认，新增安全停机

Servo Disable 原来的使能条件只看 online and enabled，不看 running，也无确认——
运行中随时可点，而那会在 10.5 rpm 下抱闸（手册门限 2.5 rpm）。"
```

---

### Task 5: IPC 发送加超时，慢客户端不再拖死整个服务

**Files:**
- Modify: `backend/src/communication/ipc_server.cpp:282-289`（accept 后设置 socket 选项）
- Modify: `backend/src/communication/ipc_server.cpp:329-344`（发送路径）

**Interfaces:**
- Consumes: 无
- Produces: 无（内部行为修复）

**背景**：fd 无 `O_NONBLOCK`、无 `SO_SNDTIMEO`、无慢客户端丢弃策略，且 `::send` 期间持全局 `send_mu_`。任一客户端停止读取，其内核发送缓冲填满后 `::send` 无限阻塞，连带卡死 telemetry 线程与 **accept 线程** ⇒ 重连也进不来，而 RT 线程仍在跑——外部看后端已死。

- [ ] **Step 1: accept 之后给 fd 设置发送超时**

在 `ipc_server.cpp` 中 accept 成功、创建 client 线程之前插入：

```cpp
        // 发送超时：慢客户端不得拖死 telemetry 与 accept 线程。
        // 450 h 无人值守时 GUI 冻结/休眠是必然事件。
        struct timeval tv{};
        tv.tv_sec  = 0;
        tv.tv_usec = 200000;      // 200 ms
        ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
```

- [ ] **Step 2: 发送失败按"慢客户端"处理，丢帧而不是阻塞**

`ipc_server.cpp` 的发送函数改为：

```cpp
    std::lock_guard<std::mutex> lk(send_mu_);
    if (::send(fd, &h, sizeof h, MSG_NOSIGNAL) != (ssize_t)sizeof h) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            ++slow_client_drops_;
            if (slow_client_drops_ % 100 == 1)
                log("WARNING", "客户端读取过慢，已丢弃 " +
                               std::to_string(slow_client_drops_) + " 帧");
            return;                       // 丢这一帧，不阻塞
        }
        return;                           // 真错误，由 serveClient 侧收尾
    }
    size_t off = 0;
    while (off < len) {
        const ssize_t n = ::send(fd, p + off, len - off, MSG_NOSIGNAL);
        if (n <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) { ++slow_client_drops_; return; }
            return;
        }
        off += static_cast<size_t>(n);
    }
```

在 `IpcServer` 类里加成员：

```cpp
    uint64_t slow_client_drops_ = 0;
```

- [ ] **Step 3: mock 下验证不再卡死**

```bash
# 终端 A：起 mock 后端
./build/ecjc-backend --config config --mock &
# 终端 B：连上但不读（模拟冻结的 GUI）
python3 -c "
import socket, time
s = socket.socket(socket.AF_UNIX)
s.connect('/tmp/ecjc-mock.sock')
time.sleep(60)      # 连上就睡，不读
"
# 终端 C：60 秒内应仍能连上并收到数据
/home/tyy/miniconda3/envs/zeroError/bin/python tests/hw_driver.py --status
```

Expected: 终端 C 能正常连上并拿到状态；后端日志出现"客户端读取过慢，已丢弃 N 帧"

- [ ] **Step 4: 清理**

```bash
pgrep -x ecjc-backend | xargs -r kill
```

- [ ] **Step 5: 提交**

```bash
git add -A
git commit -m "fix(ipc): 发送加 200ms 超时，慢客户端丢帧而不是拖死服务

原实现 fd 阻塞、无 SO_SNDTIMEO、且 ::send 期间持全局 send_mu_。
一个停止读取的客户端会连带卡死 telemetry 线程与 accept 线程——重连也进不来，
而 RT 线程仍在跑，外部看后端已死且无法诊断。450 h 无人值守下 GUI 冻结是必然事件。"
```

---

**P0 完成判据**：`ctest` 全绿；`integration_test.py` 34 项全过；GUI 在 mock 下三个危险入口都有门控；`git log` 有 5 条提交。

**此时才允许把默认运行模式切到 CSP**（在 P1 的真机验证之后做，见 Task 15）。

---

# P1 · 总线采集层

Task 6~13 是纯代码，不需要上电。Task 14~15 需要真机。

---

### Task 6: 构建重切分，解锁被结构性屏蔽的测试

**Files:**
- Modify: `CMakeLists.txt:38-45`（`ecjc_core` 源文件列表）
- Modify: `CMakeLists.txt:55-62`（可执行文件源文件列表）
- Modify: `tests/CMakeLists.txt:6`（硬编码列表改 glob）

**Interfaces:**
- Consumes: 无
- Produces: `ecjc_core` 现在包含 `mock_bus` / `realtime_task` / `ipc_server` / `data_logger`，后续任务的测试才链接得到它们

**背景**：`ecjc_core` 只含 6 个文件（cia402、scaling、trajectory、controller、parameter、config），而 `tests/CMakeLists.txt` 唯一链接的就是它。于是 `realtime_task.cpp`(573 行)、`ipc_server.cpp`(633 行)、`data_logger.cpp`(370 行)、`mock_bus.cpp`(237 行) **结构性零覆盖**——不是没写测试，是写了也链接不到。另外 `tests/CMakeLists.txt:6` 是硬编码 `foreach` 列表，新增测试文件丢进 `tests/` **什么都不会发生，ctest 保持全绿**——这是覆盖率静默回退的最可能途径。

- [ ] **Step 1: 记录当前测试数作为基线**

```bash
ctest --test-dir build -N | tail -3
```

Expected: `Total Tests: 5`（Task 2 加了 test_stop_ramp）

- [ ] **Step 2: 把四个文件移进 `ecjc_core`**

`CMakeLists.txt` 的 `ecjc_core` 源文件列表追加：

```cmake
    backend/src/mock/mock_bus.cpp
    backend/src/realtime/realtime_task.cpp
    backend/src/communication/ipc_server.cpp
    backend/src/logger/data_logger.cpp
```

并给 `ecjc_core` 补上它们需要的头文件路径与链接库：

```cmake
target_include_directories(ecjc_core PUBLIC ${HDF5_INCLUDE_DIRS} /usr/local/include)
target_link_libraries(ecjc_core PUBLIC ${HDF5_LIBRARIES} yaml-cpp pthread)
```

从可执行文件的源文件列表里删掉这四个（它们已在 `ecjc_core` 里），只留 `backend/main.cpp` 与 `backend/src/ethercat/igh_bus.cpp`、`backend/src/ethercat/scaling.cpp` 等尚未迁移的。

- [ ] **Step 3: `tests/CMakeLists.txt` 改 glob**

```cmake
# 轻量自研测试框架：不引入 gtest 依赖，保持工程可离线构建。
add_library(ecjc_test_main STATIC test_main.cpp)
target_include_directories(ecjc_test_main PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ecjc_test_main PUBLIC ecjc_core)

# 用 glob 而不是硬编码列表：新增 test_*.cpp 丢进本目录即自动纳入，
# 避免"加了测试文件却静默不参与构建、ctest 依然全绿"这种覆盖率回退。
file(GLOB ECJC_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp")
list(REMOVE_ITEM ECJC_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test_main.cpp")

foreach(src ${ECJC_TEST_SOURCES})
  get_filename_component(t ${src} NAME_WE)
  add_executable(${t} ${src})
  target_link_libraries(${t} PRIVATE ecjc_test_main)
  add_test(NAME ${t} COMMAND ${t})
  set_tests_properties(${t} PROPERTIES TIMEOUT 120)
endforeach()
```

- [ ] **Step 4: 重新配置并构建**

```bash
rm -rf build && cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure
```

Expected: 构建通过；测试数仍是 5，全绿

- [ ] **Step 5: 验证 glob 生效**

```bash
cat > tests/test_glob_probe.cpp <<'EOF'
#include "test_framework.hpp"
TEST(glob_probe_is_picked_up) { CHECK(1 == 1); }
EOF
cmake -S . -B build && cmake --build build -j && ctest --test-dir build -N | tail -3
```

Expected: `Total Tests: 6`

```bash
rm tests/test_glob_probe.cpp && cmake -S . -B build && cmake --build build -j
```

- [ ] **Step 6: 提交**

```bash
git add -A
git commit -m "build: 把 mock/rt/ipc/logger 移进 ecjc_core，测试列表改 glob

这四个文件共 1813 行，含 seqlock、安全联锁、软停、限幅、多客户端、HDF5 写入，
此前因构建切分而结构性零覆盖——不是没写测试，是写了也链接不到。

tests/CMakeLists.txt 原是硬编码 foreach 列表：新增 test_*.cpp 丢进 tests/ 什么都不会
发生，ctest 保持全绿。这是覆盖率静默回退的最可能途径。改 glob 并加 120s TIMEOUT。"
```

---

### Task 7: 补解析被静默丢弃的配置键

**Files:**
- Modify: `backend/src/config.cpp`（scaling 解析块、slave.yaml 解析块）
- Modify: `backend/include/ecjc/config.hpp`（`SlaveCfg` 加 `diagnostic_sdos`）
- Test: `tests/test_config.cpp`（新建）

**Interfaces:**
- Consumes: 无
- Produces: `ecjc::DiagnosticSdoCfg { uint16_t index; uint8_t sub; std::string type; std::string name; }`；`AppConfig::slave.diagnostic_sdos` 为其 vector

**背景**：`ScalingConfig` 里的 `target_velocity_is_motor_side`、`motor_position_modulus`、`output_position_modulus` 三个字段**在 `config.cpp` 的 scaling 解析块里一次都没出现**——YAML 改不动，只能改代码重编。`scaling.hpp` 文件头写着"所有系数来自 scaling.yaml，一个都不许写死在别处"，目前不成立。`slave.yaml` 的 `diagnostic_sdos` 整段同样从未被解析（`SlaveCfg` 里也没有对应字段），Task 13 要用它。且 `config.cpp` **完全没有测试**，正因如此这三个字段漏解析至今无人察觉。

- [ ] **Step 1: 写失败测试**

新建 `tests/test_config.cpp`：

```cpp
#include "test_framework.hpp"
#include "ecjc/config.hpp"
#include <cstdio>
#include <string>
#include <filesystem>

using namespace ecjc;

namespace {
// 在临时目录里造一套最小可用配置，只覆盖被测字段
std::filesystem::path makeConfigDir(const std::string& scaling_extra) {
    auto dir = std::filesystem::temp_directory_path() / "ecjc_cfg_test";
    std::filesystem::create_directories(dir);
    auto write = [&](const char* fn, const std::string& body) {
        FILE* f = fopen((dir / fn).c_str(), "w");
        fwrite(body.data(), 1, body.size(), f);
        fclose(f);
    };
    write("app.yaml", "app:\n  socket_path: \"/tmp/x.sock\"\n");
    write("ethercat.yaml", "ethercat:\n  cycle_us: 1000\n");
    write("slave.yaml",
          "slave:\n  vendor_id: 0x5a65726f\n  product_code: 0x00029252\n"
          "  min_cycle_us: 500\n"
          "diagnostic_sdos:\n"
          "  - {index: 0x6093, sub: 0x01, type: u32, name: \"position_factor_num\"}\n"
          "  - {index: 0x6075, sub: 0x00, type: u32, name: \"rated_current_mA\"}\n");
    write("pdo.yaml",
          "pdo:\n  rx:\n    - index: 0x1605\n      entries:\n"
          "        - {index: 0x6040, sub: 0x00, bits: 16, type: u16, name: controlword}\n"
          "  tx:\n    - index: 0x1A06\n      entries:\n"
          "        - {index: 0x6041, sub: 0x00, bits: 16, type: u16, name: statusword}\n");
    write("scaling.yaml", "scaling:\n" + scaling_extra);
    write("controller.yaml", "controller:\n  default: passthrough\n");
    write("trajectory.yaml", "trajectory:\n  default_type: constant\n");
    write("gui.yaml", "gui:\n  plot_hz: 50\n");
    return dir;
}
}  // namespace

TEST(config_parses_target_velocity_is_motor_side) {
    auto dir = makeConfigDir("  target_velocity_is_motor_side: false\n");
    AppConfig cfg; std::string err;
    CHECK(loadConfig(dir.string(), &cfg, &err));
    CHECK(cfg.scaling.target_velocity_is_motor_side == false);
}

TEST(config_parses_position_modulus) {
    auto dir = makeConfigDir("  motor_position_modulus: 131072\n"
                             "  output_position_modulus: 524288\n");
    AppConfig cfg; std::string err;
    CHECK(loadConfig(dir.string(), &cfg, &err));
    CHECK_EQ(cfg.scaling.motor_position_modulus, 131072);
    CHECK_EQ(cfg.scaling.output_position_modulus, 524288);
}

TEST(config_parses_diagnostic_sdos) {
    auto dir = makeConfigDir("  gear_ratio: 121.0\n");
    AppConfig cfg; std::string err;
    CHECK(loadConfig(dir.string(), &cfg, &err));
    CHECK_EQ((int)cfg.slave.diagnostic_sdos.size(), 2);
    CHECK_EQ((int)cfg.slave.diagnostic_sdos[0].index, 0x6093);
    CHECK_EQ((int)cfg.slave.diagnostic_sdos[0].sub, 0x01);
    CHECK(cfg.slave.diagnostic_sdos[1].name == "rated_current_mA");
}

// 拒绝路径：减速比为 0 必须被拦下并给出人能看懂的错误
TEST(config_rejects_zero_gear_ratio) {
    auto dir = makeConfigDir("  gear_ratio: 0\n");
    AppConfig cfg; std::string err;
    CHECK(!loadConfig(dir.string(), &cfg, &err));
    CHECK(err.find("减速比") != std::string::npos);
}
```

- [ ] **Step 2: 运行，确认失败**

```bash
cmake -S . -B build && cmake --build build -j && ctest --test-dir build -R test_config --output-on-failure
```

Expected: 编译失败（`diagnostic_sdos` 成员不存在）或前三个用例 FAIL

- [ ] **Step 3: 加 `DiagnosticSdoCfg` 与 `SlaveCfg` 字段**

`backend/include/ecjc/config.hpp`：

```cpp
struct DiagnosticSdoCfg {
    uint16_t index = 0;
    uint8_t  sub   = 0;
    std::string type;    // "u8" | "u16" | "u32" | "i16" | "i32"
    std::string name;
};
```

`SlaveCfg` 内加：

```cpp
    std::vector<DiagnosticSdoCfg> diagnostic_sdos;
```

- [ ] **Step 4: 补解析**

`config.cpp` 的 scaling 解析块，在 `sc.current_direction` 那行之后加：

```cpp
        sc.target_velocity_is_motor_side =
            get<bool>(s, "target_velocity_is_motor_side", true);
        sc.motor_position_modulus =
            static_cast<int64_t>(get<double>(s, "motor_position_modulus", 0.0));
        sc.output_position_modulus =
            static_cast<int64_t>(get<double>(s, "output_position_modulus", 0.0));
```

`config.cpp` 的 slave.yaml 解析块，在 `startup_sdo` 分支之后加：

```cpp
    if (n["diagnostic_sdos"]) {
        for (const auto& a : n["diagnostic_sdos"]) {
            DiagnosticSdoCfg c;
            c.index = static_cast<uint16_t>(asHex(a["index"], 0));
            c.sub   = static_cast<uint8_t>(asHex(a["sub"], 0));
            c.type  = get<std::string>(a, "type", "u32");
            c.name  = get<std::string>(a, "name", "");
            if (c.index == 0) { *err = "slave.yaml: diagnostic_sdos 条目缺少 index"; return false; }
            o->slave.diagnostic_sdos.push_back(c);
        }
    }
```

- [ ] **Step 5: 运行，确认通过**

```bash
cmake --build build -j && ctest --test-dir build -R test_config --output-on-failure
```

Expected: PASS，4/4

- [ ] **Step 6: 把三个字段写进 `config/scaling.yaml` 并加注释**

```yaml
  # 速度目标以哪一侧为准。true = GUI 输入框里填的是电机侧 rpm。
  target_velocity_is_motor_side: true

  # 编码器回绕模数。0 = 32 位累加多圈量（本机硬件当前状态）。
  # ⚠ 本机未装 3.6V 多圈电池，0x730F 上电必报，断电重启后多圈计数丢失、
  #   位置反馈退化为输出端单圈位置（0~524287）。若出现该情形，
  #   output_position_modulus 必须改成 524288，否则跨边界会静默跳 360°。
  motor_position_modulus: 0
  output_position_modulus: 0
```

- [ ] **Step 7: 全量测试 + 提交**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
git add -A
git commit -m "fix(config): 补解析三个静默丢弃的 scaling 键与 diagnostic_sdos

target_velocity_is_motor_side / motor_position_modulus / output_position_modulus
在 ScalingConfig 里有字段，但 config.cpp 的 scaling 解析块一次都没提它们——YAML 改不动。
scaling.hpp 文件头声称'所有系数来自 scaling.yaml，一个都不许写死在别处'，此前不成立。

slave.yaml 的 diagnostic_sdos 整段同样从未被解析，SlaveCfg 里也没有对应字段。

config.cpp 此前完全没有测试，正因如此这些漏解析至今无人察觉。本任务补 test_config.cpp。"
```

---

### Task 8: 修换算链路缺陷，补 rad 与扭转角换算

**Files:**
- Modify: `backend/src/ethercat/scaling.cpp:42`（力矩缺方向系数）
- Modify: `backend/src/ethercat/scaling.cpp:32`（误导性注释）
- Modify: `backend/include/ecjc/scaling.hpp`（新增换算函数）
- Modify: `tests/test_scaling.cpp`（追加用例）

**Interfaces:**
- Consumes: `ScalingConfig`
- Produces:
  - `Scaling::twistCountsToArcmin(int32_t) const -> double`
  - `Scaling::outputCountsToRad(int64_t) const -> double`
  - `Scaling::arcminToRad(double) -> double`（静态）
  - `Scaling::expectedTwistFromPositions(int32_t motor_counts, int32_t output_counts) const -> double`

- [ ] **Step 1: 写失败测试**

在 `tests/test_scaling.cpp` 追加：

```cpp
// 力矩必须和电流一样受 current_direction 影响，否则 P = τ·ω 算出来符号是错的
TEST(torque_respects_direction_sign) {
    ScalingConfig c;
    c.rated_torque_mNm = 31000.0;
    c.torque_scale = 0.001;
    c.current_direction = -1;
    Scaling s(c);
    RawIo raw{};
    raw.torque_actual = 500;              // 千分之 500 = 半额定
    JointState j{};
    s.toPhysical(raw, &j);
    CHECK_NEAR(j.torque_Nm, -15.5, 1e-6);  // 方向为负 ⇒ 力矩为负
}

// 0x2241 换算：1 count = 21600 / (131072 × 121) 角分（输出侧）
TEST(twist_counts_to_arcmin) {
    ScalingConfig c;                        // 默认 131072 / 524288 / 121
    Scaling s(c);
    CHECK_NEAR(s.twistCountsToArcmin(1), 1.3619417e-3, 1e-9);
    CHECK_NEAR(s.twistCountsToArcmin(267), 0.36363, 1e-4);  // 手册表22-2 的零扭矩开口量级
}

// 恒等式 Δ = C_m − 30.25 × C_o，30.25 = 121/4 是精确有理数
TEST(expected_twist_identity_uses_exact_30p25) {
    ScalingConfig c;
    Scaling s(c);
    CHECK_NEAR(s.expectedTwistFromPositions(30250, 1000), 0.0, 1e-9);
    CHECK_NEAR(s.expectedTwistFromPositions(30251, 1000), 1.0, 1e-9);
}

// rad 换算：计划附录 A.1 要求 theta_out_rad
TEST(output_counts_to_rad) {
    ScalingConfig c;
    Scaling s(c);
    CHECK_NEAR(s.outputCountsToRad(1), 1.198422e-5, 1e-11);
    CHECK_NEAR(s.outputCountsToRad(524288), 6.283185307, 1e-6);  // 整圈 = 2π
}

TEST(arcmin_to_rad) {
    CHECK_NEAR(Scaling::arcminToRad(1.0), 2.908882e-4, 1e-10);
}
```

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build build -j 2>&1 | tail -5
```

Expected: 编译失败 —— 四个新函数未声明

- [ ] **Step 3: 实现**

`backend/include/ecjc/scaling.hpp` 的 `Scaling` 类里加声明，`scaling.cpp` 实现：

```cpp
double Scaling::twistCountsToArcmin(int32_t counts) const {
    // 0x2241 以电机侧 17 位计数表达（手册 §22 表22-1）。
    // 换算到输出侧角分：counts / motor_cpr × 21600 / gear_ratio
    return static_cast<double>(counts) * 21600.0
           / (c_.motor_counts_per_rev * c_.gear_ratio);
}

double Scaling::expectedTwistFromPositions(int32_t motor_counts, int32_t output_counts) const {
    // Δ = C_m − (motor_cpr / output_cpr × gear_ratio) × C_o
    // 本机 = C_m − (131072/524288 × 121) × C_o = C_m − 30.25 × C_o（精确有理数 121/4）
    const double k = c_.motor_counts_per_rev / c_.output_counts_per_rev * c_.gear_ratio;
    return static_cast<double>(motor_counts) - k * static_cast<double>(output_counts);
}

double Scaling::outputCountsToRad(int64_t counts) const {
    return static_cast<double>(counts) / c_.output_counts_per_rev * 2.0 * M_PI;
}

double Scaling::arcminToRad(double arcmin) {
    return arcmin * M_PI / 10800.0;     // 2.908882e-4
}
```

`scaling.cpp:42` 的力矩换算加方向系数：

```cpp
    o->torque_Nm = dir_i * static_cast<double>(raw.torque_actual) *
                   c_.torque_scale * (c_.rated_torque_mNm / 1000.0);
```

- [ ] **Step 4: 修误导性注释**

`scaling.cpp:32` 改为：

```cpp
    // 0x606C 单位 = 输出侧 counts/s（手册 §12：n = 值/524288×60，"速度反馈是输出端的转速反馈"）。
    // ⚠ velocity_gain_correction 当前为 1.0，链路里**没有应用任何实测速度标定**。
    //   此处曾注释"实测增益 1.0006"，那是跟随误差与减速比推算的混合产物，不是单位标定。
```

同步修正 `ARCHITECTURE.md` 与 `gui/widgets/config_panel.py:40` 里同样的说法。

- [ ] **Step 5: 运行，确认通过**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
```

Expected: 全绿

- [ ] **Step 6: 提交**

```bash
git add -A
git commit -m "fix(scaling): 力矩补方向系数，新增扭转角/rad 换算与恒等式

torque_Nm 此前不乘 current_direction（电流乘了），方向取反时电流与力矩符号相反，
P = τ·ω 算错。该 bug 正好落在测试没覆盖的那一格——原测试只测了 velocity_direction=-1。

新增 0x2241 → 输出侧角分（1 count = 1.3619417e-3 角分）、输出 counts → rad
（计划 A.1 要求 theta_out_rad，工程里此前没有任何 2π/524288）、以及恒等式
Δ = C_m − 30.25×C_o（30.25 = 121/4 精确有理数），后者是 Task 15 的 J5 判据。

同时更正 scaling.cpp / ARCHITECTURE.md / config_panel.py 里'实测增益 1.0006'的说法——
velocity_gain_correction 实为 1.0，没有应用任何实测速度标定。"
```

---

### Task 9: 消除 data_logger 的列定义与写入序列位置耦合

**Files:**
- Modify: `backend/src/logger/data_logger.cpp:194-220`（`kCols[]`）
- Modify: `backend/src/logger/data_logger.cpp:280-310`（`COL_*` 写入序列）
- Test: `tests/test_logger_columns.cpp`（新建）

**Interfaces:**
- Consumes: `ecjc::Sample`
- Produces: `ECJC_SAMPLE_COLUMNS(X)` —— 单一数据源的 X-macro；`ecjc::sampleColumnCount() -> size_t`

**背景**：`kCols[]`（23 项）与写入序列 `COL_I64/COL_D/COL_I32...` 靠 `k++` **位置耦合**，没有任何编译期或运行期校验。往 `kCols` 加一列却忘了加对应 `COL_*`，或顺序不一致，**编译通过、运行不报错、数据静默写进错误的 dataset**。Task 10 要加 12 个字段，这是整个改动里最容易出错且最难发现的地方——**必须先做这一步再加字段**。

- [ ] **Step 1: 写失败测试**

新建 `tests/test_logger_columns.cpp`：

```cpp
#include "test_framework.hpp"
#include "ecjc/data_logger.hpp"

using namespace ecjc;

// 列定义与写入序列必须来自同一份声明，数量恒等
TEST(column_count_matches_writer_count) {
    CHECK_EQ((int)sampleColumnCount(), (int)sampleWriterCount());
}

// 列名不得重复（重复会让 H5Dcreate2 失败或后一列覆盖前一列）
TEST(column_names_are_unique) {
    auto names = sampleColumnNames();
    for (size_t i = 0; i < names.size(); ++i)
        for (size_t j = i + 1; j < names.size(); ++j)
            CHECK(names[i] != names[j]);
}

// 关键列必须在场——回归保护，防止有人重构时把它们弄丢
TEST(required_columns_present) {
    auto names = sampleColumnNames();
    auto has = [&](const char* n) {
        for (auto& s : names) if (s == n) return true;
        return false;
    };
    CHECK(has("system_time_ns"));
    CHECK(has("motor_position_raw"));
    CHECK(has("output_position_raw"));
    CHECK(has("seq"));        // 现在缺，事后无法从文件检测丢包
    CHECK(has("flags"));      // 现在缺
}
```

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build build -j 2>&1 | tail -5
```

Expected: 编译失败 —— 三个函数未声明

- [ ] **Step 3: 改成 X-macro 单一数据源**

`backend/include/ecjc/data_logger.hpp` 加：

```cpp
// 列定义唯一数据源。加字段只改这一处，列名/类型/取值表达式绑在一起，
// 位置耦合从根上消除。参数：(列名, HDF5 类型, C++ 缓冲类型标签, 取值表达式)
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
    X(flags,                        H5T_NATIVE_UINT8,  U8,  s[i].flags)

size_t sampleColumnCount();
size_t sampleWriterCount();
std::vector<std::string> sampleColumnNames();
```

`data_logger.cpp` 的建列处改为：

```cpp
    static const struct { const char* n; hid_t t; } kCols[] = {
#define X(name, h5type, tag, expr) {#name, h5type},
        ECJC_SAMPLE_COLUMNS(X)
#undef X
    };
```

写入序列改为：

```cpp
#define X(name, h5type, tag, expr) COL_##tag(expr)
    ECJC_SAMPLE_COLUMNS(X)
#undef X
```

并补 `COL_I8` 与 `COL_D` 宏（现有代码里 `COL_D` 已有，`COL_I8` 需新增）：

```cpp
#define COL_I8(expr) { for (size_t i=0;i<n;++i) i8[i] = (expr); if(!extend(impl_->cols[k++], i8.data())) goto fail; }
```

并在缓冲区声明处加 `std::vector<int8_t> i8(n);`。

三个查询函数：

```cpp
size_t sampleColumnCount() {
    size_t c = 0;
#define X(name, h5type, tag, expr) ++c;
    ECJC_SAMPLE_COLUMNS(X)
#undef X
    return c;
}
size_t sampleWriterCount() { return sampleColumnCount(); }   // 同一份宏展开，恒等
std::vector<std::string> sampleColumnNames() {
    std::vector<std::string> v;
#define X(name, h5type, tag, expr) v.push_back(#name);
    ECJC_SAMPLE_COLUMNS(X)
#undef X
    return v;
}
```

- [ ] **Step 4: 运行，确认通过**

```bash
cmake --build build -j && ctest --test-dir build -R test_logger_columns --output-on-failure
```

Expected: PASS，3/3（`seq` 与 `flags` 现已在列表里）

- [ ] **Step 5: mock 端到端确认 HDF5 能读**

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
ls -t data/*.h5 | head -1 | xargs -I{} /home/tyy/miniconda3/envs/zeroError/bin/python tools/h5_to_csv.py {} --list
```

Expected: 列出 25 列，含 `seq` 与 `flags`

- [ ] **Step 6: 提交**

```bash
git add -A
git commit -m "refactor(logger): 列定义改 X-macro 单一数据源，补 seq/flags 两列

kCols[] 与写入序列 COL_* 原本靠 k++ 位置耦合，无任何编译期或运行期校验。
加一列却忘了加对应宏、或顺序不一致，编译通过、运行不报错、数据静默写进错误的 dataset。
下一个任务要加 12 个字段，必须先消除这个耦合。

顺带补上 seq 与 flags——它们此前不在 HDF5 列表里，事后无法从文件检测丢包。"
```

---

### Task 10: 扩展 RawIo 与 Sample，同步线格式

**Files:**
- Modify: `backend/include/ecjc/types.hpp`（`RawIo`、`Sample`、`kProtocolVersion`、格式串注释）
- Modify: `backend/include/ecjc/data_logger.hpp`（`ECJC_SAMPLE_COLUMNS` 追加）
- Modify: `gui/ipc_client.py`（`SAMPLE_FORMAT`、`SAMPLE_FIELDS`）
- Modify: `backend/src/realtime/realtime_task.cpp`（填充新字段）
- Test: `tests/test_wire_format.cpp`（新建）

**Interfaces:**
- Consumes: Task 9 的 `ECJC_SAMPLE_COLUMNS`
- Produces: `Sample` 新增 12 个字段；`kProtocolVersion = 2`

- [ ] **Step 1: 写失败测试**

新建 `tests/test_wire_format.cpp`：

```cpp
#include "test_framework.hpp"
#include "ecjc/types.hpp"
#include <cstddef>

using namespace ecjc;

// 线格式契约：尺寸、无内部填充、协议版本已递增
TEST(sample_has_no_internal_padding) {
    // 8 字节量在前，然后 4 字节，然后 2 字节，最后 1 字节
    CHECK_EQ((int)offsetof(Sample, system_time_ns), 0);
    CHECK(offsetof(Sample, motor_position_raw) % 4 == 0);
    CHECK(offsetof(Sample, controlword) % 2 == 0);
}

TEST(protocol_version_bumped_for_new_layout) {
    CHECK_EQ((int)kProtocolVersion, 2);
}

// 新字段必须在场
TEST(new_bus_fields_present) {
    Sample s{};
    s.twist_counts = 1;
    s.following_error_counts = 2;
    s.torque_est_mNm = 3;
    s.aux_position_raw = 4;
    s.position_counts_raw = 5;
    s.motor_position_sdo = 6;
    s.dc_link_voltage_mV = 7;
    s.warning_code = 8;
    s.error_code = 9;
    s.temperature_drive_C = 10;
    s.torque_actual_permille = 11;
    s.torque_ratio = 12;
    CHECK_EQ(s.twist_counts + s.torque_ratio, 13);
}
```

- [ ] **Step 2: 运行，确认失败**

```bash
cmake --build build -j 2>&1 | tail -5
```

Expected: 编译失败 —— 新字段不存在

- [ ] **Step 3: 扩展 `RawIo`**

`types.hpp` 的 `RawIo` 输入区追加：

```cpp
    int32_t  vendor_torque   = 0;   // 0x3B69，mNm（此前已映射进 PDO 但从未读取）
    int16_t  torque_ratio    = 0;   // 0x3B6A（同上）
    int32_t  twist_counts    = 0;   // 0x2241，电机侧 17 位计数
    int32_t  following_error = 0;   // 0x60F4
    uint32_t dc_link_mV      = 0;   // 0x6079
    uint16_t drive_temp_C    = 0;   // 0x22A2，异步 SDO，与 PDO 不同步
    int32_t  motor_position_sdo = 0;// 0x2240 的异步 SDO 通道，J3 对拍专用。
                                    // 必须与 motor_position（PDO 通道）分开，
                                    // 否则两个写入者竞争同一字段，值会抖动
```

- [ ] **Step 4: 扩展 `Sample`**

在 `int32_t output_position_raw;` 之后、`uint32_t working_counter;` 之前插入 4 字节量：

```cpp
    int32_t  twist_counts;                    // 0x2241，电机侧计数
    int32_t  following_error_counts;          // 0x60F4
    int32_t  torque_est_mNm;                  // 0x3B69
    int32_t  aux_position_raw;                // 0x20A0
    int32_t  position_counts_raw;             // 0x6063
    int32_t  motor_position_sdo;              // 0x2240 异步 SDO 通道（对拍用）
    uint32_t dc_link_voltage_mV;              // 0x6079
    uint32_t warning_code;                    // 0x3B68
```

在 `uint16_t statusword;` 之后插入：

```cpp
    uint16_t error_code;                      // 0x603F
    uint16_t temperature_drive_C;             // 0x22A2。⚠ 驱动器温度，非绕组非壳体；
                                              //   异步 SDO，采样时刻与本样本不同步
    int16_t  torque_actual_permille;          // 0x6077 原始千分比
    int16_t  torque_ratio;                    // 0x3B6A
```

改断言与版本：

```cpp
static_assert(sizeof(Sample) == 184,
              "Sample 布局变了！同步更新 gui/ipc_client.py 的 SAMPLE_FORMAT 与 kProtocolVersion");
```

```cpp
constexpr uint16_t kProtocolVersion = 2;
```

格式串注释（`types.hpp:80`）改为：

```cpp
// Python 对应格式串: "<q14d8i2I4h..." —— 以 gui/ipc_client.py 的 SAMPLE_FORMAT 为准，
// 两边尺寸不一致时 GUI 连接握手会直接报错。
```

> **实测步骤**：先按上面写，编译时 `static_assert` 会报出真实 `sizeof`。若不是 184，把断言与 Python 格式串一并改成实测值——**不要为了凑数字调整字段顺序**，字段顺序按"8 字节 → 4 字节 → 2 字节 → 1 字节"排即可。

- [ ] **Step 5: 同步 Python 线格式**

`gui/ipc_client.py`：

```python
PROTOCOL_VERSION = 2

# 遥测样本：与 C++ Sample 一一对应
# q  : system_time_ns
# 14d: elapsed_time_s .. velocity_error_rpm
# 8i : motor_position_raw, output_position_raw, twist_counts, following_error_counts,
#      torque_est_mNm, aux_position_raw, position_counts_raw, motor_position_sdo
# 3I : dc_link_voltage_mV, warning_code, working_counter, seq   ← 见下方实测
SAMPLE_FORMAT = "<q14d8i4I4H2h4b"     # ⚠ 以 C++ static_assert 报出的实际 sizeof 为准调整
```

`SAMPLE_FIELDS` 按同样顺序补齐 12 个新名字。

- [ ] **Step 6: 填充新字段**

`realtime_task.cpp` 组装 `Sample` 的地方追加：

```cpp
    s->twist_counts            = raw_.twist_counts;
    s->following_error_counts  = raw_.following_error;
    s->torque_est_mNm          = raw_.vendor_torque;
    s->aux_position_raw        = raw_.output_position;
    s->position_counts_raw     = raw_.position_counts;
    s->motor_position_sdo      = raw_.motor_position_sdo;
    s->dc_link_voltage_mV      = raw_.dc_link_mV;
    s->warning_code            = raw_.warning_code;
    s->error_code              = raw_.error_code;
    s->temperature_drive_C     = raw_.drive_temp_C;
    s->torque_actual_permille  = raw_.torque_actual;
    s->torque_ratio            = raw_.torque_ratio;
```

- [ ] **Step 7: 把新字段加进 HDF5 列**

`data_logger.hpp` 的 `ECJC_SAMPLE_COLUMNS` 追加 12 行：

```cpp
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
```

补 `COL_I16` 宏与 `std::vector<int16_t> i16(n);` 缓冲。

- [ ] **Step 8: mock 补齐新字段**

`backend/src/mock/mock_bus.cpp` 的 `readInputs` 里给新字段填可辨识的假值（**不要用 `cfg_.scaling` 反算**，否则测试变成同义反复）：

```cpp
    io->twist_counts    = 1500;      // 固定值，约 2.04 角分，便于肉眼核对
    io->following_error = 120;
    io->vendor_torque   = 8200;      // mNm
    io->torque_ratio    = 265;
    io->dc_link_mV      = 48000;
    io->drive_temp_C    = 38;
    io->motor_position_sdo = io->motor_position;   // mock 里两路一致
```

- [ ] **Step 9: 运行全量测试**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
/home/tyy/miniconda3/envs/zeroError/bin/python gui/main.py --mock --screenshot /tmp/gui.png --screenshot-delay 8 --autorun
```

Expected: ctest 全绿；integration_test 全过；GUI 截图正常（线格式校验通过，日志出现 `Sample 184 字节，协议 v2`）

- [ ] **Step 10: 提交**

```bash
git add -A
git commit -m "feat(wire): Sample 扩展 12 个总线字段，协议 v2

新增：twist_counts(0x2241) / following_error_counts(0x60F4) / torque_est_mNm(0x3B69) /
aux_position_raw(0x20A0) / position_counts_raw(0x6063) / motor_position_sdo(0x2240异步通道) /
dc_link_voltage_mV(0x6079) / warning_code(0x3B68) / error_code(0x603F) /
temperature_drive_C(0x22A2) / torque_actual_permille(0x6077原始值) / torque_ratio(0x3B6A)

其中 0x3B69/0x3B6A 此前已映射进 PDO 却从未被 readInputs 读取，白占 6 字节过程数据；
0x20A0/0x6063 读进 RawIo 后被丢弃——而 0x6064/0x6063 之比按 CiA402 定义就是 0x6093
位置因子，是全链路最强的免费验证。

motor_position_sdo 必须与 motor_position 分开：0x2240 进 PDO 后两个写入者会竞争同一字段。

mock 的假值用固定常量而非 cfg_.scaling 反算——后者会让所有经 mock 的标定测试变成同义反复。"
```

---

### Task 11: readInputs 补齐已映射却从未读取的对象

**Files:**
- Modify: `backend/src/ethercat/igh_bus.cpp:265-284`（`readInputs`）
- Modify: `config/pdo.yaml`（新增 `0x1A18`、`0x1A19` 分配）

**Interfaces:**
- Consumes: Task 10 的 `RawIo` 新字段
- Produces: `RawIo::vendor_torque` / `torque_ratio` / `following_error` / `dc_link_mV` 被真实填充

- [ ] **Step 1: `pdo.yaml` 增加两个 TxPDO 分配**

在 `tx:` 列表末尾追加：

```yaml
    - index: 0x1A18
      entries:
        - {index: 0x6079, sub: 0x00, bits: 32, type: u32, name: dc_link_voltage}
    - index: 0x1A19
      entries:
        - {index: 0x60F4, sub: 0x00, bits: 32, type: i32, name: following_error}
```

同时**修正该文件顶部那条事实错误的注释**：

```yaml
# ⚠ 更正（2026-08-11）：此前这里写"一致时 IgH 不会去写映射寄存器，只做 0x1C12/0x1C13 分配"，
#   这是错的。IgH 的 master/fsm_pdo.c 里注释就是字面的 "// always write PDO mapping"——
#   每次 activate 都无条件重写全部已分配 PDO 的条目表，包括所有 Fixed="1" 的。
#   反过来说：现在这套配置能跑到 OP，恰恰证明这台驱动器接受条目表改写。
```

- [ ] **Step 2: `readInputs` 补四行**

`igh_bus.cpp` 的 `readInputs()` 里，在 `io->warning_code = rdu32("warning_code");` 之后追加：

```cpp
        io->vendor_torque   = rd32("vendor_torque");      // 0x3B69，此前已映射但从未读
        io->torque_ratio    = rd16s("torque_ratio");      // 0x3B6A，同上
        io->dc_link_mV      = rdu32("dc_link_voltage");   // 0x6079，新增映射
        io->following_error = rd32("following_error");    // 0x60F4，新增映射
```

- [ ] **Step 3: mock 同步（已在 Task 10 Step 8 做完）**

确认 `mock_bus.cpp` 的 `readInputs` 里这四个字段有值。

- [ ] **Step 4: 验证**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
ls -t data/*.h5 | head -1 | xargs -I{} /home/tyy/miniconda3/envs/zeroError/bin/python tools/h5_to_csv.py {} --list
```

Expected: 列表含 `torque_est_mNm` / `torque_ratio` / `dc_link_voltage_mV` / `following_error_counts`，且 mock 下取值为 Task 10 设的常量

- [ ] **Step 5: 提交**

```bash
git add -A
git commit -m "feat(bus): readInputs 补读 0x3B69/0x3B6A，新增 0x1A18/0x1A19 映射

0x3B69(计算扭矩,mNm) 与 0x3B6A(力矩比) 此前已在 pdo.yaml 的 0x1A08 里、占着 SM3 的
6 字节过程数据，但 readInputs() 的 10 个读取里根本没有它们——数据在总线上，被丢弃。

新增 0x6079(母线电压) 与 0x60F4(跟随误差)：前者是 450 h 的头号中断风险监控量
（手册 §3.3：eRob 关节内部没有再生制动电路，摆臂每半周期回灌，出厂上限 55V），
后者是计划 §4.8 的 tracking_error_rms。

同时更正 pdo.yaml 顶部关于 IgH 映射行为的注释——原说法与 fsm_pdo.c 的
'// always write PDO mapping' 直接矛盾。"
```

---

### Task 12: 异步 SDO 类型分派与独立的 SDO 通道

**Files:**
- Modify: `backend/src/ethercat/igh_bus.cpp:132-140`（request 创建）
- Modify: `backend/src/ethercat/igh_bus.cpp:297-315`（`pollAsyncSdo`）
- Modify: `config/pdo.yaml`（`async_sdo` 段）

**Interfaces:**
- Consumes: `AsyncSdoCfg::type`（`config.cpp` 早已解析，但从未被使用）
- Produces: `RawIo::motor_position_sdo` / `drive_temp_C` 被真实填充；异步 SDO 错误计数经状态透出

**背景**：三处硬编码——`igh_bus.cpp:133` 的 request size 写死 4、`:303` 的 `EC_READ_S32` 写死、`:304` 的接收端只有一个 `if (name == "motor_position")` 分支。于是 `pdo.yaml:57` 配的 `0x2241 → dual_encoder_diff` **读回后直接丢弃**，这条链路从来没被验证过。且 `AsyncSdoCfg::type` 是死代码。

- [ ] **Step 1: request 按类型定尺寸**

```cpp
        for (const auto& a : cfg.async_sdos) {
            const size_t sz = (a.type == "u8"  || a.type == "i8")  ? 1
                            : (a.type == "u16" || a.type == "i16") ? 2 : 4;
            ec_sdo_request_t* r = ecrt_slave_config_create_sdo_request(
                sc_, a.index, a.sub, sz);
            if (!r) { *err = "创建异步 SDO 请求失败: 0x" + toHex(a.index); return false; }
            ecrt_sdo_request_timeout(r, 500);
            reqs_.push_back({r, a, 0});
        }
```

- [ ] **Step 2: 按类型读，按名字表驱动分派**

`pollAsyncSdo` 的 SUCCESS 分支改为：

```cpp
                case EC_REQUEST_SUCCESS: {
                    const uint8_t* d = ecrt_sdo_request_data(a.req);
                    int64_t v = 0;
                    if      (a.cfg.type == "u8")  v = EC_READ_U8(d);
                    else if (a.cfg.type == "i8")  v = EC_READ_S8(d);
                    else if (a.cfg.type == "u16") v = EC_READ_U16(d);
                    else if (a.cfg.type == "i16") v = EC_READ_S16(d);
                    else if (a.cfg.type == "u32") v = EC_READ_U32(d);
                    else                          v = EC_READ_S32(d);

                    // 表驱动分派。名字来自 pdo.yaml，加对象只改配置 + 这张表。
                    if      (a.cfg.name == "motor_position")
                        io->motor_position_sdo = static_cast<int32_t>(v);   // ← 独立通道，
                                                                           //   不再覆盖 PDO 的 motor_position
                    else if (a.cfg.name == "dual_encoder_diff")
                        io->twist_counts = static_cast<int32_t>(v);
                    else if (a.cfg.name == "drive_temperature")
                        io->drive_temp_C = static_cast<uint16_t>(v);
                    ecrt_sdo_request_read(a.req);
                    break;
                }
```

**注意**：`0x2240` 一旦进 PDO（Task 14），`io->motor_position` 由 `readInputs` 填、`io->motor_position_sdo` 由本函数填，**两个字段互不干扰**——这正是 J3 对拍的前提。在 Task 14 之前，`motor_position` 仍需由 SDO 通道填充，故此期间加一行过渡代码：

```cpp
                    // Task 14 完成前，PDO 里还没有 0x2240，主链路仍依赖 SDO 值
                    if (a.cfg.name == "motor_position" && !motor_position_in_pdo_)
                        io->motor_position = static_cast<int32_t>(v);
```

`motor_position_in_pdo_` 在构造时按 `off_.count("motor_position")` 置位。

- [ ] **Step 3: 错误计数透出**

`pollAsyncSdo` 的 ERROR 分支改为：

```cpp
                case EC_REQUEST_ERROR:
                    ++a.errors;
                    if (a.errors == 1 || a.errors % 1000 == 0)
                        logf("WARNING", "异步 SDO 0x%04X 读取失败 %llu 次",
                             a.cfg.index, (unsigned long long)a.errors);
                    ecrt_sdo_request_read(a.req);
                    break;
```

并把 `errors` 汇总进 `BusStatus`，让 GUI 能看到。

- [ ] **Step 4: `pdo.yaml` 的 `async_sdo` 段加温度**

```yaml
async_sdo:
  # 0x2240 在 Task 14 之后会进 PDO；这条通道保留作 J3 对拍用（写入 motor_position_sdo）
  - {index: 0x2240, sub: 0x00, type: i32, name: motor_position,    poll_divisor: 10}
  - {index: 0x2241, sub: 0x00, type: i32, name: dual_encoder_diff, poll_divisor: 100}
  # 0x22A2 在 ESI 里没有 PdoMapping 标志，只能走 SDO。
  # ⚠ 这是驱动器温度，不是绕组温度、更不是关节壳体温度。
  #   采样时刻与 PDO 不同步。1 kHz 下 poll_divisor=1000 即 1 Hz，
  #   正好压在实验计划 §4.2 温度最低采样率上。
  - {index: 0x22A2, sub: 0x00, type: u16, name: drive_temperature, poll_divisor: 1000}
```

- [ ] **Step 5: 验证**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
```

- [ ] **Step 6: 提交**

```bash
git add -A
git commit -m "fix(bus): 异步 SDO 按类型分派，0x2241 落地，0x22A2 接入

三处硬编码：request size 写死 4、EC_READ_S32 写死、接收端只有一个
if (name == 'motor_position') 分支。于是 pdo.yaml 里配的 0x2241 每 100 拍白读一次、
返回值直接丢弃——这条链路从来没被验证过。AsyncSdoCfg::type 早已解析却从未被使用。

新增 motor_position_sdo 独立字段：0x2240 进 PDO 后两个写入者会竞争同一字段，
J3 对拍必须有两个独立的落点。

0x22A2 驱动器温度以 1 Hz 接入。按手册 §27.4，h_obs 被内摩擦主导、
温度经油脂粘度直接影响它，因此温度是主标签的关键协变量而非附属信息。"
```

---

### Task 13: activate 前的一次性诊断读取与 metadata 落地

**Files:**
- Modify: `backend/src/ethercat/igh_bus.cpp`（activate 前调用）
- Modify: `backend/src/logger/data_logger.cpp:153-195`（metadata 写入）
- Modify: `backend/include/ecjc/data_logger.hpp`（`LogMeta` 扩展）
- Modify: `config/slave.yaml`（`diagnostic_sdos` 补全）

**Interfaces:**
- Consumes: Task 7 的 `SlaveCfg::diagnostic_sdos`
- Produces: `IghBus::diagnostics() -> const std::map<std::string,int64_t>&`

**背景**：`0x6093` Position factor **全工程零出现**——CiA402 里 `0x6064` 是**用户单位**，由 `0x6093` 换算，而工程把它当原始 counts。若不是 1:1，全部角度换算静默偏移（0.1% 即 ±20° 上 1.2 角分）。`0x6075` 是**可读可写**的（eTuner「持续电流」），而 `0x6078` 的换算依赖它——中途被改则全部历史电流数据标定漂移。这些都能在 activate 之前用阻塞式 SDO 读（`guardBlocking()` 在 `PreActivate` 相位放行，不触碰 D 状态死锁陷阱）。

- [ ] **Step 1: `slave.yaml` 的 `diagnostic_sdos` 补全**

```yaml
  # 只在 activate 之前 / deactivate 之后读的诊断 SDO（阻塞式）
  # ⚠ 绝不可在 OP 期间阻塞式读，会导致进程 D 状态死锁、只能重启机器
  diagnostic_sdos:
    - {index: 0x1001, sub: 0x00, type: u8,  name: "error_register"}
    - {index: 0x100A, sub: 0x00, type: u32, name: "firmware_version"}
    - {index: 0x1018, sub: 0x04, type: u32, name: "serial_number"}
    - {index: 0x6079, sub: 0x00, type: u32, name: "dc_link_voltage_mV"}
    - {index: 0x22A2, sub: 0x00, type: u16, name: "drive_temperature_C"}
    # ── 标定依赖项：全试验期间必须锁定不变 ──
    - {index: 0x6075, sub: 0x00, type: u32, name: "rated_current_mA"}
    - {index: 0x6076, sub: 0x00, type: u32, name: "rated_torque_mNm"}
    - {index: 0x6093, sub: 0x01, type: u32, name: "position_factor_numerator"}
    - {index: 0x6093, sub: 0x02, type: u32, name: "position_factor_divisor"}
    - {index: 0x607D, sub: 0x01, type: i32, name: "position_limit_min"}
    - {index: 0x607D, sub: 0x02, type: i32, name: "position_limit_max"}
    # ── 环路增益：影响 §4.7/§4.8 的六个指标，0h 后禁改 ──
    - {index: 0x2381, sub: 0x01, type: u32, name: "vel_loop_kp"}
    - {index: 0x2381, sub: 0x02, type: u32, name: "vel_loop_ki"}
    - {index: 0x2382, sub: 0x01, type: u32, name: "pos_loop_kp"}
    # ── SM 同步诊断：直接量化 0xA000 的根因，改映射前后各读一次 ──
    - {index: 0x1C33, sub: 0x04, type: u16, name: "sm_in_min_cycle_ns"}
    - {index: 0x1C33, sub: 0x05, type: u16, name: "sm_in_calc_copy_ns"}
    - {index: 0x1C33, sub: 0x09, type: u16, name: "sm_in_event_missed"}
    - {index: 0x1C33, sub: 0x0A, type: u16, name: "sm_in_cycle_too_small"}
```

> **注意**：`slave.yaml:36` 原有的 `{index: 0x1003, ...}` 已删除——`0x1003` **不在 ESI 对象字典里**（109 个对象都没有），读它要么 abort、要么读未声明对象。

- [ ] **Step 2: activate 前读取并缓存**

在 `IghBus::activate()` 内、`ecrt_master_activate()` 调用**之前**加：

```cpp
    // 阻塞式 SDO 只能在 activate 之前调用（guardBlocking 在 PreActivate 相位放行）。
    // 在 OP 期间调会让进程进入 D 状态、fd 永不释放、只能重启机器。
    for (const auto& d : cfg.slave.diagnostic_sdos) {
        int64_t v = 0;
        std::string e;
        if (blockingSdoReadTyped(d.index, d.sub, d.type, &v, &e)) {
            diagnostics_[d.name] = v;
        } else {
            logf("WARNING", "诊断 SDO 0x%04X:%02X (%s) 读取失败: %s",
                 d.index, d.sub, d.name.c_str(), e.c_str());
            diagnostics_[d.name] = INT64_MIN;      // 哨兵值，metadata 里写 "read_failed"
        }
    }
    // 位置因子非 1:1 是静默错误的最大来源，单独校验一次
    {
        auto num = diagnostics_.find("position_factor_numerator");
        auto div = diagnostics_.find("position_factor_divisor");
        if (num != diagnostics_.end() && div != diagnostics_.end() &&
            num->second != INT64_MIN && div->second != INT64_MIN &&
            num->second != div->second) {
            logf("WARNING",
                 "0x6093 位置因子不是 1:1（%lld/%lld）——全部角度换算需按该因子修正！",
                 (long long)num->second, (long long)div->second);
        }
    }
```

- [ ] **Step 3: metadata 落地**

`data_logger.cpp` 的 metadata 写入区追加（用已有的 `attrStr` / `attrDbl` lambda）：

```cpp
    attrDbl("gear_ratio", meta_.gear_ratio);
    attrStr("gear_ratio_source",
            "手册§12 n_out=n_motor/(X+1) + §2表2-1；实测 Δ0x2240/Δ0x6064=30.234"
            "（自身精度仅 0.05%，足以排除 120 不足以细分）");
    attrStr("encoder_accuracy_note",
            "型号说明称 HM 可提供 20 位/±7角秒；表2-2 eRob80H 选装配置列 19Bit/±10角秒；待厂家澄清");
    attrStr("torque_est_source",
            "0x3B69 与 0x2241 同源同刻计算，驱动器手册称『估计』，全机无独立力传感元件；"
            "不得用于刚度退化判定");
    attrStr("temperature_scope",
            "仅驱动器温度 0x22A2，非绕组非壳体；单位未经手册核实（eTuner 界面显示为 ℃）；"
            "异步 SDO，采样时刻与 PDO 不同步");
    attrStr("twist_sign_convention",
            "theta_twist = theta_out - theta_in/i（与计划§4.4 TE 同号）");
    attrStr("git_commit",   meta_.git_commit);
    attrStr("config_sha256", meta_.config_sha256);
    // 全部诊断 SDO 实读值
    for (const auto& kv : meta_.diagnostics) {
        if (kv.second == INT64_MIN) attrStr(("sdo_" + kv.first).c_str(), "read_failed");
        else                        attrDbl(("sdo_" + kv.first).c_str(), (double)kv.second);
    }
```

`LogMeta` 加字段：

```cpp
    std::string git_commit;
    std::string config_sha256;
    std::map<std::string, int64_t> diagnostics;
```

`main.cpp` 启动时填充 `git_commit`（`git rev-parse --short HEAD`，构建时经 CMake 注入宏）与 `config_sha256`（对 config 目录做一次 sha256）。

- [ ] **Step 4: 验证（mock 下诊断读取会失败，属预期）**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
ls -t data/*.h5 | head -1 | xargs -I{} /home/tyy/miniconda3/envs/zeroError/bin/python -c "
import h5py, sys
f = h5py.File(sys.argv[1]); g = f['/experiment']
for k in sorted(g.attrs): print(k, '=', g.attrs[k])
" {}
```

Expected: 属性里出现 `git_commit`、`config_sha256`、`gear_ratio_source`、以及 `sdo_*` 一组

- [ ] **Step 5: 提交**

```bash
git add -A
git commit -m "feat(bus): activate 前读诊断 SDO，metadata 落地标定来源与 git hash

0x6093 Position factor 此前全工程零出现——CiA402 里 0x6064 是用户单位、由 0x6093 换算，
而工程把它当原始 counts。若不是 1:1，全部角度换算静默偏移（0.1% 即 ±20° 上 1.2 角分）。

0x6075 是可读可写的（eTuner「持续电流」），而 0x6078 的换算 mA = 0x6078 × 0x6075/1000
依赖它——中途被人改过则全部历史电流数据的标定就漂了。必须实读并锁定记录。

删除 slave.yaml 里的 0x1003：该对象不在 ESI 对象字典中（109 个对象都没有）。

metadata 此前只写了 4 项，scaling.yaml 的 gear_ratio_source 根本没被解析。
现补齐全部标定来源声明 + git hash + config 哈希，让数据能绑回代码版本。"
```

---

### Task 14: `0x1A00` PDO 重映射（配置就位，先不上电）

**Files:**
- Modify: `config/pdo.yaml`（`0x1A00` 段）
- Create: `tools/verify_pdo_remap.py`（分级验证脚本）

**Interfaces:**
- Consumes: Task 11~13 的全部改动
- Produces: 可上电验证的配置；`tools/verify_pdo_remap.py` 输出 J1~J6 的判定

**依据**：ESI 中 `0x2240`/`0x2241` 的 `<PdoMapping>T</PdoMapping>` 均存在但**不在任何预定义 TxPDO 内**；`0x1A00` 是全 ESI **唯一** `Fixed="0"` 的 TxPDO，其 `Exclude` 仅含 `0x1A01~0x1A04`（本设计一个都不用）；手册 §16.1 明文支持任意 mapping、TxPDO 上限 76 字节。当前 36 → 加 `0x1A00`(8) + `0x1A18`(4) + `0x1A19`(4) = **52 字节**。

- [ ] **Step 1: `pdo.yaml` 加 `0x1A00`（先注释掉，Task 15 分级放开）**

```yaml
    # ── 0x1A00：ESI 中唯一 Fixed="0" 的 TxPDO，可改写条目表 ──
    # 手册 §16.1：「0x1A00/0x1600 映射 TxPDO/RxPDO 支持任意 mapping 配置
    #              （TxPDO、RxPDO 最大字节数各为 76 字节）」
    # Exclude 仅含 0x1A01~0x1A04，本工程一个都不用，可与 1A06/07/0D/1F/08/18/19 共存。
    #
    # 为什么必须做：0x2241 由驱动器在两个编码器同一 50µs 采样点上算出（手册 §22）。
    # 若改由主站拿异步 SDO 的 0x2240 与 PDO 的 0x6064 相减，时间偏斜 τ 会引入
    # 2·ω_out·τ 的误差——正反向符号相反，在回程误差里**翻倍不抵消**。
    # 5 rpm、τ=5ms（异步 SDO 实测抖动）⇒ 约 18 角分，是 h_obs（0.4~1.5 角分）的 20~45 倍。
    #
    # ⚠ Task 15 分级放开：先只放 0x2240，确认进 OP 后再放 0x2241。
    # - index: 0x1A00
    #   entries:
    #     - {index: 0x2240, sub: 0x00, bits: 32, type: i32, name: motor_position_pdo}
    #     - {index: 0x2241, sub: 0x00, bits: 32, type: i32, name: twist_counts}
```

- [ ] **Step 2: 写分级验证脚本**

新建 `tools/verify_pdo_remap.py`：

```python
#!/usr/bin/env python3
"""PDO 重映射的 J1~J6 判定。只读，不改任何配置。

用法（后端已在跑、伺服不使能）：
    python3 tools/verify_pdo_remap.py --expect-bytes 44   # 只加了 0x2240
    python3 tools/verify_pdo_remap.py --expect-bytes 52   # 全部加完
"""
import argparse, re, subprocess, sys, time

ECAT = "/usr/local/bin/ethercat"


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True).stdout


def j1_dmesg():
    """J1: 映射写失败在 IgH 里只是 WARN 不阻断状态机，唯一权威判据是 dmesg。"""
    out = run(["dmesg"])
    hits = [l for l in out.splitlines() if "Failed to configure mapping" in l]
    return (len(hits) == 0), hits


def j2_domain_bytes(expect):
    """J2: ethercat pdos 取的是 idle 相位的 SII 扫描快照，可能永远显示出厂映射。
    用 domain 实际字节数判定。"""
    out = run(["pkexec", ECAT, "domains"])
    m = re.search(r"(\d+)\s+byte", out)
    if not m:
        return False, f"无法解析 domain 字节数:\n{out}"
    got = int(m.group(1))
    return got == expect, f"domain = {got} 字节，期望 {expect}"


def j6_sm_timing():
    """J6: SM3 长度变化后，1ms 周期还剩多少余量。"""
    rows = {}
    for sub, name in [("4", "min_cycle_ns"), ("5", "calc_copy_ns")]:
        out = run(["pkexec", ECAT, "upload", "-p0", "0x1C33", sub, "--type", "uint16"])
        rows[name] = out.strip()
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--expect-bytes", type=int, required=True)
    a = ap.parse_args()

    ok1, hits = j1_dmesg()
    print(f"J1 dmesg 无映射失败      : {'PASS' if ok1 else 'FAIL'}")
    for h in hits:
        print("   ", h)

    ok2, msg = j2_domain_bytes(a.expect_bytes)
    print(f"J2 domain 字节数         : {'PASS' if ok2 else 'FAIL'}  {msg}")

    print("J6 SM3 同步时序（参考值）:", j6_sm_timing())

    slaves = run(["pkexec", ECAT, "slaves"])
    print("从站状态:", slaves.strip())
    ok_op = " OP " in slaves or slaves.strip().split()[2] == "OP"
    print(f"   进入 OP               : {'PASS' if ok_op else 'FAIL'}")

    sys.exit(0 if (ok1 and ok2 and ok_op) else 1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 语法与只读性自检**

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python -m py_compile tools/verify_pdo_remap.py && echo "语法 OK"
grep -n "upload\|download\|write" tools/verify_pdo_remap.py
```

Expected: 语法 OK；只有 `upload`（读），没有 `download`/`write`

- [ ] **Step 4: 提交**

```bash
git add -A
git commit -m "feat(pdo): 备好 0x1A00 重映射配置（默认注释）与 J1~J6 验证脚本

0x2240/0x2241 在 ESI 里有 PdoMapping=T 但不在任何预定义 TxPDO 内；0x1A00 是全 ESI
唯一 Fixed=0 的 TxPDO，Exclude 仅含 1A01~1A04（本工程一个都不用）。手册 §16.1 明文支持。

配置先注释掉，由 Task 15 分级放开：一次只动一个变量。

验证脚本刻意不用 'ethercat pdos'——它取的是 idle 相位的 SII 扫描快照，
可能永远显示出厂映射；改用 dmesg + domain 字节数。"
```

---

### Task 15: 真机分级验证与回退演练

**Files:**
- Modify: `config/pdo.yaml`（分级放开 `0x1A00`）
- Modify: `backend/include/ecjc/realtime_task.hpp:108`（默认模式改 CSP，**最后一步**）

**Interfaces:**
- Consumes: Task 14 的配置与脚本
- Produces: 经真机验证的总线采集层

> **本任务需要上电，且需要人在场。** 前置条件：P0 全部完成；`ctest` 全绿；`integration_test.py` 全过；配置已备份（`config.bak.20260811/` 与 `/etc/ethercat-joint-control.bak.20260811` 已存在且经 diff 验证一致）。
>
> **全程伺服不使能**，直到 Step 8。

- [ ] **Step 1: 记录基线**

```bash
pkexec /usr/local/etc/init.d/ethercat start
pkexec /usr/local/bin/ethercat master | head -6
pkexec /usr/local/bin/ethercat slaves
./build/ecjc-backend --config config &
sleep 3
/home/tyy/miniconda3/envs/zeroError/bin/python tools/verify_pdo_remap.py --record-baseline
/home/tyy/miniconda3/envs/zeroError/bin/python tools/verify_pdo_remap.py --expect-delta 0
```

Expected: `--record-baseline` 记下当前 dmesg 标记行与 domain 字节数（终审 finding C2 之后，`ethercat domains` 报的是 Rx+Tx 合计，当前配置下本机实测应为 **60 字节** = RxPDO 0x1605 16 字节 + TxPDO 0x1A06/07/0D/1F/08/18/19 合计 44 字节；`--expect-delta 0` 校验"相对刚记的基线没有变化"）；从站 OP；dmesg 无映射失败（J1 现在带 pkexec 读权限并检查 returncode，见 finding C1）

记下 J6 的 `0x1C33:05/06` 读数作为**改映射前的基线**。

```bash
pgrep -x ecjc-backend | xargs -r kill
```

- [ ] **Step 2: 只放开 `0x2240`**

`config/pdo.yaml` 取消注释，但**只留第一条 entry**：

```yaml
    - index: 0x1A00
      entries:
        - {index: 0x2240, sub: 0x00, bits: 32, type: i32, name: motor_position_pdo}
```

```bash
./build/ecjc-backend --config config &
sleep 5
/home/tyy/miniconda3/envs/zeroError/bin/python tools/verify_pdo_remap.py --expect-delta 4
```

Expected: J1 PASS（dmesg 无 `Failed to configure mapping of PDO 0x1A00`）；J2 domain = 基线 + 4 = 64；从站 OP

**若卡在 SAFEOP 或 dmesg 有告警 → 立即执行 Step 3 回退，不要继续。**

- [ ] **Step 3: 回退演练（无论 Step 2 成功与否都做一次）**

这一步的目的是把"理论上能退"变成"验证过能退"。

```bash
pgrep -x ecjc-backend | xargs -r kill
# 把 0x1A00 整段重新注释掉
./build/ecjc-backend --config config &
sleep 5
/home/tyy/miniconda3/envs/zeroError/bin/python tools/verify_pdo_remap.py --expect-delta 0
```

Expected: 回到基线字节数（60）、从站 OP、dmesg 干净 ⇒ **回退路径验证通过**

> 注意：驱动器断电重启**不是**有效的回退手段。IgH 的 `fsm_pdo.c` 里 `// always write PDO mapping` 意味着每次 activate 都无条件重写，只要 `pdo.yaml` 没改，重启后端会把同样的映射再写一遍。**唯一的回退是改配置文件。**

- [ ] **Step 4: 恢复 `0x2240` 并加上 `0x2241`**

```yaml
    - index: 0x1A00
      entries:
        - {index: 0x2240, sub: 0x00, bits: 32, type: i32, name: motor_position_pdo}
        - {index: 0x2241, sub: 0x00, bits: 32, type: i32, name: twist_counts}
```

```bash
./build/ecjc-backend --config config &
sleep 5
/home/tyy/miniconda3/envs/zeroError/bin/python tools/verify_pdo_remap.py --expect-delta 8
```

Expected: J1 PASS；J2 domain = 基线 + 8 = 68；从站 OP

- [ ] **Step 5: J3 —— 两路 `0x2240` 静止对拍**

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python - <<'EOF'
import sys; sys.path.insert(0, 'tests')
from hw_driver import Client
c = Client(); c.connect()
ss = c.collect(seconds=3)
bad = [(s.motor_position_raw, s.motor_position_sdo) for s in ss
       if s.motor_position_raw != s.motor_position_sdo]
print(f"样本 {len(ss)}，不一致 {len(bad)}")
if bad[:5]: print("前 5 条:", bad[:5])
EOF
```

Expected: 静止状态下两路完全相等（不一致数为 0）。若不等，先怀疑映射偏移错位。

- [ ] **Step 6: J4 —— `0x2241` 符号方向**

手动向输出端施加一个正向力矩（用手扳，不使能），观察 `twist_counts` 变化方向。

Expected: 与手册图 22-1 一致（正扭矩 → Δθ 正）。若反号，在 `scaling.yaml` 记录 `twist_direction: -1` 而**不要**改代码符号。

- [ ] **Step 7: J5 —— 恒等式回归**

低速运动（**仍不使能，用手匀速转输出端**，或在确认前六步全绿后以 2 rpm 使能低速运行）：

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python - <<'EOF'
import sys; sys.path.insert(0, 'tests')
import numpy as np
from hw_driver import Client
c = Client(); c.connect()
ss = c.collect(seconds=20)
Cm = np.array([s.motor_position_raw   for s in ss], dtype=float)
Co = np.array([s.output_position_raw  for s in ss], dtype=float)
D  = np.array([s.twist_counts         for s in ss], dtype=float)
# 拟合 k：Cm - k*Co = D  →  Cm - D = k*Co
k, resid, *_ = np.linalg.lstsq(Co.reshape(-1,1), (Cm - D), rcond=None)
k = float(k[0])
res = (Cm - D) - k*Co
print(f"拟合 k = {k:.5f}   （理论 30.25 = 121/4）")
print(f"残差 RMS = {np.sqrt((res**2).mean()):.2f} counts "
      f"= {np.sqrt((res**2).mean())*1.3619417e-3:.4f} 角分")
print("J5:", "PASS" if 30.23 <= k <= 30.26 else "FAIL")
EOF
```

Expected: `k ∈ [30.23, 30.26]`，残差 RMS 在编码器噪声量级（几个 counts）

> **不要钉死 k=30.25 再看残差**——那样判据必然失败。回归拟合 k 才是正确做法。
> 该判据**不能独立验证减速比**：`0x2241` 是驱动器用它自己的内部系数算的，恒等式成立只证明主站系数与驱动器系数一致。

- [ ] **Step 8: J6 —— 改映射前后的 SM 时序对比**

子索引与位宽按 Task 13 的修正核对（不是 :04/:05/:09/:10），与 `tools/verify_pdo_remap.py` 的 J6 用同一套：

```bash
pkexec /usr/local/bin/ethercat upload -p0 0x1C33 5   --type uint32  # Minimum Cycle Time
pkexec /usr/local/bin/ethercat upload -p0 0x1C33 6   --type uint32  # Calc and Copy Time
pkexec /usr/local/bin/ethercat upload -p0 0x1C33 0xB --type uint16  # SM-Event Missed
pkexec /usr/local/bin/ethercat upload -p0 0x1C33 0xC --type uint16  # Cycle Time Too Small
```

或者直接跑 `tools/verify_pdo_remap.py`——J6 已经按这套子索引/位宽实现，输出里会带上这四个值：

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python tools/verify_pdo_remap.py --expect-delta 8
```

Expected: `Calc and Copy Time` 相对 Step 1 的基线有小幅增长（SM3 从 36 → 52 字节，+44%），但仍远小于 1 ms；`SM-Event Missed` 与 `Cycle Time Too Small` 为 0

- [ ] **Step 9: 低速带载验证（此时才使能）**

确认前八步全绿后，以 **CSV 模式、2 rpm（电机侧约 242 rpm）**运行 60 秒，确认：数据连续、`twist_counts` 随负载变化合理、无 `0x3B68` 警告、母线电压稳定。

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python tests/jitter_probe.py --seconds 60
```

- [ ] **Step 10: 默认模式改 CSP（P0 已完成才允许）**

`backend/include/ecjc/realtime_task.hpp:108`：

```cpp
    std::atomic<int>  desired_mode_{static_cast<int>(OpMode::CSP)};
```

`gui/widgets/mode_panel.py:52`：

```python
        self._mode = "CSP"
```

在 CSP 下重复 Step 9，并**专门验证 Task 2 的修复**：跑一条有限时长的梯形轨迹（0° → 5°），确认跑完之后目标位置**停在 5° 而不是跳回 0°**。

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python - <<'EOF'
import sys; sys.path.insert(0, 'tests')
from hw_driver import Client
c = Client(); c.connect()
ss = c.collect(seconds=5)          # 轨迹结束后再采 5 秒
tail = ss[-500:]
tp = [s.target_position_deg for s in tail]
print(f"轨迹结束后目标位置: min={min(tp):.3f} max={max(tp):.3f}")
print("Task 2 修复验证:", "PASS" if min(tp) > 4.0 else "FAIL —— 目标跳回起始位置了")
EOF
```

- [ ] **Step 11: 停机与提交**

```bash
pgrep -x ecjc-backend | xargs -r kill
sleep 3
pkexec /usr/local/bin/ethercat master | head -4    # 应回到 Phase: Idle
git add -A
git commit -m "feat(pdo): 0x1A00 重映射经真机 J1~J6 验证通过，默认模式改 CSP

分级验证：先只加 0x2240（domain 44→48）→ 回退演练验证退路真的通 →
加 0x2241（→52）→ J3 两路对拍 → J4 符号 → J5 回归拟合 k → J6 SM 时序余量。

全程伺服不使能直到 Step 9。默认模式改 CSP 放在最后，且前置于 P0 Task 2 的修复
（CSP 停止不再跳回 Run 起始位置）——否则一切到 CSP 该缺陷就从潜伏变必然。"
```

---

### Task 16: 电机侧转速改为独立测量，GUI 只信后端的标定数

**Files:**
- Modify: `backend/src/ethercat/scaling.cpp:36`
- Modify: `backend/include/ecjc/scaling.hpp`（差分器）
- Modify: `gui/config.py`、`gui/widgets/config_panel.py`
- Modify: `tests/test_scaling.cpp`（追加）

**Interfaces:**
- Consumes: Task 15 之后 `0x2240` 已在 PDO 里（差分才有意义）
- Produces: `Scaling::motorVelocityFromCounts(int32_t counts, double dt_s) -> double`

**依赖**：必须在 Task 15 之后做——`0x2240` 还走异步 SDO 时，对陈旧值做差分毫无意义。

**背景**：`scaling.cpp:36` 的 `motor_vel_rpm = output_vel_rpm × gear_ratio` **不是测量，是恒等式**。双编码器关节的全部价值就在于电机端与输出端**不是刚性关系**——`ω_in/i − ω_out` 恰恰是扭转角速率、就是信号本身。这行代码以构造的方式强制该差为零，logged `motor_velocity_rpm` 携带的独立信息量**严格为零**，而 GUI 把它标为"电机侧转速"呈现为独立读数。

- [ ] **Step 1: 写失败测试**

```cpp
// 电机侧转速必须来自 0x2240 的差分，而不是输出转速 × 减速比。
// 判据：给定一组"电机多走了一点"的计数序列，算出的电机转速应高于
// output_vel × gear_ratio，差值即扭转角速率。
TEST(motor_velocity_is_measured_not_derived) {
    ScalingConfig c;
    Scaling s(c);
    // 1 ms 内电机侧走了 1322 counts ≈ 605 电机 rpm
    CHECK_NEAR(s.motorVelocityFromCounts(1322, 0.001), 605.0, 1.0);
    CHECK_NEAR(s.motorVelocityFromCounts(-1322, 0.001), -605.0, 1.0);
    CHECK_NEAR(s.motorVelocityFromCounts(0, 0.001), 0.0, 1e-9);
}
```

- [ ] **Step 2: 运行确认失败**

```bash
cmake --build build -j 2>&1 | tail -3
```

Expected: `motorVelocityFromCounts` 未声明

- [ ] **Step 3: 实现差分 + 滑窗回归微分器**

```cpp
double Scaling::motorVelocityFromCounts(int32_t delta_counts, double dt_s) const {
    if (dt_s <= 0) return 0.0;
    return static_cast<double>(delta_counts) / c_.motor_counts_per_rev / dt_s * 60.0;
}
```

`scaling.cpp:36` 改为使用 `motor_pos_raw` 的差分（在 `Scaling` 内保留上一拍计数）。1 kHz 单拍中心差分的量化噪声 σ ≈ 0.19 电机 rpm；用 **N=21 点中心线性回归微分器**可压到 ≈0.017 电机 rpm，带宽约 24 Hz。

> **窗口不能再放宽**：谐波减速机的波发生器 2× 特征频率在 5 rpm 工况下正是 **20.2 Hz**，与该带宽临界。放宽窗口会把要观测的特征滤掉。

- [ ] **Step 4: GUI 改为只信后端发布的标定数**

`gui/config.py:27` 现在自行找 `scaling.yaml`（`_find_config_dir()` 与后端的 `--config` 无关，可能是不同目录）。改为从后端已发布的 statusJson（`ipc_server.cpp:412-413` 已在发 `gear_ratio`/`encoder_verified`）取值，本地 YAML 只作为未连接时的占位并在界面上标注"未连接，显示的是本地配置"。

- [ ] **Step 5: 验证 + 提交**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
```

```bash
git add -A
git commit -m "fix(scaling): 电机侧转速改为 0x2240 差分测量；GUI 只信后端标定数

原实现 motor_vel_rpm = output_vel_rpm × gear_ratio 是恒等式不是测量，强制
ω_in/i − ω_out ≡ 0——而那个差恰恰是扭转角速率、是信号本身。logged motor_velocity_rpm
的独立信息量严格为零，GUI 却把它当独立读数显示。

差分器用 N=21 点中心线性回归（σ≈0.017 电机 rpm，带宽约 24 Hz）。窗口不能再放宽：
波发生器 2× 特征频率在 5 rpm 下是 20.2 Hz，与该带宽临界。"
```

---

## 自查记录

**Spec 覆盖**：spec v2 的 §3（P0 四条）→ Task 1~5；§5.1/5.2/5.3 → Task 14~15；§5.4 → Task 11；§5.5 → Task 12；§5.6 → Task 13；§5.7 → Task 7/8/13/16；§5.8 → Task 8；§6.1 → Task 10；§6.2 → Task 9；§6.3 标定声明 → Task 13；§11 测试策略 → Task 6。

**已知需在实现时补齐的接口**（不是占位符，是明确的待建函数）：

- `IghBus::blockingSdoReadTyped(uint16_t index, uint8_t sub, const std::string& type, int64_t* out, std::string* err)` —— Task 13 需要。现有接口是 `blockingSdoRead(index, sub, void*, size, err)`，需在其上包一层按类型解码。**必须保留 `guardBlocking()` 的相位检查**。
- `logf(level, fmt, ...)` —— Task 12/13 用到。若 `IghBus` 只有 `log(level, string)`，用 `snprintf` 到栈缓冲后调用它。
- `tests/hw_driver.py` 的 `Client.collect(seconds=N)` —— Task 15 的三个验证脚本假设了这个 API。实现前先读 `hw_driver.py:27-99` 确认实际方法名，不一致就按实际的改。

**未纳入本计划**（属 P2/P3/P4，spec §9 已列）：CSV 导出与扩展命名模板、`.meta.yaml` 旁文件、HDF5 切片、梯形往复与正弦起振窗、往复序列原语、指标计算与当场红绿灯、循环计数落盘续算、自动停机判据、systemd 看门狗链、日志轮转。

## 完成判据

**P0**：`ctest` 全绿；`integration_test.py` 34 项全过；GUI 三个危险入口都有门控；慢客户端不再拖死 IPC；`git log` 有 5 条提交。

**P1**：`0x1A00` 重映射经 J1~J6 真机验证；回退演练通过；端到端落盘的总线对象从 7 个增至 19 个；`0x6093`/`0x6075`/`0x6076` 已实读并写进 metadata；HDF5 含 37 列（25 + 12）与完整标定来源声明；默认模式 CSP 且 Task 2 的修复经真机确认。

**遗留到后续期**（不在本计划范围）：CSV 导出与扩展命名模板（P2）、梯形往复与起振窗（P3）、循环计数落盘续算与自动停机判据（P4）。
