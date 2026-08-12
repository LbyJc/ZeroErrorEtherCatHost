# EtherCAT Robot Joint Control Software — 总体架构设计（Step 1）

针对任务书《基于 Linux + IgH EtherCAT Master 的机器人关节实时控制与可视化上位机开发任务》。
本文对应任务书第五十四节 Step 1 要求的 16 项内容。

目标硬件：ZeroErr eRob80H120I-BHM-18ET[V6] 一体化关节 + Intel I210 网卡 + IgH EtherCAT Master 1.5.4。

---

## 0. 先说三条硬约束（它们决定了后面所有设计）

这三条不是理论推导，是 2026-08-10 在这台机器上实测踩出来的，架构必须绕开：

### 0.1 阻塞式 SDO 只能在 activate 之前或 deactivate 之后调用

主循环停止后、`ecrt_master_deactivate()` 之前调用阻塞式 `ecrt_master_sdo_upload()`，此时主站仍在 OP
模式而没有任何线程再 `send/receive` 泵总线 → 调用线程进入 **D 状态（不可中断睡眠）**，`kill -9` 无效，
`/dev/EtherCAT0` 的 fd 永不释放，主站永远 `Active: yes`，`rmmod` 永远 "in use"，
**唯一恢复手段是重启机器**（`rmmod -f` 会 panic）。

> **架构约束**：`RealtimeTask` 内部对 SDO 的访问一律走 `ecrt_slave_config_create_sdo_request()`
> 创建的**异步**请求；阻塞式 SDO 封装成 `IghMaster::blockingSdoRead/Write()`，并用一个运行期断言
> 保证它只在 `Phase::PreActivate` 或 `Phase::PostDeactivate` 被调用，其它相位直接返回错误而不是真的去调。
> 这条规则写进 `igh_master.cpp` 的注释里，不允许后来者"优化"掉。

### 0.2 内核是 PREEMPT_DYNAMIC，不是 PREEMPT_RT

`uname -r` = 6.8.0-137-generic，`PREEMPT_RT` 未启用。这意味着：

- 1 kHz 周期可以稳定跑，但**周期抖动是几十微秒量级**，偶发调度延迟可达毫秒。
- 250 µs / 100 µs 周期在本内核上不保证，配置里允许设，但 GUI 必须如实显示抖动与 deadline miss，
  **不能假装是硬实时**。
- 因此：抖动统计（max / p99 / deadline miss count）是一等公民指标，不是可选装饰。

架构上留好升级路径：换 PREEMPT_RT 内核后不需要改任何代码，只是抖动数字变好。

### 0.3 普通用户拿不到实时优先级，`/dev/EtherCAT0` 是 0600 root:root

`ulimit -r` = 0。所以 **Backend 必须以 root 运行**（`mlockall` + `SCHED_FIFO` + 打开 `/dev/EtherCAT0`），
而任务书第十一节明确禁止整个 GUI 跑 root。

> **架构约束**：进程边界必须切在 GUI 与 Backend 之间。GUI 普通用户 → Unix socket → root Backend。
> 这不是"推荐做法"，是这台机器的权限现实逼出来的唯一解。

---

## 1. 系统总体架构

```
┌─────────────────────────────────────────────────────────────┐
│  GUI 进程   ecjc-gui   (普通用户 tyy, 无特权)                 │
│  PySide6 + PyQtGraph                                        │
│  ┌────────────┬────────────┬───────────┬──────┬──────────┐  │
│  │  Monitor   │ Parameter  │  Data     │ Log  │  System  │  │
│  │            │  Tuning    │ Acquisition│      │  Config  │  │
│  └────────────┴────────────┴───────────┴──────┴──────────┘  │
│         ▲ 遥测(binary, 50Hz 批)      ▼ 命令(JSON, <10Hz)     │
└─────────┼───────────────────────────┼───────────────────────┘
          │      Unix Domain Socket (SOCK_STREAM)             
          │      /run/ethercat-joint-control/control.sock     
          │      属主 root:ethercat  权限 0660                 
┌─────────┴───────────────────────────┴───────────────────────┐
│  Backend 进程  ecjc-backend  (root, systemd service)         │
│                                                             │
│  ┌───────────────┐   telemetry_ring   ┌──────────────────┐  │
│  │ Realtime      │══════════════════▶ │ IPC / Telemetry  │  │
│  │ Thread        │   (SPSC 无锁)       │ Thread           │  │
│  │ SCHED_FIFO 80 │                    │ SCHED_OTHER      │  │
│  │ 1 kHz         │──── log_ring ─────▶│                  │  │
│  │               │        ║           └──────────────────┘  │
│  │ · PDO 收发     │        ║                    ▲            │
│  │ · CiA402 SM   │        ║           ┌─────────┴────────┐  │
│  │ · 轨迹发生器   │        ╚══════════▶│ Logger Thread    │  │
│  │ · Controller  │                    │ SCHED_OTHER      │  │
│  │ · 物理量换算   │                    │ → HDF5 批量写盘   │  │
│  └───────┬───────┘                    └──────────────────┘  │
│          │ ▲ ParameterBlock (双缓冲 + 原子代号)               │
│          │ └──────────────── 在线调参 ─────────────────       │
│  ┌───────▼──────────────────────────────────────────────┐   │
│  │ IEtherCATBus  (抽象接口)                              │   │
│  │   ├── IghBus     : 真实 IgH EtherCAT Master           │   │
│  │   └── MockBus    : 关节动力学仿真 + CiA402 仿真         │   │
│  └───────┬──────────────────────────────────────────────┘   │
└──────────┼──────────────────────────────────────────────────┘
           │ ecrt_* API
    ┌──────▼──────────┐
    │ IgH Master 1.5.4│ ─── enp3s0 ──▶ ZeroErr eRob 关节
    └─────────────────┘
```

**为什么是两个进程而不是一个**：见 0.3。GUI 崩溃不能带走正在使能的伺服；Backend 以 root 常驻，
GUI 可以随意重启、连断。两者用 socket 解耦后，Mock 模式下 GUI 开发完全不需要硬件。

---

## 2. Process / Thread 模型

| 线程 | 进程 | 调度 | 周期 | 职责 | 禁止 |
|---|---|---|---|---|---|
| **Realtime** | backend | SCHED_FIFO 80 | `cycle_us`（默认 1000） | PDO 收发、DC、CiA402、轨迹、控制器、物理量换算、写两个 ring | 磁盘 IO、malloc、printf、锁、socket |
| **IPC/Telemetry** | backend | SCHED_OTHER | 事件驱动 + `gui_publish_hz` | socket accept/读命令/发遥测、参数落到 ParameterBlock | 阻塞 RT |
| **Logger** | backend | SCHED_OTHER nice 5 | 批量（`flush_every` 或 200 ms） | 从 log_ring 取样本、写 HDF5、统计落盘量 | 阻塞 RT |
| **Watchdog** | backend | SCHED_OTHER | 10 Hz | 检查 RT 心跳、WC 错误、从站掉 OP → 触发安全状态 | — |
| **GUI main** | gui | 普通 | Qt 事件循环 | 界面、绘图（`gui.plot_fps`，默认 50 Hz） | 任何阻塞调用 |

RT 线程启动时做：`mlockall(MCL_CURRENT|MCL_FUTURE)` → 栈预触（8 MB 写 0）→ 预分配所有 ring →
`pthread_setschedparam(SCHED_FIFO)` → `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` 绝对时间唤醒
（不是相对 sleep，否则误差累积）。

---

## 3. 数据流

### 3.1 下行（GUI → 关节）
```
GUI 控件
  → JSON 命令 {"cmd":"set_param","name":"vel_kp","value":1.5}
  → socket
  → IPC 线程解析、校验
  → ParameterBlock 的 staging 副本，写完 generation++ (memory_order_release)
  → RT 线程每周期检查 generation (acquire)，变了才 memcpy 到本地活动副本
```
RT 线程**永不加锁**、永不等待。参数最坏延迟 1 个周期。

### 3.2 上行（关节 → GUI / 磁盘）
```
RT 线程组装 Sample(144 B)
  ├─▶ log_ring       (容量 = sample_rate × 8 s)  → Logger 线程 → HDF5
  └─▶ telemetry_ring (容量 = sample_rate × 2 s)  → IPC 线程 → 抽稀 → socket → GUI
```
两个独立 ring 的理由：Logger 要**每一个**样本（长时间实验数据完整性），GUI 只要够画图。
共用一个 ring 会让 GUI 断连时拖累 Logger，或者反过来 Logger 卡盘时丢实验数据。分开后互不影响，
各自单独统计 `dropped`。

**抽稀策略**：`gui_publish_hz` = 50 时，1 kHz 数据每 20 个一组。不是简单取第一个，而是
**每组保留 min/max 两点**（对 position/velocity/current 各自取），这样 GUI 上不会因为抽稀丢掉尖峰——
调伺服时尖峰恰恰是最要看的东西。

### 3.3 Ring Buffer
单生产者单消费者无锁环形队列，容量取 2 的幂用位与代替取模：

```cpp
template <typename T, size_t CapacityPow2>
class SpscRing {
    alignas(64) std::atomic<size_t> head_{0};   // 生产者写
    alignas(64) std::atomic<size_t> tail_{0};   // 消费者写
    alignas(64) std::array<T, CapacityPow2> buf_;
    // push: 满则丢弃并 dropped_++，绝不阻塞 RT
};
```
`alignas(64)` 避免 head/tail 伪共享。RT 侧 `push()` 满了就丢并计数——**宁可丢遥测也不能拖慢 RT**。

---

## 4. IPC 方案选择与理由

**选择：Unix Domain Socket (SOCK_STREAM) + 自定义帧，遥测走二进制、命令走 JSON。**

| 候选 | 否决理由 |
|---|---|
| 共享内存 + 信号量 | 没有连接生命周期。GUI 崩了 Backend 不知道；GUI 重启后要处理陈旧共享段和残留信号量。而且 GUI 是 Python，映射 C 结构体虽可行但错一个字节就是段错误。 |
| ZeroMQ / gRPC | 引入重依赖。1 kHz × 144 B = 144 kB/s，本地 socket 毫无压力，杀鸡用牛刀。gRPC 还要 protoc 工具链。 |
| TCP socket | 无谓地暴露到网络，还要处理端口占用。本机通信没有理由走 IP 栈。 |
| **Unix socket** | **✓** 有连接语义（GUI 断开 = `EPOLLHUP`，Backend 立刻知道并可自动进入安全状态）；文件系统权限即访问控制（`root:ethercat 0660`，不用自己写认证）；`QLocalSocket` 原生接入 Qt 事件循环，GUI 侧零轮询；`socat` 可直接抓包调试。 |

**帧格式**（小端）：
```
struct FrameHeader { uint32 magic='ECJC'; uint16 type; uint16 version; uint32 length; }
  type=1 TELEMETRY  payload = N × Sample(144B)     // 高频、二进制、固定布局
  type=2 JSON       payload = UTF-8 JSON           // 低频、自描述
```
混合的理由：遥测追求效率且 schema 稳定，用固定二进制；命令/状态/日志追求可读可扩展且频率极低，
用 JSON。两者共用一条流，靠 `type` 区分，不需要第二个连接。

`Sample` 的 144 字节布局在 `types.hpp` 里用 `static_assert(sizeof(Sample)==144)` 锁死，
Python 侧 `struct` 格式串 `<q14d2i2I2HbBBB` 与之一一对应，并在 GUI 启动时用一条自检消息校验尺寸，
**尺寸不匹配直接报错而不是画出乱码曲线**。

---

## 5. Real-Time Thread 设计

```cpp
void RealtimeTask::run() {
    clock_gettime(CLOCK_MONOTONIC, &wakeup);
    for (;;) {
        wakeup += cycle_ns;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, nullptr);
        // ---- 抖动统计：实际唤醒 - 期望唤醒 ----
        jitter = now() - wakeup;  stats_.update(jitter);

        bus_->receive();                 // ecrt_master_receive + domain_process
        readPdoInputs(&raw_);            // 原始计数
        scaling_.toPhysical(raw_, &phys_);  // → deg / rpm / A / Nm，含多圈 unwrap

        params_.refreshIfChanged();      // 无锁取新参数
        cia402_.update(raw_.statusword); // 解析状态字 → CiA402State
        appSm_.update(cia402_, bus_state, faults);   // 应用层状态机

        if (appSm_.running()) {
            traj_->eval(t, &setpoint);           // 轨迹发生器
            ctrl_->update(setpoint, phys_, &out);// 控制器（CST 自研算法入口）
        } else {
            safety_.rampToSafe(&out);            // 停止时按斜坡归零
        }

        cw_ = cia402_.controlwordFor(desiredState_);  // 06→07→0F 逐级
        writePdoOutputs(cw_, out);
        buildSample(&sample_);
        log_ring_.push(sample_);         // 满则丢，不阻塞
        gui_ring_.push(sample_);

        bus_->send();                    // DC 同步 + ecrt_master_send
    }
}
```

**RT 线程里绝对没有的东西**：`new`/`malloc`、`std::string` 构造、`printf`、`std::mutex`、
文件 IO、socket 调用、阻塞 SDO。需要报错时往 `log_ring` 塞一个**预分配的定长记录**（枚举码 + 几个数值），
文字化留给 Logger 线程做。

**DC**：`ecrt_master_application_time` → `sync_reference_clock` → `sync_slave_clocks` → `send`。
本机曾出现 `Slave did not sync after 5000 ms`（DC 不收敛，进 OP 慢 5 秒），所以
`ethercat.yaml` 里 `dc_enabled` 可关，关掉后退化为 SM 同步，1 kHz 下 CSV/CSP 依然可用。

---

## 6. CiA402 模块设计

**纯逻辑、零 EtherCAT 依赖** —— 这是能写单元测试的前提。

```cpp
enum class Cia402State { NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn,
                         SwitchedOn, OperationEnabled, QuickStopActive,
                         FaultReactionActive, Fault, Unknown };

struct StatuswordBits { bool ready_to_switch_on, switched_on, operation_enabled,
                        fault, voltage_enabled, quick_stop, switch_on_disabled,
                        warning, remote, target_reached, internal_limit, ms1, ms2; };

Cia402State     decodeState(uint16_t sw);        // 掩码匹配，纯函数
StatuswordBits  decodeBits(uint16_t sw);
ControlwordBits decodeCw(uint16_t cw);

class Cia402StateMachine {
    // 关键：给定「当前状态 + 目标状态」返回本周期该发的控制字。
    // Servo Enable 时严格 0x06 → 0x07 → 0x0F 逐级推进，绝不无条件发 0x0F。
    uint16_t controlwordFor(Cia402State cur, Cia402Target want);
    // Fault Reset 是 bit7 的「上升沿脉冲」：先保证 bit7=0 至少一周期，再置 1 若干周期，再清零。
    void requestFaultReset();
};
```
单元测试覆盖：8 个状态的掩码解码（含 `xxxx xxxx x0xx 0000` 这类通配）、
使能序列必须经过 0x06/0x07/0x0F 三步、Fault Reset 脉冲形状、非法跳转被拒绝。

---

## 7. Controller 模块设计

```cpp
struct ControlContext {          // 只读输入
    double t;                    // 运行时间 s
    double dt;                   // 周期 s
    Setpoint  ref;               // 轨迹给定：pos_deg / vel_rpm / trq_Nm
    JointState act;              // 实测：电机侧/输出侧 pos vel、电流、力矩
    const ParameterView& p;      // 在线参数（只读快照）
};
struct ControlOutput { double target_pos_deg, target_vel_rpm, target_trq_Nm; };

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual void reset() = 0;                                    // Run 开始时调用
    virtual void update(const ControlContext&, ControlOutput*) = 0;
    virtual const char* name() const = 0;
    virtual void declareParams(ParameterRegistry&) = 0;          // 自报参数给 GUI
};
```

`declareParams()` 是关键设计：控制器**自己声明**有哪些可调参数（名字、单位、范围、默认值），
IPC 线程把这张表发给 GUI，**GUI 的调参页面是根据它动态生成的**。
这样以后加 RBFNN / Backstepping 控制器，只写一个 .cpp，GUI 不用改一行——
任务书第三十八节要的"统一接口"落到实处就是这个。

第一阶段实现：`PassthroughController`（直通，验证链路）、`PidPositionController`、
`PidVelocityController`、`CstPidController`。
预留空壳：`AdaptiveController`、`RbfnnController`、`BacksteppingController`。

---

## 8. Trajectory 模块设计

```cpp
struct Setpoint { double pos_deg, vel_rpm, trq_Nm; bool finished; };
class TrajectoryBase {
public:
    virtual void   start(const JointState& q0) = 0;   // 以当前位置为起点，避免突跳
    virtual void   eval(double t, Setpoint*) = 0;
    virtual double duration() const = 0;              // <0 表示无限
};
```
实现：`Constant` / `Sine`(offset,amp,freq,phase,duration) / `Ramp`(v0,v1,T) /
`Triangle`(offset,amp,freq) / `Trapezoidal`(target,vmax,acc,dec) / `CsvFile`（time,target… 线性插值）。

`CsvFile` 的文件在 **Run 之前**全部读入预分配数组（RT 线程内不许碰磁盘），运行时只做二分查找 + 插值。

---

## 9. Data Logger 设计

**格式：HDF5（列存）**，一个字段一个 chunked、可扩展 dataset。

```
/experiment                      (group)
  @test_name @description @operation_mode @controller @cycle_us @sampling_hz
  @slave_name @vendor_id @product_code
  @motor_encoder_counts_per_rev @output_encoder_counts_per_rev @gear_ratio
  @software_version @start_time @end_time @control_params(JSON)
  /system_time_ns      int64   chunk=4096  
  /elapsed_time_s      float64
  /motor_position_deg  float64
  ...（任务书第三十四节全部字段）
```

列存而非行存的理由：分析时通常只取两三列画图，列存可以只读需要的部分；
同列数据相邻，`gzip` 压缩率远高于行存。

**长时间稳定性**（任务书要求 10–20 h）：
- 1 kHz × 24 字段 × 8 B ≈ 190 kB/s ≈ **6.6 GB / 10 h**（未压缩）。开 gzip level 4 后约 2–3 GB。
  当前磁盘余量 417 GB，充裕。GUI 上实时显示剩余空间，低于 `min_free_gb`（默认 5）时告警并停止采集。
- 每 `rotate_hours`（默认 2 h）自动切一个新文件，避免单文件过大且一旦损坏全丢。
- 写盘批大小 1000 样本（1 s），`H5Dwrite` 后不每次 flush，每 30 s 一次 `H5Fflush`——
  兼顾崩溃鲁棒性和写放大。
- Logger 线程**永远不阻塞 RT**：ring 满时 RT 侧丢弃并计数，`dropped_samples` 上报 GUI 实时显示。
  这个数字非零就说明磁盘跟不上，是要在实验记录里如实体现的。

**CSV 导出**：不作为主存储（任务书明确要求）。提供 `tools/h5_to_csv.py` 与 GUI 按钮，
按时间区间和字段子集导出。

---

## 10. GUI 页面布局

```
┌──────────────────────────────────────────────────────────────────────────┐
│ Master:Running │ EtherCAT:OP │ Slave:OP │ Servo:Op Enabled │ Mode:CSV    │  ← 顶部状态栏
│ Run:Running │ Rec:● ON │ Cycle:1.000ms │ Jitter:18µs │ Miss:0            │    (始终可见)
├───────────────┬──────────────────────────────┬───────────────────────────┤
│ 左：系统       │ 中：模式 / 轨迹 / 目标 / 控制器  │ 右：实时数值                │
│ ┌───────────┐ │ ┌──────────────────────────┐ │ ┌───────────────────────┐ │
│ │EtherCAT   │ │ │ Mode  ○PP ○PV ○PT ○Homing│ │ │ 电机侧                 │ │
│ │ 启动主站   │ │ │       ●CSP ○CSV ○CST     │ │ │  位置  123.456 °      │ │
│ │ 停止主站   │ │ │ 6060=8  6061=8  ✓一致    │ │ │  多圈  1483.456 °     │ │
│ │ 重新连接   │ │ ├──────────────────────────┤ │ │  转速  99.8 rpm       │ │
│ │ 打开配置   │ │ │ Trajectory               │ │ │  电流  0.31 A         │ │
│ │ 打开数据   │ │ │  ●Constant ○Sine ○Ramp   │ │ ├───────────────────────┤ │
│ │ 打开日志   │ │ │  ○Triangle ○Trapz ○CSV   │ │ │ 输出侧                 │ │
│ ├───────────┤ │ │  Offset/Amp/Freq/Phase   │ │ │  位置  1.020 °        │ │
│ │CiA402     │ │ ├──────────────────────────┤ │ │  多圈  12.257 °       │ │
│ │ SW 0x1637 │ │ │ Target   [ 100.0 ] rpm   │ │ │  转速  0.83 rpm       │ │
│ │ ●Op Enab  │ │ │ (单位随模式自动切换)       │ │ ├───────────────────────┤ │
│ │ CW 0x000F │ │ ├──────────────────────────┤ │ │ 力矩  1.24 Nm         │ │
│ │ bit 明细   │ │ │ Controller ▼ PID_Vel     │ │ │ 位置误差 0.003 °      │ │
│ ├───────────┤ │ │  (参数在调参页动态生成)    │ │ │ 速度误差 0.2 rpm      │ │
│ │Servo      │ │ ├──────────────────────────┤ │ └───────────────────────┘ │
│ │ Enable    │ │ │  【开始运行】【停止运行】   │ │                           │
│ │ Disable   │ │ └──────────────────────────┘ │                           │
│ │ Fault Rst │ │                              │                           │
│ │ Quick Stop│ │                              │                           │
│ └───────────┘ │                              │                           │
├───────────────┴──────────────────────────────┴───────────────────────────┤
│ Plot1 位置(目标/电机/输出)  │ Plot2 转速(目标/电机/输出)                    │
│ Plot3 电流                 │ Plot4 力矩(目标/实际)                        │
│ 每图：☑Auto Y / [Ymin][Ymax] │ X 轴：5/10/30/60/300s/Custom               │
└──────────────────────────────────────────────────────────────────────────┘
  Tab: [Monitor] [Parameter Tuning] [Data Acquisition] [Log] [System Config]
```

**状态灯一律"颜色 + 文字 + 形状"三重编码**（任务书第十七节明确要求不能只用颜色）：
`● Operation Enabled` / `▲ Fault` / `○ Switch On Disabled`。

**Plot 内存**：每条曲线用固定长度 numpy 环形数组，长度 = `x_window_s × gui_publish_hz`。
切换时间窗只换视图不重新分配。**GUI 内存不随实验时长增长**——10 小时实验和 10 秒实验占用一样。

---

## 11. systemd Service 设计

```ini
# /etc/systemd/system/ethercat-joint-control.service
[Service]
Type=notify
ExecStart=/opt/ethercat-joint-control/bin/ecjc-backend --config /etc/ethercat-joint-control
User=root
RuntimeDirectory=ethercat-joint-control        # 自动建 /run/... 并在停止时清理
RuntimeDirectoryMode=0750
LimitRTPRIO=99
LimitMEMLOCK=infinity
CPUAffinity=2 3                                # 与 GUI/其它负载隔离
Restart=on-failure
```
socket 建在 `RuntimeDirectory` 下，属主 `root:ethercat`、权限 0660。
安装脚本建 `ethercat` 组并把当前用户加进去——这样 GUI 不需要任何特权就能连。

**为什么用 `Type=notify`**：Backend 完成 master 请求 + 从站扫描 + PDO 配置后才 `sd_notify(READY=1)`，
`systemctl start` 返回即代表真的就绪，GUI 不用轮询猜。

**"启动主站"按钮的权限路径**：GUI（普通用户）→ `pkexec /opt/.../ecjc-helper start` →
helper（root）负责 `ethercat` 主站模块 + `systemctl start ecjc-backend`。
配套 polkit policy 让弹一次图形密码框即可，**不保存任何密码**（任务书第十一节）。

> 本机注意：systemd 的 `ethercat.service` 读 `/usr/local/etc/ethercat.conf`，
> 而该文件里 `MASTER0_DEVICE=""` 是空的，所以 `systemctl start ethercat` 必失败；
> 实际可用的是 `/usr/local/etc/init.d/ethercat start`（读 `/etc/sysconfig/ethercat`）。
> `ecjc-helper` 先试 systemd、失败回落 init.d，并把用的哪条路径写进日志。

---

## 12. Desktop Application 启动流程

双击 `EtherCAT Joint Control` 图标后：

```
加载配置 (config/*.yaml)
  → 自检 Python 环境与 Sample 结构体尺寸
  → 检查网卡 enp3s0 是否存在、Link 状态
  → 检查 IgH 内核模块 / /dev/EtherCAT0
  → 尝试连接 Backend socket
      ├── 连上   → 读状态 → 显示
      └── 没连上 → 显示「Backend: Stopped」，等用户点【启动主站】
  → 显示系统状态面板
```
**默认不做**：不自动 Servo Enable、不自动 Run、不自动产生力矩、不自动开始采集
（任务书第十二节）。`app.yaml` 里 `auto_start_master` 可选开启，但 Enable 与 Run 永远手动。

【启动主站】的九步进度，每步实时回报 `{"ev":"startup","step":"...","ok":true/false,"msg":"..."}`，
GUI 逐条点亮 `[✓]`。失败时显示的是具体原因而不是错误码——
例如"配置网卡 enp3s0 不存在，请检查 ethercat.yaml"。

---

## 13. 完整目录结构

```
ethercat_joint_control/
├── CMakeLists.txt            顶层，含 tests
├── README.md
├── install.sh / uninstall.sh
├── docs/ARCHITECTURE.md      本文
├── config/                   app / ethercat / slave / pdo / scaling / gui
│                             / trajectory / controller .yaml
├── backend/
│   ├── include/ecjc/         types config ring_buffer cia402 scaling trajectory
│   │                         controller parameter ethercat_bus igh_bus mock_bus
│   │                         realtime_task ipc_server data_logger log
│   ├── src/
│   │   ├── ethercat/         igh_bus.cpp pdo_manager.cpp
│   │   ├── cia402/           cia402.cpp
│   │   ├── realtime/         realtime_task.cpp
│   │   ├── trajectory/       trajectory.cpp
│   │   ├── controllers/      controller.cpp pid_controller.cpp
│   │   ├── communication/    ipc_server.cpp
│   │   ├── logger/           data_logger.cpp
│   │   ├── parameter/        parameter.cpp
│   │   └── mock/             mock_bus.cpp
│   └── main.cpp
├── gui/
│   ├── main.py  main_window.py  ipc_client.py  app_state.py  config.py
│   └── widgets/  status_bar  system_panel  cia402_panel  mode_panel
│                 trajectory_panel  plot_panel  data_record_panel
│                 parameter_panel  log_panel
├── system/  *.service  *.desktop  *.policy  ecjc-helper
├── tests/   test_cia402  test_scaling  test_trajectory  test_ring_buffer
├── tools/   h5_to_csv.py
├── data/    实验数据 (HDF5)
└── logs/    应用日志
```

---

## 14. 关键 Class

**C++**
| Class | 职责 |
|---|---|
| `Config` | 加载 8 个 yaml，提供强类型访问，启动时校验 |
| `SpscRing<T,N>` | 无锁环形队列，RT ↔ Logger / IPC |
| `Cia402StateMachine` | 状态解码 + 使能序列 + Fault Reset 脉冲 |
| `Scaling` | raw ↔ 物理量，多圈 unwrap，方向取反 |
| `IEtherCATBus` / `IghBus` / `MockBus` | 总线抽象，Mock 与真实同接口 |
| `PdoManager` | 按 `pdo.yaml` 分配 PDO、注册 domain entry、算偏移 |
| `RealtimeTask` | RT 主循环，见 §5 |
| `TrajectoryBase` + 6 个实现 | 轨迹发生 |
| `ControllerBase` + 实现 | 控制算法，自报参数 |
| `ParameterBlock` | 双缓冲 + 原子 generation 的无锁参数传递 |
| `IpcServer` | Unix socket、帧编解码、命令分发 |
| `DataLogger` | HDF5 列存、切文件、metadata |
| `AppStateMachine` | DISCONNECTED…RUNNING…FAULT 九态 |

**Python**
| Class | 职责 |
|---|---|
| `IpcClient(QObject)` | `QLocalSocket` 收发，解帧，发 Qt 信号 |
| `AppState(QObject)` | 单一数据源，持有最新状态，驱动按钮使能 |
| `PlotPanel` | 4 图 + 环形 numpy 缓冲 + 轴控制 |
| `ParameterPanel` | 按 Backend 上报的参数表**动态生成**控件 |
| 其余 8 个 Panel | 见 §10 |

---

## 15. 模块间接口

### Backend → GUI
- `type=1` 二进制 `Sample[]`（144 B/个，见 §4）
- `type=2` JSON 事件：
  - `{"ev":"status", master, ethercat, slave, servo, mode, run, recording, cycle_us, jitter_us, jitter_max_us, deadline_miss, wc, wc_state, dc_ok, link, dropped_log, dropped_gui, buffer_usage}`
  - `{"ev":"startup","step":"PDO Configured","ok":true,"msg":""}`
  - `{"ev":"log","level":"INFO","ts":...,"msg":"..."}`
  - `{"ev":"params","controller":"PID_Vel","items":[{name,unit,min,max,default,value}...]}`
  - `{"ev":"recording", file, size_bytes, samples, dropped, disk_free_gb, elapsed_s}`
  - `{"ev":"ack","cmd":"servo_enable","ok":false,"msg":"当前 EtherCAT 未进入 OP，禁止使能"}`

### GUI → Backend
`{"cmd": ...}`：`connect_bus` `disconnect_bus` `reconnect` `servo_enable` `servo_disable`
`fault_reset` `quick_stop` `homing` `set_mode` `set_trajectory` `set_target` `set_controller`
`set_param` `start_run` `stop_run` `record_start` `record_stop` `get_params` `shutdown`

每条命令都回 `ack`，**失败必须带人话原因**（任务书第四十三节）。

---

## 16. 第一阶段开发计划

| Step | 内容 | 验收 |
|---|---|---|
| 1 | 架构设计 | 本文 |
| 2 | 工程骨架 + Mock | `cmake --build` 通过；`--mock` 下 GUI 出曲线；无需硬件 |
| 3 | CiA402 + Scaling + Trajectory + 单元测试 | `ctest` 全绿 |
| 4 | IgH 真实后端 | 从站进 OP，状态字实时刷新 |
| 5 | IPC 打通 | GUI 显示真实数据 |
| 6 | 桌面化 | .desktop / systemd / pkexec 启停主站 |
| 7 | CSP/CSV/CST | 三种模式实机可跑 |
| 8 | 轨迹 | 6 种轨迹 |
| 9 | 绘图 | 4 图 + 轴控制 |
| 10 | HDF5 Logger | 长时间采集 + metadata |
| 11 | 在线调参 | 参数表动态生成 |
| 12 | 自研控制器 | CST 下挂自定义算法 |

---

## 17. 已知硬件参数（2026-08-10 实测，直接写进 config）

任务书第五十三节要求"不要编造 PDO 地址，等提供 ESI 后再适配"。这些参数已经实测完毕，
因此 `config/` 里填的是**真实值**而非占位：

| 项 | 值 | 来源 |
|---|---|---|
| Vendor ID / Product Code | `0x5a65726f` / `0x00029252` | `ethercat slaves -v` |
| 网卡 / 周期 | `enp3s0` / 1000 µs | 现场 |
| Rx PDO | `0x1609`（控制字/最大力矩/目标位置/目标速度/运行模式） | ESI V3.2.0 |
| Tx PDO | `0x1A06`+`0x1A07`+`0x1A0D`+`0x1A1F`+`0x1A08` | ESI，`0x1A05` 之后无 Exclude 可叠加 |
| 电机侧位置 | `0x2240`（**不在任何 PDO 里，只能异步 SDO**） | ESI |
| 输出侧位置 | `0x20A0` | ESI |
| 电机侧分辨率 | 131072 counts/rev (2^17) | 实测反推 |
| 输出侧分辨率 | 524288 counts/rev (2^19) | **物理转角实测确认**（半圈法，见下） |
| 减速比 | 121:1（铭牌标称 120） | 实测 Δ2240/Δ6064 = 30.24 ≈ 121/4 |
| 额定电流 / 力矩 | 6300 mA / 31000 mNm | `0x6075` / `0x6076` |
| `0x60FF` 单位 | `0x6064` 的 counts/s，`velocity_gain_correction` 现为 1.0（未应用任何实测速度标定；此前"增益 1.0006"是跟随误差与减速比推算的混合产物，不是单位标定） | 实测，100 rpm 电机侧 ↔ 7207 |
| 支持模式 | `0x6502`=0x38D：pp/pv/tq/csp/csv/cst，**无 Homing** | SDO |

**输出侧分辨率的验证方法（半圈法）**：两侧编码器都是 32 位累加多圈量，
不会在一圈处回绕，所以拿不到回绕模数，只能靠物理转角实测。
陷阱是**不能走整圈**——524288 counts 在 2^19 下是 1 圈、2^18 下是 2 圈，
记号两种情况都回原位，等于没测。走半圈（262144 counts）三种假设才完全分开：
2^19 → 停在对面；2^18 → 回到原位；2^20 → 只转 90°。
2026-08-10 实测走 263509 counts，记号停在对面 → **2^19 确认**。脚本见 `tests/verify_encoder.py`。

> ⚠️ 剩下一条需要在 GUI 上如实标注的限制：
> 本驱动器 `0x6502` 不含 Homing。GUI 的 Homing 按钮保留，但点击会明确提示
> "本驱动器不支持 Homing (0x6502=0x38D)"，而不是发一条注定失败的命令。

另有两条运行期已知现象，写进日志说明而非当故障：
- 每次 `ecrt_master_deactivate()` 后驱动器会记录一条 `0x603F=0xA000`（通讯中断），
  **不置 FAULT 位、不锁存**，属正常。
- 未装 3.6 V 多圈电池时上电必报 `0x730F`（编码器电池欠压），Fault Reset 清不掉，
  需向 `0x2242` 写 1。GUI 检测到 `0x730F` 时直接给出这句提示和一个【重置负载端编码器】按钮。
