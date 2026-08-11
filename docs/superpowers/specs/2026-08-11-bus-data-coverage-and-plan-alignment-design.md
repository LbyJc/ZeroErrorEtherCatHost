# 上位机总线数据全覆盖与实验计划对齐 — 设计 v2

- 日期：2026-08-11（v2；v1 同日，经四轮对抗审核后重写）
- 工程：`/home/tyy/Desktop/ethercat_joint_control/`
- 对象：ZeroErr eRob80H120I-BHM-18ET[V6]，IgH EtherCAT Master 1.5.4，当前 `cycle_us = 1000`
- 上游文档：
  - 《整机关节不可拆在线寿命实验方案》（文件名 V4.0 / 正文标题 V3.2）
  - 《2026-08-11 实验计划修订建议 v2》（`~/Desktop/`，本设计按其口径实现）
  - eRob 模组手册 V3.42、驱动器手册 V3.14
  - `ZeroErr+Driver_V3.2.0.xml`（ESI）

## v2 相对 v1 的变化

v1 经四轮独立对抗审核（手册 / 计划 / 总线 / 代码全貌 + 交叉仲裁），结果：

- **v1 的验证判据 J1~J5 基本全废**（§5.3），失败模式判断方向是错的
- **重映射的论证理由要换**，v1 用的理由不够硬，正确的论证使收益大两个数量级（§5.1）
- **发现一条会损坏硬件的既有缺陷**，且计划要求的模式切换会激活它 → 新增 **P0 止血期**（§3）
- **P1 范围要扩**——若干"硬前置"v1 没纳入（§9）
- 减速比错误的**受害对象判断有误**：主标签几乎不受影响，死的是 `TE_pp`（§2.1）

## 0. 第一原则

**以手册和总线实测为准；实验计划里本上位机测不了的，就不测。**

测不了的量（输出端实际转矩、电机绕组温度、关节壳体温度、振动）由外部传感器/性能试验台解决，不在本上位机范围。本设计不用近似量冒充这些字段。

---

## 1. 目标与非目标

**目标**

1. 把 EtherCAT 总线上这只关节能读到的物理量**全部**纳入采集并端到端落盘
2. 数据存储与文件格式对齐计划附录 A.1 / A.4 与 §4.2 命名规范（按修订 v2 的口径）
3. 上位机在停止采集时自动计算它能算全的节点指标
4. 具备支撑 450 h 无人值守的基础能力

**非目标**：外部转矩/温度/振动接入；`k1/k2/k3` 与 `hysteresis_area` 的计算（需外部转矩基准）；摆臂加载装置控制。

---

## 2. 关键事实（经四轮审核确认，构成后续设计的前提）

### 2.1 传动比 = 121，但主标签几乎不受影响

手册 §12 `n_out = n_motor/(X+1)` + §2 表 2-1「输出端转一圈，电机端转减速比+1 圈」。

**受害对象的正确判断**（v1 判错）：

`TE_算(θ) = θ_out − θ_in/120 = −θ_out/120 − ε/120`，第一项**只依赖 θ_out、与方向无关**。因此：

| 量 | 影响 |
|---|---|
| `h_obs`（正反向差分） | **几乎不受影响**，只被放大 121/120 = +0.83%，`t_b` 严格不变 |
| `TE_pp` / `TE_load_sensitivity` | **彻底失效**，60 s @5 rpm 产生 900 角分斜坡 |
| `k1/k2/k3` | 准静态、θ_out 几乎不动 ⇒ 近乎恒定偏置，斜率几乎不受影响 |

> **一般规律**：TE 差分（回程误差）**消除一切"偶"误差**（仅依赖 θ_out：减速比、CPR、`0x6093`、编码器偏心与绝对精度），**放大一切"奇"误差**（速度相关、方向反号：时间偏斜、摩擦滞后）。
> 抵消**仅在 forward/reverse 插值到同一 θ_out 栅格时成立**。

**实测证据强度如实标注**：`Δ0x2240/Δ0x6064 = 30.234` 的自身精度只有约 0.05%——`hw_driver.py` 差分的是 `Sample.motor_position_raw`，而那是**异步 SDO 的陈旧值**，5 ms 抖动在 10 s 窗口上恰好产生该量级误差。**足以排除 120，不足以细分。**

### 2.2 主标签测的是"机械连接间隙 + 内摩擦×柔性"

手册 §27.4：「**由于减速器齿轮啮合部的间隙控制为『0』**，因此齿隙量是指**机械结构连接时产生的间隙**」；「滞后损失主要由于**内部摩擦**产生，因此转矩极小的情况下几乎不存在滞后损失」。

⇒ 对上位机的含义：`h_obs` 对**温度**高度敏感（温度→油脂粘度→摩擦），跑合期可能**下降**。因此 `temperature_drive_C` 虽不是壳体温度，仍必须逐样本记录并进入指标表作为协变量。

### 2.3 `0x2241` 是驱动器给出的同刻扭转角

手册 §22：`0x2241` = 谐波减速机总扭转角 Δθ，INT32，**17 位电机侧计数**；两个编码器采样周期均 **50 µs**，Δθ 在采样后立即计算，带宽取决于通信周期。

```
θ_twist[arcmin, 输出侧] = Δ × 21600 / (131072 × 121) = Δ × 1.3619417e-3
1 count = 0.0014 角分 = 0.0817 角秒（输出侧）
Δ = C_m − 30.25 × C_o        30.25 = 121/4，精确有理数
```

**`0x3B69` 是派生量**：与 `0x2241` 同源于同一对编码器、同一时刻计算，驱动器手册用词是「**估计**」，全机爆炸图无独立力传感元件。采集并记录，但**不得用于刚度退化判定**。

### 2.4 关键的手册-实现约束

| 事实 | 出处 | 对本设计的影响 |
|---|---|---|
| `0x1A00`/`0x1600` 支持任意 mapping，TxPDO/RxPDO 各 **76 字节** | 手册 §16.1 | 重映射是文档化功能；驱动器手册 §18.1 另说"总计 80 字节"，适用对象不同，以 76/方向为准 |
| 8 位对象需补空 8 位对齐 | 手册 §16.1 | 本设计新增全为 32 位，不涉及 |
| `0x6078` 单位是**额定电流千分比**，`mA = 0x6078 × 0x6075 / 1000` | 手册表 25-3 | `0x6075` **可读可写**（eTuner「持续电流」），必须总线实读并锁定 |
| 位置保护出厂默认 INT32 满量程 | 手册 §9.6 图 9-10 | 等于**没有限位** |
| CST/PT 触限位切电且**不抱闸** | 手册 §9.6 表 9-1 | 摆臂必须 CSP |
| 制动器只许 <10% 最大转速（**2.5 rpm**）动态制动 | 手册 §7.1 | 摆臂峰值 10.5 rpm，急停必须先减速 |
| 关节内部**无再生制动电路** | 手册 §3.3 注 | 母线电压必须连续记录（`0x6079`） |

---

## 3. P0 · 止血（最高优先级，先于一切）

四轮审核发现的、**会损坏硬件或使实验静默失败**的既有缺陷。这些不是本次新增功能，是现存代码的问题。

### 3.1 【阻断】CSP 停止会把位置指令跳回 Run 起始位置

**已在本机代码上逐处核实：**

```cpp
// realtime_task.cpp:411-417
case OpMode::CSP:
default:
    o->target_pos_deg = cfg_.stop_ramp.csp_hold_position
                            ? hold_position_deg_          // ← 不是"当前位置"
                            : joint_.output_pos_unwrapped_deg;
    done = true;
```

- `hold_position_deg_` **全工程只有一处赋值**：`realtime_task.cpp:247`，Run 启动那一拍
- 初值 `realtime_task.hpp:146` = **0**
- 默认 `csp_hold_position = true`（`config.hpp:96`）
- `config/trajectory.yaml:45` 的注释写「CSP 停止 = **保持当前位置**」——**与实现不符**
- `degToTargetPosition`（`scaling.cpp:54-59`）是**绝对**换算，0° → 计数 0

**触发路径**（前两条伺服仍保持使能）：

| 场景 | 后果 |
|---|---|
| 有限时长轨迹正常跑完 | 目标从终点**跳回起点**（0°→90° 走完后被命令回 0°） |
| 手动停止 / 安全联锁跌出 | 摆臂在 +30° 停止 → 指令跳回起始角 |
| 首次 Run 前撤使能 | 目标 = **绝对编码器零位** |

CSP 下驱动器不做 profile 限制（`0x6081`/`0x6083` 不生效），这是真正的位置阶跃。

**为什么至今没炸**：默认模式是 CSV（`realtime_task.hpp:108`）。**计划要求摆臂用 CSP，一切换这个缺陷就从潜伏变必然。**

**Mock 掩盖了它**：`mock_bus.cpp` 的 CSP 是钳在 30 rpm 的软 P 跟随器，表现为"慢慢滑回去"。

**修法**：CSP 分支跟随 `joint_.output_pos_unwrapped_deg`（即默认改 `csp_hold_position = false`，或在轨迹结束时把 `hold_position_deg_` 更新为终点）；并对三条路径统一加兜底——**目标位置与实测位置偏差超阈值即拒绝下发并报错**。

### 3.2 【阻断】所有停机路径绕过软停斜坡

```cpp
// realtime_task.cpp:462-466
void RealtimeTask::servoDisable() {
    run_req_.store(false);
    stopping_.store(true);                                     // 起斜坡
    desired_target_.store((int)Cia402Target::DisableVoltage);   // 同拍撤使能
}
```

`desired_target_` 在主循环 `realtime_task.cpp:210` 被**无条件消费**（无 `if (!stopping_)` 门）⇒ 控制字**下一拍（1 ms）**就是 Disable Voltage，**斜坡是装饰品**。

`main.cpp:161-164` 的停主站序列只等 **300 ms**，而 `csv_decel_rpm_per_s = 200` 下从摆臂峰值（10.5 输出 rpm = 1270 电机 rpm）减到零需 **6.4 秒**。

`0x605A`（quick stop option code）**从未配置**——`slave.yaml` 的 `startup_sdo` 只下发 `0x3B61` 一项，快停行为取决于驱动器 NVM 里碰巧存了什么。

**结合手册 §7.1**（>2.5 rpm 动态制动造成**永久性损坏**），450 h 内必然发生的"人为中断"与"故障触发"各是一次损坏机会。

**修法**：停机路径串行化——`servoDisable()` / `disconnect()` 必须**等软停真正完成**（`stopping_ == false` 且 `|实测转速| < 2.5 rpm`）才允许下 DisableVoltage，加超时上限；GUI 的 Servo Disable / 停止主站增加 `running` 门控与确认对话框；新增真正的"减速急停"按钮；`startup_sdo` 补 `0x605A`。

### 3.3 【阻断】IPC 阻塞式 send 持全局锁

`ipc_server.cpp:329-344`：fd 无 `O_NONBLOCK`、无 `SO_SNDTIMEO`、无慢客户端丢弃，且 `::send` 期间持 `send_mu_`。任一客户端停止读取 → telemetry 线程与 **accept 线程**一起卡死 ⇒ **重连也进不来**，而 RT 线程仍在跑。外部看后端已死，无机制发现或恢复。

**修法**：`SO_SNDTIMEO` 或非阻塞 + 慢客户端丢帧。

### 3.4 【阻断】不是 git 仓库

450 h 实验期间"这份数据是哪版代码产生的"无法回答。

**修法**：`git init` 并提交；HDF5 metadata 写入 git hash 与 config 哈希（见 §7.2）。

> 注：已核实桌面源码与 `/opt` 安装副本**当前一致**（二进制 md5 相同，`gui`/`tools`/`system` diff 无输出，`config` 仅 `app.yaml` 的两行为 `install.sh` 故意改写）。但这是纪律而非强制——机制是"源码树构建后拷贝"，无软链。

---

## 4. 范围边界：A.1 字段对账

| A.1 字段 | 能否 | 来源 |
|---|---|---|
| `theta_in_rad` | ✅ | `0x2240`（重映射后全速率同步） |
| `theta_out_rad` | ✅ | `0x6064`（权威源；`0x20A0` 同步采集做对拍，见 §5.4） |
| `omega_out_rad_s` | ✅ | `0x606C` |
| `omega_in_rad_s` | ✅ | `0x2240` **差分**（现状是 `output_vel × 121` 的恒等式，**不是测量**，见 §6.3） |
| `motor_current_A` | ✅ | `0x6078` × 总线实读的 `0x6075` |
| `timestamp_s` | ✅ | `elapsed_time_s` + `system_time_ns` |
| `speed_rpm_target` / `direction` / `amplitude_deg` / `frequency_hz` / `rep` | ✅ | 轨迹设定值 + 会话元数据 |
| `life_hours` | ✅（P4） | 上位机循环计数换算 |
| `theta_out_ref_deg` / `theta_in_ref_phase_deg` / `theta_in_phase_deg` / `phase_error_in_deg` | ✅ | 复位流程记录与计算 |
| `sample_id` / `baseline_stage` / `test_item` / `operator` / `notes` / `mounting_phase_mark` / `mounting_orientation` / `loader_coupled` / `gain_set_id` | ✅ 人工填 | GUI 实验元数据面板 |
| `load_percent_Tr` / `load_torque_Nm_target` | ✅ 人工填 | 外部加载装置**设定值** |
| `zero_approach_direction` | ✅ 固定 | `negative_to_zero` |
| **`load_torque_Nm_actual`** | ❌ | **输出列但留空**。注意：不是"测不到转矩"，而是 `0x3B69` 测的是**关节自估的传递转矩**，与本字段要求的**加载装置施加转矩**语义不同 |
| **`temperature_motor_C`** | ❌ | **输出列但留空**，总线无绕组温度 |
| **`temperature_joint_C`** | ❌ | **输出列但留空**，总线无壳体温度 |

**留空写法**：写**空字段**（相邻两个逗号），**不写 `NA`**。计划用 `NA` 表示"本可测量但本次跳过"（如 `k3=NA`），而这三列是**原理上不产出**，语义不同。空字段含义在 `.meta.yaml` 的 `empty_columns` 中声明。

### 4.1 扩展字段（计划之外、总线可提供）

| 字段 | 来源 | 价值 |
|---|---|---|
| `twist_counts` | `0x2241` | 同刻扭转角，TE / `theta_twist` / `h_obs` 的直接来源 |
| `torque_est_mNm` | `0x3B69` | 厂商扭矩估计（派生量） |
| `torque_actual_permille` | `0x6077` | 原始千分比（现在只存了换算后的 Nm） |
| `torque_ratio` | `0x3B6A` | 厂商力矩比 |
| `following_error_counts` | `0x60F4` | `tracking_error_rms` |
| `dc_link_voltage_mV` | `0x6079` | **母线电压——关节无再生制动电路，回灌过压是 450 h 的头号中断风险** |
| `warning_code` / `error_code` | `0x3B68` / `0x603F` | 逐样本记录，异常事件才有时间轴 |
| `temperature_drive_C` | `0x22A2` | **驱动器温度，非绕组非壳体**。按 §2.2 是 `h_obs` 的关键协变量 |
| `aux_position_raw` | `0x20A0` | 现被丢弃；与 `0x6064` 对拍（§5.4） |
| `position_counts_raw` | `0x6063` | 现被丢弃；**`0x6064/0x6063` 之比按定义就是 `0x6093` 位置因子**（§5.4） |
| `motor_position_sdo` | 异步 SDO `0x2240` | J3 对拍专用的**独立**字段（§5.3） |
| `statusword` bit11 | `0x6041` | 软限位超限指示 |

---

## 5. P1 · 总线采集层

### 5.1 PDO 重映射

**改动**：把 `0x1A00` 条目表改写为 `0x2240`(32b) + `0x2241`(32b)，与现有 `0x1A06/0x1A07/0x1A0D/0x1A1F/0x1A08` 一同分配到 `0x1C13`；另新增分配 `0x1A18`(`0x6079`) 与 `0x1A19`(`0x60F4`)。

**依据**：ESI 中 `0x2240`/`0x2241` 的 `<PdoMapping>T</PdoMapping>` 均存在但**不在任何预定义 TxPDO 内**；`0x1A00` 是全 ESI 唯一 `Fixed="0"` 的 TxPDO，其 `Exclude` 仅含 `0x1A01~0x1A04`（本设计一个都不用）；手册 §16.1 明文支持。

**收益（v1 的论证换成这个）**：

时间偏斜 τ 引入 `ω_out·τ`，而 ω_out 在正反向符号相反 ⇒

```
return_error = TE_f − TE_r = 真值 + 2·ω_out·τ      ← 不抵消，翻倍
```

| 方案 | τ | 误差 | 相对 h_obs（0.4~1.5 角分） |
|---|---|---|---|
| **当前实现**（异步 SDO 的 `0x2240` 与 PDO 的 `0x6064` 相减） | ~5 ms 抖动 | **≈18 角分** | **20~45 倍** |
| 改用 `0x2241`（驱动器同 50 µs 采样） | ≤50 µs | ≤0.18 角分固定偏置 | 可标定消除 |

改后随机误差约 **0.001 角分**，节点间可分辨变化量 **~0.005 角分**——**余量约两个数量级**，瓶颈转移到装夹重复性。

**免费的消偏手段**：时间偏斜误差严格 ∝ ω，真实失动量不随 ω 变。**在 ≥2 个速度（如 2 rpm 与 5 rpm）各跑一次回程误差并外推到 ω→0**，可把全部时间错位伪影分离掉。建议纳入节点流程。

**字节预算**：现 36 字节（`0x1A06` 16 + `0x1A07` 4 + `0x1A0D` 4 + `0x1A1F` 2 + `0x1A08` 10）→ + `0x1A00` 8 + `0x1A18` 4 + `0x1A19` 4 = **52 字节**，上限 76。
（注：`0x1A00` 的**默认**内容是 10 字节，8 字节是重映射**成功后**的值——见 J1。）

### 5.2 风险

| # | 风险 | 后果 |
|---|---|---|
| R1 | 从站拒绝新映射 | 主站按它以为的映射算 SM3 长度，从站按实际映射组帧。**大概率卡 SAFEOP**（AL 状态码 `0x001D`/`0x001E`）；**小概率长度碰巧对上进 OP，`0x6064` 的字节被当 `motor_position` 读出——静默数据污染** |
| R2 | ~~阻塞式 SDO 死锁~~ | **不适用**：`ecrt_slave_config_pdos()` 由主站状态机执行，不经过该路径 |
| R3 | 过程数据长度变化 | 需同步 **5 处**：`types.hpp` 的 `static_assert(sizeof(Sample))`、`offsetof` 断言、`types.hpp:80` 的格式串注释、`types.hpp:15` 的 `kProtocolVersion`、`gui/ipc_client.py` 的 `SAMPLE_FORMAT` |
| **R4** | **`data_logger.cpp` 的 `kCols[]`（:196-212）与写入序列 `COL_*`（:287起）靠位置耦合，无任何编译期或运行期校验** | 加列忘了加对应宏或顺序不一致 ⇒ **编译通过、运行不报错、数据静默写进错误的 dataset**。本次要新增 ~12 个字段，这是整个改动里最容易出错且最难发现的地方 |
| R5 | `0x2241` 符号与内部传动比未验证 | 手册说它是"负载端**折算至**电机端"，但折算用 121 还是 120 未知。若用 120，`0x2241` 内含 `0.25×C_o` 斜坡（±20° 上 ≈19.8 角分）。**可事后修正，但前提是同拍记录 `0x2241`/`0x2240`/`0x6064` 三个原始量** |
| R6 | 重复 `(index, sub)` 静默别名 | `ecrt_slave_config_reg_pdo_entry()` 线性扫描、第一个匹配即返回，无重复检查。若 `0x1A00` 里保留 `0x6064`，两次注册都拿到第一个偏移 |

**关键安全属性**：本改动只动 TxPDO（读方向），RxPDO 的 `0x1605` 不碰 ⇒ **配错的后果是读到垃圾数据，不是发出垃圾指令**。次生风险是垃圾反馈喂进控制器变成真实运动，故**验证第一步伺服不使能**。

### 5.3 验证判据（v1 的 J1~J5 基本全废，以下为重写）

**分级验证，一次只动一个变量**：① 只加 `0x2240` → ② 加 `0x2241` → ③ 加 `0x1A18`/`0x1A19`。

| 判据 | 方法 | 预期 |
|---|---|---|
| **J1** | **`dmesg \| grep "Failed to configure mapping of PDO"`** | 无输出。<br>*v1 用"停在 PREOP 即被拒"是错的*：`fsm_pdo.c:566-575` 里映射失败只是 `EC_SLAVE_WARN` 然后 `next_pdo_mapping()` 继续，**不置 error**；实际会卡 **SAFEOP** 或静默污染 |
| **J2** | `ethercat domains` 的字节数 = **52** | *v1 用 `ethercat pdos` 逐条比对不可靠*：`ioctl.c:311` 取的是 **idle 相位的 SII 扫描快照**，可能永远显示出厂映射 |
| **J3** | PDO 路径的 `0x2240` 与**独立字段** `motor_position_sdo` 静止时比对 | 完全相等。<br>*前提*：必须先给 `RawIo` 加独立的 `motor_position_sdo` 字段，否则 PDO 与 SDO **写同一个 `io->motor_position`** 互相打架 |
| **J4** | 手动向输出端加载 | Δθ 变化方向与手册图 22-1 一致 |
| **J5** | 运动中对 `Δθ` 与 `(C_m − k·C_o)` 做**线性回归拟合 k** | `k ∈ [30.23, 30.26]` 且残差 RMS < 编码器噪声。<br>*v1 钉死 k=30.25 再看残差是错的*：理论 30.25 与实测 30.234 差 0.053%，在 40° 行程上产生 **1.27 角分**残差斜坡，判据必然失败。<br>*v1 还声称 J5 能独立验证减速比——不成立*：`0x2241` 是驱动器用**它自己的内部系数**算的，恒等式成立只证明主站系数与驱动器系数一致 |
| **J6**（新增） | 改映射前后各读一次 `0x1C33:04`(Minimum Cycle Time) / `:05`(Calc and Copy Time) | SM3 从 36 → 52 字节（+44%）后，1 ms 周期仍有余量。**activate 之前阻塞读合法**，成本近乎为零 |

**回退**：`pdo.yaml` 中 `0x1A00` 段整段注释 → 重启后端（秒级）。

> **v1 的 L2「断电重启回出厂配置」是假安全网**：IgH 的 `fsm_pdo.c:536` 注释就是字面的 `// always write PDO mapping`，**每次 activate 都无条件重写全部已分配 PDO 的映射**（包括所有 `Fixed="1"` 的）。只要 `pdo.yaml` 没改，重启后端还会把同样的坏映射再写一遍。**真正的回退只有改配置文件一条路。**
>
> 反过来这也说明：现在这套配置能跑到 OP，**证明这台驱动器早就在接受条目表改写**。重映射 `0x1A00` 不是"第一次尝试"，只是"第 N+1 次，内容不同"——**增量风险比 v1 设想的小**。
>
> 相应地，工程里 `pdo.yaml` 与 `igh_bus.cpp` 那条「一致时 IgH 不会去写映射寄存器」的注释是**事实错误**，需一并修正。

### 5.4 已在总线上、却被丢弃的数据（零成本回收）

审核发现"已采集"是个假象——端到端落盘的只有 **7 个对象**：

| 对象 | 在 pdo.yaml | 读进 RawIo | 进 Sample | 落 HDF5 |
|---|:-:|:-:|:-:|:-:|
| `0x3B69` / `0x3B6A` | ✅ | ❌ **`readInputs()` 根本没读** | ❌ | ❌ |
| `0x2241` | ✅ 异步SDO | ❌ **读回后无接收字段，直接丢弃** | ❌ | ❌ |
| `0x20A0` / `0x6063` | ✅ | ✅ | ❌ 丢弃 | ❌ |
| `0x603F` / `0x3B68` | ✅ | ✅ | ❌ | ❌ |

`0x3B69`/`0x3B6A` 白占 6 字节过程数据从未被读；`0x2241` 的异步通道每 100 拍白跑一次。

**两个免费的验证**（数据此刻就在线上被扔掉）：

- **`0x6064 / 0x6063`**：CiA402 里 `0x6063` 是 *position actual value\**（内部增量），`0x6064` 是用户单位值——**两者之比按定义就是 `0x6093` 位置因子**。这是全链路最强的验证。
- **`0x6064` vs `0x20A0`**：`scaling.hpp:7` 声称"实测逐拍相等"，但**代码里没有任何运行时校验**，两者从未被对比过。同时可检验 `0x20A0` 是否为 2× 分辨率（即辅助编码器是否 20 位）。

### 5.5 异步 SDO 通道

**三处缺陷**（不是 v1 说的一处）：

```cpp
igh_bus.cpp:133   ecrt_slave_config_create_sdo_request(sc_, a.index, a.sub, 4);  // size 写死 4
igh_bus.cpp:303   const int32_t v = EC_READ_S32(...);                            // 读宏写死
igh_bus.cpp:304   if (a.cfg.name == "motor_position") io->motor_position = v;    // 接收端只有一个分支
```

且 `config.cpp:180` 解析了 `AsyncSdoCfg::type`，但 `igh_bus.cpp` **从未读取它**——配置驱动是假的。

**修法**：按 `type` 分派 request size 与读宏；接收端改表驱动；`0x22A2` 以 `poll_divisor = 1000`（1 Hz）加入。**该字段采样时刻与 PDO 不同步**，在字段注释与 metadata 中注明。

### 5.6 activate 前的一次性读取

`slave.yaml` 的 `diagnostic_sdos` 段**从未被 `config.cpp` 解析**，是死配置。需补解析并在 activate 前（`guardBlocking()` 的 `PreActivate` 相位合法）读取：

`0x6075`（额定电流，**可写，必须锁定**）、`0x6076`（额定力矩，两本手册均无记载）、**`0x6093:01/02`（位置因子，全工程零出现）**、`0x607D:01/02`（软限位）、`0x1018:04`（序列号）、`0x100A`（固件版本）、`0x2381`/`0x2382`（环路增益）、`0x1C32`/`0x1C33` 同步诊断组。

> `slave.yaml:36` 引用的 `0x1003` **不在 ESI 对象字典中**（109 个对象都没有），需核实或删除。

### 5.7 换算链路的既有缺陷（P1 内一并修）

| 缺陷 | 位置 | 后果 |
|---|---|---|
| `torque_Nm` **漏了方向系数** | `scaling.cpp:42`（电流有 `dir_i`，力矩没有） | 方向取反时电流与力矩符号相反，`P = τ·ω` 算错 |
| `rated_current_mA = 6300` **写死** | `scaling.yaml:30` | 应改为总线实读并写入 metadata |
| `motor_vel_rpm = output_vel_rpm × gear_ratio` | `scaling.cpp:36` | **不是测量，是恒等式**，强制 `ω_in/i − ω_out ≡ 0`——恰恰抹掉了信号本身。改为 `0x2240` 差分（N=21 点中心回归微分器，σ ≈ 0.017 电机 rpm，带宽约 24 Hz；注意谐波 2× 特征频率在 5 rpm 下正是 20.2 Hz，窗口不能再放宽） |
| `config.cpp` **不解析** `motor/output_position_modulus`、`target_velocity_is_motor_side` | `scaling.hpp:34/38/39` | YAML 改不动。**且本机无多圈电池 ⇒ 断电重启后多圈退化为单圈**，此时 `modulus=0` 会在跨边界处静默跳 360° |
| 注释与文案错误宣称"实测增益 1.0006" | `scaling.cpp:32`、`ARCHITECTURE.md`、`gui/config_panel.py:40` | `velocity_gain_correction` 实为 1.0，链路里没有应用任何实测速度标定 |
| GUI 自行读 `scaling.yaml`，不读后端已发布的标定数 | `gui/config.py:27` vs `ipc_server.cpp:412` | 可与后端的 `--config` 不同目录 ⇒ 显示与实际不符 |

### 5.8 rad 换算全链路缺失

计划 A.1 要求 `theta_out_rad` / `omega_out_rad_s` / `omega_in_rad_s`，而工程里**没有任何 `2π/524288`**。正确系数 `1.198422e-5 rad/count`；`arcmin ↔ rad` 为 `2.908882e-4 rad/arcmin`。

---

## 6. 数据模型

### 6.1 `Sample` 扩展

定长 POD，经 seqlock 发布，**不得含 `std::string`/`vector`**。**只放原始量，派生量在导出层与 GUI 层换算**。

新增：`twist_counts`(i32)、`following_error_counts`(i32)、`torque_est_mNm`(i32)、`aux_position_raw`(i32)、`position_counts_raw`(i32)、`motor_position_sdo`(i32)、`dc_link_voltage_mV`(u32)、`warning_code`(u32)、`error_code`(u16)、`temperature_drive_C`(u16)、`torque_actual_permille`(i16)、`torque_ratio`(i16)。

合计 +40 字节，`sizeof(Sample)` 由 144 变为 **预期 184**。**以编译期 `static_assert` 实测为准。**

同步 5 处（见 R3）。现有线格式自检（GUI 连接时校验字节数与协议版本）作兜底。

> 现有 `Sample` 恰好 144 且**无尾部填充**（`offsetof` 断言 120 已验证），不存在"新字段填进空洞"的情况。

### 6.2 `data_logger` 的位置耦合必须消除

`kCols[]` 与 `COL_*` 写入序列靠位置对应且无校验（R4）。**本次新增 12 个字段前，必须先把这两处改成单一数据源**（宏表或 X-macro），否则静默写错 dataset 的概率很高。

同时补：`seq` 与 `flags` **不在 HDF5 列表里**，事后无法从文件检测丢包。

### 6.3 会话元数据

进 HDF5 attributes 与 CSV 表头/旁文件：

`sample_id`、`baseline_stage`、`test_item`、`rep`、`direction`、`amplitude_deg`、`frequency_hz`、`load_percent_Tr`、`load_torque_Nm_target`、`speed_rpm_target`、`operator`、`notes`、`mounting_phase_mark`、`mounting_orientation`、`loader_coupled`、`gain_set_id`、`phase_tol_in_deg`、`theta_out_ref_deg`、`theta_in_ref_phase_deg`、`theta_in_phase_deg`、`phase_error_in_deg`、`life_hours`、`cycle_count`

**标定与来源声明**（现状：`data_logger.cpp:188-193` 只写了 4 项，`scaling.yaml:21` 的 `gear_ratio_source` 根本没被解析）：

```
gear_ratio                = 121
gear_ratio_source         = "手册§12 n_out=n_motor/(X+1) + §2表2-1；实测 30.234（自身精度仅 0.05%，足以排除 120 不足以细分）"
motor_encoder_counts_rev  = 131072   # 2^17，手册§9.1
output_encoder_counts_rev = 524288   # 2^19，手册§9.1/§12 + 半圈法实测
encoder_accuracy_note     = "型号说明称 HM 可提供 20 位/±7角秒；表2-2 eRob80H 选装配置列 19Bit/±10角秒；待厂家澄清"
position_factor_0x6093    = <总线实读>
rated_current_0x6075_mA   = <总线实读>   # 可写对象，全试验锁定
rated_torque_0x6076_mNm   = <总线实读>   # 两本手册均无记载，来源存疑
torque_est_source         = "0x3B69 与 0x2241 同源同刻计算，驱动器手册称『估计』，全机无独立力传感元件；不得用于刚度退化判定"
temperature_scope         = "仅驱动器温度 0x22A2，非绕组非壳体；单位未经手册核实（eTuner 界面显示为 ℃）"
twist_sign_convention     = "theta_twist = theta_out − theta_in/i（与计划§4.4 TE 同号；计划§4.6 原文符号相反，本实现按修订 v2 统一）"
h_obs_regime              = "栅格步长 <值>°，线性插值；恒速段判据 |ω−ω_target|≤2%；有符号均值；先按周期后按重复"
sync_method               = "0x2241 驱动器同刻计算 / 或 主站相减，skew_us=<值>"
git_commit                = <hash>
config_sha256             = <hash>
```

---

## 7. 存储与文件格式

**HDF5 是主记录，CSV 是交付物。**

### 7.1 线 A（摆臂寿命运行）

- HDF5 全速率 1 kHz 全字段，**按 1~2 h 切片**（v1 的单文件方案不可行：`data_logger` 现在是单 group + `H5S_UNLIMITED` 一直长）
- 容量：184 B × 1 kHz ≈ 662 MB/h；节点最长间隔 50 h → **约 33 GB/段**（gzip 后约 10~14 GB）。可用盘 414 GB
- **不导出全速率 CSV**（50 h × 1 kHz = 1.8 亿行）
- **逐循环汇总 CSV**：每个正弦周期一行——循环号、`life_hours`、电流峰值/RMS/均值、`twist_counts` 峰值、驱动器温度、跟随误差、**母线电压最大值**、警告码。450 h 约 54 万行

### 7.2 线 B（节点性能测试）

CSV 按修订 v2 的**扩展模板**导出：

```
sample_<id>__life_<h>h__test_<item>__load_<pct>Tr__speed_<rpm>rpm__dir_<cw|ccw|na>__amp_<deg|na>__freq_<Hz|na>__rep_<nn>.csv
```

`test_<item>` 用受控词表：`current` / `TE` / `backlash` / `stiffness` / `p2p` / `sine`。

> **原计划模板会覆盖数据**（§4.8 的 8 个测试点塌缩成 2 个文件名），照抄即照抄一个 bug。

**列顺序** = A.1 公共字段（含三个留空列）+ 扩展列，扩展列排后，保证按 A.1 读取的下游脚本不受影响。

**元数据两个载体**：A.1 列出的字段一律作为 CSV 列逐行重复（计划的原始设计）；标定与来源声明写入同名 **`.meta.yaml`** 旁文件 + HDF5 attributes，含 `empty_columns` 说明。

**现成资产**：`tools/h5_to_csv.py`（97 行，有 `--fields`/`--start`/`--end`/`--decimate`，属性写成 `#` 注释头）质量不错，缺**流式读取**（现在全列驻留内存）与 GUI 入口。

---

## 8. 轨迹与指标

### 8.1 轨迹（按修订 v2 的口径）

| 项 | 现状 | 要改成 |
|---|---|---|
| §4.5 主标签轨迹 | `trajectory.cpp:99-108` 的三角波**端点零平滑**，等效加速度 16000 °/s²（A=10°、f=0.2 Hz），是手册推荐值的 300 倍 | **梯形速度剖面往复**：恒速段 5 rpm，**换向减速 0.3 s + 加速 0.3 s**（各自满足手册推荐）。±20° 行程减去每侧 4.5° 换向段后仍有 31° 恒速段（77.5%） |
| §2.1 摆臂正弦 | φ=0 时 t=0 速度 = **10.47 rpm 从静止在 1 ms 内达到**（手册动态制动门限的 4 倍）；φ=−90° 则位置阶跃 30° | 加**起振窗**（前 N 个周期幅值余弦升起） |
| §4.7 点到点 | `trapezoidal` 数学实现正确（t1 = 1.5 s ≥ 0.3 s），但 `(void)mode_` **无视运行模式**，acc/dec 无校验，且**无往复/序列机制** | 支持模式分派；acc/dec 加正数与 ≥0.3 s 校验；**新增往复序列原语**（A↔B 重复 N 次 + 循环计数输出）——13 节点寿命测试的核心缺件 |
| `triangle` 配置段 | `config.cpp` **不解析**（`default_type: triangle` 会以 amplitude=0 启动） | 保留三角波实现供调试，但**不用于主标签**；补解析 |
| CSV 轨迹 | `trajectory.cpp:247` 单调性用 `<` 而非 `<=` ⇒ 重复时间戳 → 除零 → NaN 下发；`:270` 硬编码 CSP | 修 |
| 默认模式 | CSV | **改 CSP**，但**必须在 §3.1 修完之后** |

> `test_trajectory.cpp:104` 断言三角波 t=0 的速度阶跃为正确行为——**这条测试把缺陷固化成了契约**，需一并修改。

### 8.2 指标（上位机能算全的部分）

按修订 v2 的 `h_obs` 计算规程实现：

- **栅格**：线性插值到公共输出角栅格，步长 **0.05~0.1°**（TE 主谐波空间周期 = 360/(2×121) = 1.488°，需 ≥15 点/周期）。禁止最近邻
- **恒速段**：由实测 ω 判定，`|ω_out − ω_target| ≤ 2%`
- **符号**：负→正为 forward；`return_error(θ) = TE_f(θ) − TE_r(θ)`
- **均值**：先对每个有效周期求**有符号均值**，再对 4 个周期求均值。另报 `mean|·|` 与噪声底 σ 作诊断
- **无效判据**：照 §4.4 那张表，加温度条款

**能算全**：`h_obs_arcmin`、`return_error_max/curve`、`cycle_repeatability`、`TE_pp_*`、`current_rms_*`、`overshoot_percent`、`rise_time_s`、`settling_time_s`、`steady_error_rad`、`peak_current_A`、`current_integral`、`tracking_error_rms`、`twist_counts_at_load_drift`、`dc_link_voltage_max_V`

**填 `NA`**：`k1/k2/k3`、`hysteresis_area`、`zero_torque_gap`、`friction_proxy`（横轴需外部转矩）

**当场红绿灯**（计划 §5.1/§5.2）：速度偏差 >±5%、3 次重复离群（用池化 σ 的绝对判据）、温度异常。**载荷判据对 0 Nm 档改为核查 `loader_coupled == false`**，不核查载荷值。

---

## 9. 实施分期

v1 的三期不够，审核发现的既有缺陷需要一个前置的止血期。

| 期 | 内容 | 门禁 |
|---|---|---|
| **P0 止血** | §3 全部：CSP 保持位置、停机路径串行化、GUI 门控+确认、`0x605A`、IPC 阻塞 send、`git init` | 修完才允许把默认模式切到 CSP |
| **P1 总线采集层** | `0x1A00` 重映射 + `0x1A18`/`0x1A19`、`Sample` 扩展、`data_logger` 位置耦合消除、异步 SDO 三处缺陷、真正读取 `0x3B69`/`0x3B6A`/`0x2241`/`0x20A0`/`0x6063`、`RawIo` 加 `motor_position_sdo`、activate 前一次性读取（含 `0x6093`）、`0x6041` bit11、§5.7 换算缺陷、rad 换算、mock 补齐 | **必须通过 §5.3 的 J1~J6 真机验证（伺服不使能）** |
| **P2 存储与格式** | 标定/来源声明 metadata、CSV 导出与扩展命名、A.1 列对齐与留空列、`.meta.yaml`、HDF5 切片、线 A 逐循环汇总、`h5_to_csv.py` 流式化 | P1 通过 |
| **P3 轨迹与指标** | 梯形往复、正弦起振窗、往复序列原语、模式分派与校验、§8.2 指标与当场红绿灯 | P2 通过 |
| **P4 无人值守** | 循环计数落盘续算、§2.6 自动停机判据引擎、systemd 看门狗链、日志轮转、慢客户端策略、告警通道 | P3 通过 |

**本轮做 P0 + P1。** P1 相对 v1 扩大了——扩出来的都是硬前置：没有 `0x6093` 角度换算的前提不成立，没有独立的 `motor_position_sdo` 字段 J3 执行不了，不先消除 `kCols[]` 位置耦合就加 12 个字段风险太高。

---

## 10. 错误处理

- 映射被拒 → 启动时明确报「从站拒绝 PDO 配置，请查 `dmesg` 的 `Failed to configure mapping of PDO 0x1A00` 并参考判据 J1/J2」
- 异步 SDO 读取失败 → 计数并**在状态中透出**（现在 `a.errors++` 后无任何地方透出），温度字段保持上次有效值并置无效标志
- `0x6041` bit11 置位 → 记录为异常事件
- **采集因磁盘满或写失败停止 → 必须上报并触发停机判据**。现状：`data_logger.cpp:339-344` 精心构造的错误串 `err` 被直接丢弃，`:362-365` 置 `active=false` 无任何日志，而 `ipc_server.cpp:628` 只在 `active` 为真时推送 ⇒ **GUI 永久冻结在"记录中"**，且 HDF5 文件未收尾
- CSV 导出失败 → 不影响 HDF5 主记录；HDF5 是权威来源，可事后重新导出

---

## 11. 测试策略

**现状**：实际是 **4 个 ctest / 39 个 `TEST()`**（README 写的 43 两头都对不上），且全部是纯函数数学。`realtime_task.cpp`(573 行)、`ipc_server.cpp`(633 行)、`data_logger.cpp`(370 行)、`mock_bus.cpp`(237 行) **结构性零覆盖**——根因在 `CMakeLists.txt:38-45` 的构建切分，`ecjc_core` 只含 6 个文件。

**且 mock 生成数据时用的就是 `cfg_.scaling`** ⇒ 所有经 mock 的标定测试是**同义反复**（`integration_test.py:180` 的 `abs(ratio − 121.0) < 1.0` 在 mock 下恒真）。

**要做的**：

1. **构建重切分**（解锁一切）：把 `mock_bus`/`realtime_task`/`ipc_server`/`data_logger` 移进 `ecjc_core`；`tests/CMakeLists.txt:6` 的硬编码列表改 glob（否则新增测试文件**静默不存在**）；加 `-DENABLE_SANITIZERS`（TSan 打 seqlock 与三处原子指针交接）
2. **新增单元测试**：`test_config.cpp`（9 条拒绝路径，现在零覆盖，正因如此三个字段漏解析至今无人察觉）；三个方向位的九宫格（`torque_Nm` 漏方向系数这个真 bug 正好落在没测的那格）；`velocity_gain_correction ≠ 1`；往返恒等；int32 饱和；`Δ = C_m − 30.25·C_o` 与 `1 count = 1.3619417e-3 arcmin` 的数值断言；**全部轨迹的速度/加速度连续性断言**（并修掉 `test_trajectory.cpp:104` 那条错误契约）
3. **mock 加故障注入**（`fault_ = true` 现在无任何赋值点 ⇒ 故障路径永远测不到）
4. **真机分级验证**：§5.3 的 J1~J6，**伺服不使能**下先跑通 J1/J2/J3/J6

---

## 12. 待办与未决

| 项 | 状态 |
|---|---|
| 《eRob CANopen and EtherCAT 用户手册》 | **缺**。需其确认 `0x22A2` 单位、`0x3B68` 警告码全表、过温阈值、`0x7121`/`0x8400`/`0x8611` |
| 输出端编码器精度 ±7 还是 ±10 角秒 | 待厂家澄清。直接决定 `h_obs` 噪声地板（0.12′ vs 0.17′） |
| `0x2241` 内部折算用 121 还是 120 | 待 J5 回归确认。**必须同拍记录 `0x2241`/`0x2240`/`0x6064` 三个原始量**，否则不可事后修正 |
| `0x20A0` 是输出侧还是电机侧 | 工程注释与 ESI 名称指向输出侧，但从未与 `0x6064` 对拍。P1 的 §5.4 解决 |
| `0x607D` 软限位配置 | 需写 Flash（`0x1010:01 = 0x65766173`），须单独确认后执行 |
| 多圈电池 | 本机无 3.6 V 电池，`0x730F` 上电必报 ⇒ 断电重启丢多圈、绝对零位参考改变。**威胁 13 节点的啮合相位可比性**。需在 metadata 记录每次开机的绝对零位，或用手册 §9.5 机械零点标定固定为 262144 |
| 壳体温度、绕组温度、输出端转矩、振动 | 外部传感器方案，不在本上位机范围。`0x2205 Analog Input` 在 ESI 中是 `PdoMapping=T`，若将来外接可经 `0x1A00` 同帧同步接入 |
| 计划 §4.4 的 `i=120`、§1.3 的 20 bit、§4.2 命名模板、§4.5 三角波与计算规程、§4.6 符号 | 已出《修订建议 v2》，待上游修订 |
