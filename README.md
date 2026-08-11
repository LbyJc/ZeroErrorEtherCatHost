# EtherCAT Robot Joint Control Software

基于 **Linux + IgH EtherCAT Master + CiA402** 的机器人关节实时控制、在线调参、
实时曲线与长时间数据采集上位机。

目标硬件：ZeroErr eRob80H120I-BHM-18ET[V6] 一体化关节 + Intel I210 + IgH Master 1.5.4。

```
PySide6 GUI (普通用户)  ──Unix socket──▶  C++ 实时后端 (root)  ──ecrt──▶  IgH Master ──▶ 关节
   50 Hz 绘图                 遥测/命令          1 kHz SCHED_FIFO
```

架构设计详见 **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)**。

---

## 快速开始

### 无硬件试用（Mock 模式）

不需要 EtherCAT 硬件，完整界面与数据链路都能跑：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
/home/tyy/miniconda3/envs/zeroError/bin/python gui/main.py --mock
```

Mock 后端会仿真 CiA402 状态机与关节动力学，产生**与真实驱动器同尺度的原始计数**，
所以 scaling / 控制器 / Logger / IPC 全链路都被真实走过一遍。

### 真实硬件

```bash
# 1. 启动 IgH 主站（本机 systemd 单元不可用，见下方「已知坑」）
pkexec /usr/local/etc/init.d/ethercat start

# 2. 启动后端（需要 root：mlockall + SCHED_FIFO + /dev/EtherCAT0）
pkexec ./build/ecjc-backend --config config

# 3. 启动 GUI（普通用户）
/home/tyy/miniconda3/envs/zeroError/bin/python gui/main.py
```

### 安装为桌面应用

```bash
pkexec ./install.sh
# 重新登录后，应用菜单里会出现 "EtherCAT Joint Control"，双击即可
```

安装后正常实验全程不需要打开终端。卸载：`pkexec ./uninstall.sh`
（默认保留实验数据、配置与日志，`--purge` 才会删）。

---

## 典型实验流程

```
双击图标 → 【启动主站】(九步进度实时显示) → 选择模式 CSP/CSV/CST
        → 【Servo Enable】 → 设置轨迹与目标 → 【开始数据采集】
        → 【开始运行】 → 实时观察曲线 → 【停止运行】 → 【停止采集】
```

默认行为：**不自动使能、不自动运行、不自动产生力矩、不自动采集**。
`config/app.yaml` 里可开启「启动时自动启动主站」，但 Enable 与 Run 永远手动。

---

## 构建与测试

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
ctest --test-dir build --output-on-failure          # 43 个单元测试
python tests/integration_test.py                    # 34 项端到端检查（无需硬件）
```

单元测试覆盖 CiA402 状态机（含使能序列必须逐级 06→07→0F、Fault Reset 的 bit7 上升沿）、
物理量换算（用例数字取自真机实测，兼作标定回归保护）、六种轨迹、无锁环形队列并发。

---

## 目录

| 路径 | 内容 |
|---|---|
| `backend/` | C++17 实时后端：IgH 接口、CiA402、RT 任务、轨迹、控制器、IPC、HDF5 Logger、Mock |
| `gui/` | PySide6 + PyQtGraph 上位机 |
| `config/` | 8 个 yaml：app / ethercat / slave / pdo / scaling / gui / trajectory / controller |
| `system/` | systemd service、desktop entry、polkit 策略、root helper |
| `tests/` | 单元测试 + 端到端集成测试 |
| `tools/` | `h5_to_csv.py` 数据导出 |
| `docs/` | 架构设计 |

---

## 数据

长期存储用 **HDF5 列存**（一字段一 dataset，chunked + gzip），CSV 仅作导出。
每次采集写入完整 metadata：控制器与参数、周期、采样率、从站标识、
编码器分辨率、减速比、软件版本、起止时间。

1 kHz × 23 字段 ≈ 190 kB/s，压缩后 10 小时约 2–3 GB。

```bash
python tools/h5_to_csv.py data/exp_20260810_163043.h5 --list
python tools/h5_to_csv.py data/exp_*.h5 --fields elapsed_time_s,motor_velocity_rpm --decimate 10
```

---

## 已标定的硬件参数

2026-08-10 实测，已写入 `config/scaling.yaml`：

| 项 | 值 |
|---|---|
| 电机侧编码器 | 131072 counts/rev (2^17) |
| 输出侧编码器 | 524288 counts/rev (2^19)，**已用物理转角验证** |
| 减速比 | **121 : 1**（铭牌标称 120，实测 Δ0x2240/Δ0x6064 = 30.24 ≈ 121/4） |
| `0x60FF` 单位 | `0x6064` 的 counts/s，增益 1.0006 |
| 电机侧 100 rpm | `0x60FF ≈ 7207`（实测 99.89 rpm） |
| 额定电流 / 力矩 | 6300 mA / 31000 mNm |

**输出侧分辨率的验证**（2026-08-10 完成，`tests/verify_encoder.py`）：

这个数原本沿用厂商 demo 的 `ENCODER_RES`，没有独立依据，而它与电机侧是绑定的——
若输出侧实为 2^18，电机侧同步变 2^16，**所有 rpm 数值翻倍**。两侧编码器都是 32 位
累加多圈量（`0x20A0` 跑到 573956 也不回绕），拿不到回绕模数，所以只能靠物理转角实测。

关键是**不能走整圈**：524288 counts 在 2^19 下是 1 圈、2^18 下是 2 圈，记号两种情况
都回原位。改走**半圈**（262144 counts）后三种假设完全分开：

| 假设 | 记号最终位置 |
|---|---|
| 2^19（本机结论） | 约 181°，**停在对面** |
| 2^18 | 约 362°，回到原位 |
| 2^20 | 约 90° |

实测走了 263509 counts，记号停在对面 → **2^19 确认**。

---

## 已知坑（都已在代码里绕开，但值得知道）

**1. 阻塞式 SDO 只能在 activate 之前或 deactivate 之后调用。**
在主站 Active 相位调 `ecrt_master_sdo_upload()` 会让进程进入 D 状态（不可中断），
`kill -9` 无效，`/dev/EtherCAT0` 的 fd 永不释放，**只能重启机器**。
`igh_bus.cpp` 有运行期守卫，Active 相位直接拒绝而不是真的去调；
OP 期间读 SDO 一律走异步请求（`0x2240` 电机侧位置就是这么读的）。

**2. 本机 systemd 的 `ethercat.service` 必然失败。**
它读 `/usr/local/etc/ethercat.conf`，而该文件里 `MASTER0_DEVICE=""` 是空的。
实际可用的是 `/usr/local/etc/init.d/ethercat start`（读 `/etc/sysconfig/ethercat`）。
`ecjc-helper` 先试 systemd、失败回落 init.d。

**3. 上电必报 `0x730F`。**
= 负载端编码器多圈保持电池电压低于 3.05V（未装 3.6V 电池）。
Fault Reset 清不掉，需向 `0x2242` 写 1（会重置多圈计数）。
GUI 识别到这个码会直接给出这句提示和一个【重置负载端编码器】按钮。

**4. `0xA000` 要分两种情况看，别一律当成噪声。**
它是「EtherCAT 总线通信异常」（手册 7.2.27）。
**撤使能之后**发生（例如正常停止主站时 deactivate）只写进 0x1003 历史、不置 FAULT 位，可忽略；
**使能状态下**发生则是真故障，会锁存，必须 Fault Reset。
实测触发场景：`cycle_us=500` 且 CPU 未做内核级隔离时，实时线程偶发毫秒级抖动
→ 帧错过 DC 同步窗口 → 驱动器判定通信异常。

**5. 本驱动器不支持 Homing。**
`0x6502 = 0x38D`，无 hm 位。GUI 的 Homing 按钮会明确说明原因并建议改用 CSP + 梯形轨迹。

**6. 通信周期的下限是 500 µs，卡在从站而不是 PC。**
厂商手册 4.5.2 节：eRob 支持的最小通信周期为 500 µs（2 kHz），
且主站周期必须是它的整数倍。实测 250 µs 时从站在 PREOP 阶段就拒绝配置，
连 SAFEOP 都进不去。`slave.yaml` 里有 `min_cycle_us: 500`，
配置校验会在启动时直接拦下并说明原因，不会让人对着 PREOP 干瞪眼。

**7. 内核是 `PREEMPT_DYNAMIC`，且 CPU 未做内核级隔离——这是当前跑 1000 µs 而非 500 µs 的原因。**

`tools/rt-tune.sh` 能做的（governor→performance、抑制深度 C-state、网卡中断迁离实时核、
把已有进程赶出实时核）都做了，典型抖动很好：p50 ≈ 1 µs、p90 ≈ 7 µs。
但**尾部压不下去**：实测 cpu2 上仍有内核自身的活动——本地时钟中断 3450 次/秒、
RCU 软中断约 690 次/秒、调度 IPI 若干，这些**不受进程亲和性控制**；
加上 snapd 之类的服务重启后会飘回该核。结果是偶发 1.7–2.7 ms 的抖动尖峰。

在 500 µs 下这种尖峰会让帧错过 DC 同步窗口，驱动器报 `0xA000` 通信异常并锁存故障
（已在真机上复现）。1000 µs 下同样的绝对抖动只占半个周期，余量翻倍，
今天所有长时间运行都在 1000 µs 下完成且未出现该故障。

要可靠地跑 500 µs（这是从站的硬件极限），必须改内核启动参数并重启：
`isolcpus=2 nohz_full=2 rcu_nocbs=2 intel_idle.max_cstate=1 processor.max_cstate=1`，
最好再换 PREEMPT_RT 内核。届时把 `ethercat.yaml` 的 `cycle_us` 改回 500 即可，代码不用动。

⚠ `rt-tune.sh` 的效果**重启后全部失效**，需要时重跑。

---

## 扩展自研控制器

控制器在 `declareParams()` 里**自报**参数表（名字/单位/范围/默认值），
后端把这张表通过 IPC 发给 GUI，调参页面据此**动态生成**控件。

所以加一个新算法只需要：

1. 在 `backend/src/controllers/controller.cpp` 里写一个 `ControllerBase` 子类
2. 在 `makeController()` 里加一行

**GUI 一行都不用改。** `cst_custom` 已经是预留好的接入点，
参数按自适应/神经网络控制的常用命名（γ、λ、学习率、摩擦补偿）留好了。
