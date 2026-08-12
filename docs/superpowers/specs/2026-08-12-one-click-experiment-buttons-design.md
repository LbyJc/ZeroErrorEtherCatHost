# GUI 一键实验按钮 — 设计（P2 存储与格式对齐）

- 日期：2026-08-12
- 工程：`/home/tyy/Desktop/ethercat_joint_control/`
- 上游：《整机关节不可拆在线寿命实验方案》附录 A.1（CSV 公共字段）、§4.2（命名规范）、§5.3（交付物）
- 前置：P0（安全）、P1（总线采集层）已完成并合并 master（真机验证通过，`8dd0e91`）

## 0. 目标

给上位机 GUI 加"傻瓜式一键"实验按钮，把现在散在多个面板的操作（选模式、应用轨迹、使能、开始采集、开始运行）收成两个大按钮，让实验操作员按一下开始、按一下结束、数据自动按计划书格式落盘。

**非目标**：不做起振窗/梯形往复/循环计数续算/自动停机（那些是 P3/P4）；不做节点测试的六项固定流程自动化（P3）；不碰伺服使能与轨迹发生（半自动 = 这些用户先手动调好）。

## 1. 用户决策（brainstorm 已定，作为设计约束）

| 决策点 | 选定 |
|---|---|
| 使能与轨迹 | **半自动**：一键只同步起停"采集 + 运行"，伺服/轨迹用户先手动调好 |
| 两条线 | **两个独立大按钮**：「持续运行(线A)」「节点实验(线B)」 |
| 落盘目录 | **每次开始弹框选目录 + 填元数据**（记住上次目录作默认） |
| CSV 导出 | **结束时自动导出，仅线 B**；线 A 只留 HDF5（50h×1kHz CSV 会超 100GB） |
| 弹框字段 | **最精简**：线 A 问样机号+目录；线 B 加节点小时+测试项+重复号+载荷%+速度 |
| 开始前检查 | **拦下并提示**：伺服未使能/Fault/未选轨迹/未进 OP → 弹框说明缺哪步，不开始 |

## 2. 架构

新增一个面板 `gui/widgets/experiment_panel.py`，放在 GUI 显眼位置（采集/运行面板上方）。它是**纯编排层**：

- **不发明新 IPC 命令**——复用已有的 `record_start` / `start_run` / `stop_run` / `record_stop`
- 职责 = 前置状态检查 + 弹框收集实验元数据 + 按正确顺序编排已有命令 + 结束时触发 CSV 导出
- 不碰 RT、不碰伺服使能、不碰轨迹发生（半自动，这些由用户经既有面板操作）
- 不重复 P0 的安全逻辑——底层门控（CSP 阶跃兜底、2.5rpm 撤使能门控）已经兜着

**后端改动最小**：只需让 `record_start` 命令的 metadata 多接受一组实验字段（见 §4），并让它们进入 HDF5 的 group attributes。现有 `data_logger.cpp` 的 metadata 写入机制（`attrStr`/`attrDbl`）直接可用。

**为什么这样切**：半自动使这个功能退化为 UI 编排器，与实时/安全逻辑完全解耦。单一职责、可独立用 mock 测试。

### 2.1 文件边界

| 文件 | 责任 | 依赖 |
|---|---|---|
| `gui/widgets/experiment_panel.py`（新建） | 两个按钮的状态机、前置检查、编排 | 发 command 信号（既有机制）；读遥测状态判断 OP/使能 |
| `gui/widgets/experiment_dialog.py`（新建） | 开始弹框（线 A/线 B 两种字段集）+ 结束汇总弹框 | Qt，无业务逻辑 |
| `gui/experiment_naming.py`（新建，纯函数） | §4.2 文件名拼接、A.1 列顺序、留空列格式 | 无（纯字符串逻辑，可单元测试） |
| `backend/src/logger/data_logger.cpp`（改） | `LogMeta` 加实验字段，写进 HDF5 attrs | 既有 attr 写入 |
| `backend/src/communication/ipc_server.cpp`（改） | `record_start` 解析并透传实验字段到 LogMeta | 既有命令分发 |
| `tools/h5_to_csv.py`（改） | 支持按 A.1 列顺序 + 留空列 + `.meta.yaml` 旁文件导出 | 既有 HDF5 读取 |

## 3. 两个按钮的行为

每个按钮是"开始/结束"切换（点一下进"运行中"、按钮文字变「结束」，再点结束）。

### 3.1 点「开始」

```
1. 前置检查（读当前遥测状态快照）:
   - EtherCAT 是否 OP?（遥测 ethercat_state == OP）
   - 伺服是否 OperationEnabled?（遥测 cia402_state == OperationEnabled）
   - 运行模式是否匹配?（遥测 operation_mode 与目标模式一致，即 mode_matched）
   任一不满足 → 弹框列出缺哪步（"伺服未使能。请先 Servo Enable 并确认轨迹后再开始。"）
              → 中止，按钮保持"开始"态，不采集不运行
2. 弹出开始弹框收集元数据:
   线A: sample_id + 保存目录
   线B: sample_id + life_hours + test_item(下拉六选) + rep + load_percent_Tr + speed_rpm + 保存目录
   用户取消 → 什么都不做，按钮回"开始"态
3. 编排命令:
   record_start(dir=<选定目录>, meta=<实验元数据 + baseline_stage>)
   start_run
   两条同步发，样本从第一拍带 running 标志
4. 按钮切到"结束"态
```

`baseline_stage` 自动标记：线 A → `"continuous_run"`；线 B → `"life_node"`（若 life_hours=0 则 `"formal_0h"`）。

### 3.2 点「结束」

```
1. stop_run          （软停，P0 门控保证 <2.5rpm 才撤力矩；此处不撤使能）
2. record_stop       （HDF5 收尾、closeFile）
3. 线B: 自动导出 CSV — 调 h5_to_csv 按 §4.2 命名 + A.1 列顺序 + 写 .meta.yaml 旁文件
   线A: 跳过 CSV
4. 弹框汇总: 存储路径、样本数、(线B) CSV 路径；CSV 导出失败则报错但不影响 HDF5
5. 按钮回"开始"态
```

## 4. 数据落盘与命名

### 4.1 HDF5（两条线都写）

文件名：`<sample_id>__<line>__<timestamp>.h5`（如 `A01__nodeB__20260812_153012.h5`）。

metadata（group attributes）在现有基础上追加实验字段：
```
sample_id, baseline_stage, life_hours, test_item, rep,
load_percent_Tr, load_torque_Nm_target, speed_rpm_target, operator, notes
```
以及既有的标定来源声明（gear_ratio_source 等，P1 已实现）。

测不了的三列在 metadata 注明：
```
temperature_joint_C  = "外部传感器，未采集"
temperature_motor_C  = "总线无绕组温度，未采集"
load_torque_Nm_actual= "外部转矩传感器，未采集；0x3B69 是关节自估传递转矩，语义不同，未填此列"
```

### 4.2 CSV（仅线 B）

文件名严格按 §4.2 + 修订建议 v2 的扩展模板：
```
sample_<id>__life_<h>h__test_<item>__load_<pct>Tr__speed_<rpm>rpm__dir_<cw|ccw|na>__amp_<deg|na>__freq_<Hz|na>__rep_<nn>.csv
```
本期弹框不问方向/幅值/频率，一律填 `na`——本期定位是"半自动一键采集一段"，一段数据里可能同时含正反转（用户手动切轨迹的话），文件名层面无法用单一 dir 表达，故填 na；真实运动方向在 HDF5 的 `output_velocity_rpm` 逐样本可查。方向/幅值/频率作为文件名字段的细分，属 P3 的节点测试流程（那时一键会拆成六项、每项单一工况）。

`test_item` 受控词表：`current` / `TE` / `backlash` / `stiffness` / `p2p` / `sine`（弹框下拉，写入文件名前校验属于此集合）。

**列顺序** = A.1 公共字段（含三个留空列，写空字段不写 NA）+ 扩展列（扩展列排后）。

同名 `.meta.yaml` 旁文件：标定来源声明 + `empty_columns` 说明（列出三个留空列及原因）。

### 4.3 保存目录

弹框选择，用 QSettings 记住上次选的作默认。目录不存在则创建。

## 5. 错误处理

| 情况 | 处理 |
|---|---|
| 前置检查不通过 | 弹框列缺哪步，不开始，按钮保持"开始" |
| 弹框中途取消 | 什么都不做，按钮回"开始" |
| record_start 被后端拒绝（如磁盘不足） | 不发 start_run，弹框报错，按钮回"开始" |
| 运行中后端断连 | 按钮回"开始"态并提示；HDF5 由后端侧收尾（既有逻辑） |
| CSV 导出失败（磁盘满/h5 损坏） | 不影响已收尾的 HDF5；弹框报错并提示可事后手动 `h5_to_csv.py` |
| 结束时 stop_run 已因安全联锁自停 | record_stop 照常收尾，汇总注明 |

## 6. 测试

- **单元测试**（纯函数，`gui/experiment_naming.py`）：§4.2 文件名拼接（各种字段组合、na 填充、受控词表校验）、A.1 列顺序、留空列写空字段而非 NA、`.meta.yaml` 的 empty_columns 内容
- **后端单元测试**：`record_start` 解析实验字段 → LogMeta → HDF5 attrs（mock DataLogger 端到端写一批，h5py 读回验证字段在场）
- **mock 端到端**：GUI 面板逻辑走 mock 后端全链路——点开始→前置检查拦截（构造未使能态）→填元数据→采集+运行→结束→线B 生成 CSV + .meta.yaml；验证文件名、列、留空列
- **回归**：现有 record_start/start_run 单独使用不受影响（一键是叠加，不替换既有面板按钮）

## 7. 分期

单一实施计划可完成，无需再分。任务顺序建议：
1. `experiment_naming.py` 纯函数 + 单元测试（无依赖，先做）
2. 后端 `record_start` 扩 metadata + LogMeta + HDF5 attrs + 测试
3. `h5_to_csv.py` 扩展为 A.1 列序 + 留空列 + .meta.yaml
4. `experiment_dialog.py` 弹框
5. `experiment_panel.py` 编排面板 + 前置检查
6. mock 端到端集成测试

## 8. 待办与未决

| 项 | 状态 |
|---|---|
| 线 A 的 life_hours 循环计数 | 本期不做（P4）；线 A 的 HDF5 metadata 里 life_hours 留空或记"手动" |
| 节点测试六项固定流程、指标计算、当场红绿灯 | P3 |
| 方向/幅值/频率字段 | 本期一律 na；P3 的节点细分测试再填 |
| operator/notes/mounting_mark 等选填字段 | 本期弹框不问，metadata 留空；如需要后续加"高级"折叠面板 |
