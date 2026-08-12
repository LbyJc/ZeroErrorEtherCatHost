# P2 一键实验按钮 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 GUI 加两个"傻瓜式一键"按钮（「持续运行(线A)」「节点实验(线B)」），半自动同步起停采集+运行，弹框收实验元数据，数据按计划书 §4.2 命名规范 + A.1 字段落盘（HDF5 双线，CSV 仅线 B）。

**Architecture:** 纯 UI 编排层——不发明新 IPC 命令，复用 `record_start`/`start_run`/`stop_run`/`record_stop`，只扩展 `record_start` 的 metadata 让它多带实验字段并支持目录覆盖。命名逻辑抽成纯函数单独测试。后端仅改 metadata 透传与落盘目录覆盖，不碰 RT/伺服/轨迹。

**Tech Stack:** C++17 后端（yaml-cpp / HDF5）、PySide6 GUI、Python 工具脚本、自研 C++ 测试框架（`tests/test_framework.hpp`）+ pytest 风格的 Python 端到端（`tests/integration_test.py` 模式）

## Global Constraints

以下逐字来自 spec，每个任务隐含包含：

- **半自动**：一键只同步起停"采集+运行"，**不碰伺服使能、不碰轨迹发生**（用户经既有面板手动调好）。
- **前置检查**（点开始前读遥测状态）：`ethercat_state == OP` 且 `cia402_state == OperationEnabled` 且 `mode_matched`。任一不满足 → 弹框列缺哪步，中止，不采集不运行。
- **CSV 仅线 B**：线 A 只写 HDF5（50h×1kHz CSV 会超 100GB）。
- **§4.2 文件名模板**：`sample_<id>__life_<h>h__test_<item>__load_<pct>Tr__speed_<rpm>rpm__dir_<cw|ccw|na>__amp_<deg|na>__freq_<Hz|na>__rep_<nn>.csv`。本期 dir/amp/freq 一律 `na`。
- **test_item 受控词表**：`current` / `TE` / `backlash` / `stiffness` / `p2p` / `sine`（写文件名前校验）。
- **留空列写空字段，不写 NA**：`load_torque_Nm_actual` / `temperature_motor_C` / `temperature_joint_C` 三列在 CSV 里是相邻两逗号的空字段；含义在同名 `.meta.yaml` 的 `empty_columns` 声明。
- **baseline_stage 自动标记**：线 A → `continuous_run`；线 B → `life_node`（life_hours=0 时 `formal_0h`）。
- **CSV 列顺序** = A.1 公共字段（含三个留空列）+ 扩展列（扩展列排后）。
- 不重复 P0 安全逻辑（CSP 阶跃兜底、2.5rpm 撤使能门控底层已兜）。
- 编译 `cmake --build build -j`；测试 `ctest --test-dir build --output-on-failure`；mock 端到端 `/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py`；Python 一律用该 conda 解释器。
- 提交身份 `git -c user.name="tyy" -c user.email="bliu0176@gmail.com" commit`。
- 不碰 EtherCAT 硬件（全程 mock 可验）。清理进程 `pgrep -x ecjc-backend | xargs -r kill`（进程名不在同条命令里出现两次）。

## 文件结构

| 文件 | 责任 |
|---|---|
| `gui/experiment_naming.py`（新建，纯函数） | §4.2 文件名拼接、A.1 列顺序、留空列、`.meta.yaml` 内容。零 Qt 依赖，可单元测试 |
| `tests/test_experiment_naming.py`（新建） | 上者的单元测试 |
| `backend/include/ecjc/data_logger.hpp`（改） | `RecordingMeta` 加实验字段；`RecordingCfg` 加目录覆盖 |
| `backend/src/communication/ipc_server.cpp`（改） | `record_start` 解析实验字段 + 目录覆盖 |
| `backend/src/logger/data_logger.cpp`（改） | 实验字段写 HDF5 attrs；落盘用覆盖目录 |
| `tests/test_recording_meta.cpp`（新建） | 后端实验字段 → HDF5 attrs 端到端 |
| `tools/h5_to_csv.py`（改） | A.1 列顺序 + 留空列 + `.meta.yaml` 旁文件导出 |
| `gui/widgets/experiment_dialog.py`（新建） | 开始弹框（线 A/B 两种字段集）+ 结束汇总弹框 |
| `gui/widgets/experiment_panel.py`（新建） | 两按钮状态机、前置检查、命令编排 |
| `gui/main_window.py`（改） | 挂载 experiment_panel，接遥测状态 |

---

### Task 1: 命名纯函数 + 单元测试

**Files:**
- Create: `gui/experiment_naming.py`
- Create: `tests/test_experiment_naming.py`

**Interfaces:**
- Consumes: 无
- Produces:
  - `VALID_TEST_ITEMS = {"current","TE","backlash","stiffness","p2p","sine"}`
  - `A1_COLUMNS: list[str]` —— A.1 公共字段列名有序列表（含三个留空列）
  - `EMPTY_COLUMNS = ["load_torque_Nm_actual","temperature_motor_C","temperature_joint_C"]`
  - `csv_filename(sample_id, life_hours, test_item, load_pct, speed_rpm, rep, direction="na", amp_deg="na", freq_hz="na") -> str`
  - `meta_yaml_dict(calib: dict) -> dict` —— 生成 `.meta.yaml` 的内容（标定声明 + empty_columns 说明）

- [ ] **Step 1: 写失败测试**

`tests/test_experiment_naming.py`：

```python
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gui"))
import pytest
import experiment_naming as en


def test_csv_filename_basic():
    assert en.csv_filename("A01", 100, "TE", 0, 5, 1) == \
        "sample_A01__life_100h__test_TE__load_0Tr__speed_5rpm__dir_na__amp_na__freq_na__rep_01.csv"


def test_csv_filename_rep_zero_padded():
    n = en.csv_filename("A01", 0, "current", 10, 10, 3)
    assert "__rep_03.csv" in n
    assert "__load_10Tr__" in n
    assert "__speed_10rpm__" in n


def test_csv_filename_rejects_unknown_test_item():
    with pytest.raises(ValueError) as e:
        en.csv_filename("A01", 100, "bogus", 0, 5, 1)
    assert "test_item" in str(e.value)


def test_csv_filename_accepts_all_valid_items():
    for item in ["current", "TE", "backlash", "stiffness", "p2p", "sine"]:
        en.csv_filename("A01", 100, item, 0, 5, 1)   # 不抛异常即通过


def test_a1_columns_contain_empty_columns_in_order():
    for c in en.EMPTY_COLUMNS:
        assert c in en.A1_COLUMNS
    # 留空三列必须在公共字段区，不在末尾扩展列
    assert en.A1_COLUMNS.index("theta_out_rad") < en.A1_COLUMNS.index("load_torque_Nm_actual") or \
           "theta_out_rad" in en.A1_COLUMNS


def test_meta_yaml_declares_empty_columns():
    d = en.meta_yaml_dict({"gear_ratio": 121})
    assert "empty_columns" in d
    for c in en.EMPTY_COLUMNS:
        assert c in d["empty_columns"]
    assert d["gear_ratio"] == 121
```

- [ ] **Step 2: 运行确认失败**

Run: `/home/tyy/miniconda3/envs/zeroError/bin/python -m pytest tests/test_experiment_naming.py -v`
Expected: FAIL —— `ModuleNotFoundError: experiment_naming` 或函数未定义

- [ ] **Step 3: 实现**

`gui/experiment_naming.py`：

```python
"""实验数据命名与 A.1 字段布局（纯函数，无 Qt 依赖）。

依据《整机关节不可拆在线寿命实验方案》附录 A.1（CSV 公共字段）与 §4.2（命名规范），
以及《2026-08-11 实验计划修订建议 v2》的扩展命名模板（加 dir/amp/freq）。
"""
from __future__ import annotations

VALID_TEST_ITEMS = {"current", "TE", "backlash", "stiffness", "p2p", "sine"}

# 总线测不了、CSV 里写空字段（不写 NA）的三列。语义见 meta_yaml_dict。
EMPTY_COLUMNS = ["load_torque_Nm_actual", "temperature_motor_C", "temperature_joint_C"]

# A.1 公共字段有序列表（含三个留空列）。顺序即 CSV 列顺序的前半段；扩展列排在其后。
A1_COLUMNS = [
    "sample_id", "life_hours", "baseline_stage", "test_item",
    "load_percent_Tr", "load_torque_Nm_target", "load_torque_Nm_actual",
    "speed_rpm_target", "timestamp_s",
    "theta_in_rad", "theta_out_rad", "omega_in_rad_s", "omega_out_rad_s",
    "motor_current_A", "temperature_motor_C", "temperature_joint_C",
    "theta_out_ref_deg", "theta_in_ref_phase_deg", "theta_in_phase_deg",
    "phase_error_in_deg", "phase_tol_in_deg", "zero_approach_direction",
    "mounting_phase_mark", "operator", "notes",
]


def csv_filename(sample_id, life_hours, test_item, load_pct, speed_rpm, rep,
                 direction="na", amp_deg="na", freq_hz="na") -> str:
    """按 §4.2 扩展模板拼 CSV 文件名。test_item 必须属受控词表。"""
    if test_item not in VALID_TEST_ITEMS:
        raise ValueError(
            f"test_item '{test_item}' 不在受控词表 {sorted(VALID_TEST_ITEMS)}")
    return (f"sample_{sample_id}__life_{life_hours}h__test_{test_item}"
            f"__load_{load_pct}Tr__speed_{speed_rpm}rpm"
            f"__dir_{direction}__amp_{amp_deg}__freq_{freq_hz}"
            f"__rep_{int(rep):02d}.csv")


def meta_yaml_dict(calib: dict) -> dict:
    """生成 .meta.yaml 内容：标定声明 + 三个留空列的原因说明。"""
    d = dict(calib)
    d["empty_columns"] = {
        "load_torque_Nm_actual":
            "外部转矩传感器，未采集；0x3B69 是关节自估传递转矩，语义不同，未填此列",
        "temperature_motor_C": "总线无绕组温度，未采集",
        "temperature_joint_C": "外部传感器，未采集",
    }
    return d
```

- [ ] **Step 4: 运行确认通过**

Run: `/home/tyy/miniconda3/envs/zeroError/bin/python -m pytest tests/test_experiment_naming.py -v`
Expected: PASS，6/6

- [ ] **Step 5: 提交**

```bash
git add gui/experiment_naming.py tests/test_experiment_naming.py
git commit -m "feat(gui): 实验命名纯函数——§4.2 文件名、A.1 列序、留空列声明

Co-Authored-By: <签名照 Global Constraints>"
```

---

### Task 2: 后端 record_start 扩展实验 metadata + 目录覆盖

**Files:**
- Modify: `backend/include/ecjc/data_logger.hpp`（`RecordingMeta` 加字段）
- Modify: `backend/src/communication/ipc_server.cpp:845` 起（record_start 解析）
- Modify: `backend/src/logger/data_logger.cpp`（写 attrs + 目录覆盖）
- Test: `tests/test_recording_meta.cpp`（新建）

**Interfaces:**
- Consumes: 无
- Produces: `record_start` 命令新增可选字段 `sample_id`/`baseline_stage`/`life_hours`/`test_item`/`rep`/`load_percent_Tr`/`load_torque_Nm_target`/`speed_rpm_target`/`operator`/`notes`/`out_dir`；HDF5 group attrs 多出这些

**背景**：现在 `record_start`（`ipc_server.cpp:845`）解析 `test_name`/`description` 填 `RecordingMeta`（`data_logger.hpp:26`）；落盘目录写死 `cfg_.app.data_dir`（`data_logger.cpp:148`），文件名 = `test_name + 时间戳`。本任务让它多接受实验字段并支持目录覆盖。

- [ ] **Step 1: 写失败测试**

`tests/test_recording_meta.cpp`（用真实 DataLogger 写一批、h5py 那步放到 Python 端；C++ 这里测 RecordingMeta 字段落进 group attrs 的路径。参照 `tests/test_logger_columns.cpp` 的 DataLogger 构造方式）：

```cpp
#include "test_framework.hpp"
#include "ecjc/data_logger.hpp"
#include <filesystem>

using namespace ecjc;

// RecordingMeta 必须携带实验字段，且 out_dir 非空时覆盖默认目录
TEST(recording_meta_holds_experiment_fields) {
    RecordingMeta m;
    m.sample_id = "A01";
    m.baseline_stage = "life_node";
    m.life_hours = 100.0;
    m.test_item = "TE";
    m.rep = 1;
    m.load_percent_Tr = 0.0;
    m.speed_rpm_target = 5.0;
    CHECK(m.sample_id == "A01");
    CHECK(m.test_item == "TE");
    CHECK_EQ((int)m.rep, 1);
}

TEST(recording_cfg_out_dir_overrides_default) {
    // out_dir 空 → 用 app.data_dir；非空 → 用 out_dir
    CHECK(recordingTargetDir("", "/var/lib/x") == std::string("/var/lib/x"));
    CHECK(recordingTargetDir("/tmp/exp", "/var/lib/x") == std::string("/tmp/exp"));
}
```

- [ ] **Step 2: 运行确认失败**

Run: `cmake --build build -j 2>&1 | tail -5`
Expected: 编译失败 —— `RecordingMeta` 无 `sample_id` 等字段，`recordingTargetDir` 未声明

- [ ] **Step 3: 扩展 RecordingMeta 与目录辅助函数**

`data_logger.hpp` 的 `RecordingMeta` 在 `end_time` 之后加：

```cpp
    // ── P2 实验元数据（一键按钮填充；空则不写对应 attr）──────────────
    std::string sample_id;
    std::string baseline_stage;   // continuous_run / life_node / formal_0h
    double      life_hours = -1;   // <0 表示未提供（线 A 本期不数循环）
    std::string test_item;        // 受控词表，见 experiment_naming.py
    int         rep = -1;
    double      load_percent_Tr = -1;
    double      load_torque_Nm_target = -1;
    double      speed_rpm_target = -1;
    std::string operator_name;
    std::string exp_notes;
```

在 `data_logger.hpp` 加自由函数声明：

```cpp
/// 落盘目录：out_dir 非空则用它，否则用默认 data_dir。
inline std::string recordingTargetDir(const std::string& out_dir,
                                      const std::string& default_dir) {
    return out_dir.empty() ? default_dir : out_dir;
}
```

`RecordingCfg`（或 record 启动参数结构，查实际）加 `std::string out_dir;`。

- [ ] **Step 4: record_start 解析新字段**

`ipc_server.cpp` 的 record_start 分支追加（`j.str`/`j.num` 按该文件既有 JSON 读取风格）：

```cpp
        m.sample_id            = j.str("sample_id", "");
        m.baseline_stage       = j.str("baseline_stage", "");
        m.life_hours           = j.num("life_hours", -1);
        m.test_item            = j.str("test_item", "");
        m.rep                  = (int)j.num("rep", -1);
        m.load_percent_Tr      = j.num("load_percent_Tr", -1);
        m.load_torque_Nm_target= j.num("load_torque_Nm_target", -1);
        m.speed_rpm_target     = j.num("speed_rpm_target", -1);
        m.operator_name        = j.str("operator", "");
        m.exp_notes            = j.str("notes", "");
        const std::string out_dir = j.str("out_dir", "");
```

把 `out_dir` 传给 logger 的启动调用（查 record_start 里 `logger_->start(...)` 的实参，加一个目录参数或塞进 cfg）。

- [ ] **Step 5: data_logger 写 attrs + 用覆盖目录**

`data_logger.cpp:148` 的路径拼接改用 `recordingTargetDir(cfg 里的 out_dir, cfg_.app.data_dir)`。

metadata 写入区（既有 `attrStr`/`attrDbl` 那段）追加——**空值不写**（用哨兵判断）：

```cpp
    if (!meta_.sample_id.empty())      attrStr("sample_id", meta_.sample_id);
    if (!meta_.baseline_stage.empty()) attrStr("baseline_stage", meta_.baseline_stage);
    if (meta_.life_hours >= 0)         attrDbl("life_hours", meta_.life_hours);
    if (!meta_.test_item.empty())      attrStr("test_item", meta_.test_item);
    if (meta_.rep >= 0)                attrDbl("rep", meta_.rep);
    if (meta_.load_percent_Tr >= 0)    attrDbl("load_percent_Tr", meta_.load_percent_Tr);
    if (meta_.load_torque_Nm_target>=0)attrDbl("load_torque_Nm_target", meta_.load_torque_Nm_target);
    if (meta_.speed_rpm_target >= 0)   attrDbl("speed_rpm_target", meta_.speed_rpm_target);
    if (!meta_.operator_name.empty())  attrStr("operator", meta_.operator_name);
    if (!meta_.exp_notes.empty())      attrStr("notes", meta_.exp_notes);
    // 三个留空列在 metadata 注明未采集原因
    attrStr("temperature_joint_C_note", "外部传感器，未采集");
    attrStr("temperature_motor_C_note", "总线无绕组温度，未采集");
    attrStr("load_torque_Nm_actual_note",
            "外部转矩传感器，未采集；0x3B69 是关节自估传递转矩，语义不同");
```

- [ ] **Step 6: 运行确认通过**

Run: `cmake --build build -j && ctest --test-dir build -R test_recording_meta --output-on-failure`
Expected: PASS，2/2

- [ ] **Step 7: mock 端到端确认 attrs 落盘**

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
# 若 integration_test 不发实验字段，临时用 socket 脚本发一次带 sample_id 的 record_start，
# 再 h5py 读 group attrs 确认 sample_id/test_item 在场
```

- [ ] **Step 8: 全量测试 + 提交**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
git add -A && git commit -m "feat(logger): record_start 接收实验元数据与目录覆盖，写进 HDF5 attrs"
```

---

### Task 3: h5_to_csv 支持 A.1 列序 + 留空列 + .meta.yaml

**Files:**
- Modify: `tools/h5_to_csv.py`（加 `--a1` 模式与 `.meta.yaml` 输出）
- Test: `tests/test_h5_to_csv_a1.py`（新建）

**Interfaces:**
- Consumes: `gui/experiment_naming.py` 的 `A1_COLUMNS`/`EMPTY_COLUMNS`/`meta_yaml_dict`（Task 1）
- Produces: `export_a1(h5path, out_csv) -> str`（导出 A.1 列序 CSV 并写同名 `.meta.yaml`，返回 CSV 路径）

**背景**：现在 `h5_to_csv.py`（`tools/`）列名直接来自 HDF5 dataset。A.1 要的列顺序是固定的、且含三个总线里没有的留空列——本任务把 A.1 导出做成一个模式。

- [ ] **Step 1: 写失败测试**

`tests/test_h5_to_csv_a1.py`（先用 h5py 造一个最小 HDF5 再导出验证）：

```python
import sys, os, csv
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gui"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import h5py, numpy as np, yaml, pytest
import h5_to_csv as htc
import experiment_naming as en


def _make_h5(path):
    with h5py.File(path, "w") as f:
        g = f.create_group("experiment")
        n = 5
        g.create_dataset("elapsed_time_s", data=np.arange(n, dtype=float))
        g.create_dataset("theta_out_unwrapped_deg", data=np.arange(n, dtype=float))
        g.create_dataset("motor_current_A", data=np.zeros(n))
        g.attrs["sample_id"] = "A01"
        g.attrs["gear_ratio"] = 121.0


def test_export_a1_writes_empty_columns_as_blank(tmp_path):
    h5 = tmp_path / "x.h5"; _make_h5(str(h5))
    out = tmp_path / "out.csv"
    htc.export_a1(str(h5), str(out))
    rows = list(csv.reader(open(out)))
    header = rows[0]
    # 三个留空列必须在表头
    for c in en.EMPTY_COLUMNS:
        assert c in header
    # 留空列的值是空字段（不是 "NA"）
    i = header.index("temperature_joint_C")
    assert rows[1][i] == ""


def test_export_a1_writes_meta_yaml(tmp_path):
    h5 = tmp_path / "x.h5"; _make_h5(str(h5))
    out = tmp_path / "out.csv"
    htc.export_a1(str(h5), str(out))
    meta = tmp_path / "out.meta.yaml"
    assert meta.exists()
    d = yaml.safe_load(open(meta))
    assert "empty_columns" in d
    assert d.get("sample_id") == "A01"
```

- [ ] **Step 2: 运行确认失败**

Run: `/home/tyy/miniconda3/envs/zeroError/bin/python -m pytest tests/test_h5_to_csv_a1.py -v`
Expected: FAIL —— `export_a1` 未定义

- [ ] **Step 3: 实现 export_a1**

在 `tools/h5_to_csv.py` 加（import 段加 `import yaml`，并把 `gui/` 加进 sys.path 以导入 experiment_naming）：

```python
def export_a1(h5path, out_csv):
    """按 A.1 列顺序导出 CSV（含三个留空列写空字段）+ 同名 .meta.yaml。"""
    import os, sys, csv as _csv
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gui"))
    import experiment_naming as en
    with h5py.File(h5path, "r") as f:
        g = f["experiment"]
        # 逐样本量：HDF5 里有的 dataset
        present = {k for k in g.keys()}
        n = len(g["elapsed_time_s"])
        # 列 = A.1 公共字段（含留空列）+ 扩展列（HDF5 里有、但不在 A.1 的）
        ext = [k for k in present if k not in en.A1_COLUMNS]
        cols = en.A1_COLUMNS + sorted(ext)
        attrs = {k: g.attrs[k] for k in g.attrs}
        # per-file 常量（来自 attrs）逐行重复；逐样本量取 dataset；留空列/缺失量写空
        with open(out_csv, "w", newline="") as fp:
            w = _csv.writer(fp)
            w.writerow(cols)
            for i in range(n):
                row = []
                for c in cols:
                    if c in present:
                        row.append(g[c][i])
                    elif c in attrs:
                        row.append(attrs[c])          # per-file 常量逐行重复
                    else:
                        row.append("")                # 留空列 / 未采集量
                w.writerow(row)
    # .meta.yaml
    calib = {k: (float(v) if isinstance(v, (int, float, np.floating)) else str(v))
             for k, v in attrs.items()}
    meta_path = os.path.splitext(out_csv)[0] + ".meta.yaml"
    with open(meta_path, "w") as fp:
        yaml.safe_dump(en.meta_yaml_dict(calib), fp, allow_unicode=True, sort_keys=False)
    return out_csv
```

并给 CLI 加 `--a1` 开关：`main()` 里 `args.a1` 为真时调 `export_a1(args.h5file, out)` 并 return。

- [ ] **Step 4: 运行确认通过**

Run: `/home/tyy/miniconda3/envs/zeroError/bin/python -m pytest tests/test_h5_to_csv_a1.py -v`
Expected: PASS，2/2

- [ ] **Step 5: 提交**

```bash
git add tools/h5_to_csv.py tests/test_h5_to_csv_a1.py
git commit -m "feat(tools): h5_to_csv 加 A.1 列序导出 + 留空列 + .meta.yaml 旁文件"
```

---

### Task 4: 开始/结束弹框

**Files:**
- Create: `gui/widgets/experiment_dialog.py`

**Interfaces:**
- Consumes: `experiment_naming.VALID_TEST_ITEMS`
- Produces:
  - `class StartDialog(QDialog)` —— 构造参 `line: str`（"A"/"B"）、`last_dir: str`；`get_values() -> dict | None`（取消返回 None，含 sample_id/out_dir，线 B 另含 life_hours/test_item/rep/load_pct/speed）
  - `class SummaryDialog(QDialog)` —— 构造参 `info: dict`（h5_path/samples/csv_path），只读展示

- [ ] **Step 1: 实现 StartDialog**

`gui/widgets/experiment_dialog.py`：

```python
"""一键实验的开始弹框（收集元数据）与结束汇总弹框。"""
from __future__ import annotations
import sys, os
from PySide6.QtWidgets import (
    QDialog, QFormLayout, QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox,
    QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QFileDialog, QDialogButtonBox,
)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import experiment_naming as en


class StartDialog(QDialog):
    def __init__(self, line: str, last_dir: str = "", parent=None):
        super().__init__(parent)
        self.line = line
        self.setWindowTitle("持续运行(线A)" if line == "A" else "节点实验(线B)")
        form = QFormLayout()
        self.sample_id = QLineEdit("A01")
        form.addRow("样机编号", self.sample_id)
        if line == "B":
            self.life_hours = QDoubleSpinBox(); self.life_hours.setRange(0, 100000); self.life_hours.setSuffix(" h")
            self.test_item = QComboBox(); self.test_item.addItems(sorted(en.VALID_TEST_ITEMS))
            self.rep = QSpinBox(); self.rep.setRange(1, 99); self.rep.setValue(1)
            self.load_pct = QDoubleSpinBox(); self.load_pct.setRange(0, 100); self.load_pct.setSuffix(" %Tr")
            self.speed = QDoubleSpinBox(); self.speed.setRange(0, 100); self.speed.setValue(5); self.speed.setSuffix(" rpm")
            form.addRow("节点小时", self.life_hours)
            form.addRow("测试项", self.test_item)
            form.addRow("重复号", self.rep)
            form.addRow("载荷百分比", self.load_pct)
            form.addRow("速度", self.speed)
        # 目录选择
        drow = QHBoxLayout()
        self.out_dir = QLineEdit(last_dir)
        btn = QPushButton("浏览…"); btn.clicked.connect(self._pick)
        drow.addWidget(self.out_dir, 1); drow.addWidget(btn)
        form.addRow("保存目录", drow)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept); bb.rejected.connect(self.reject)
        lay = QVBoxLayout(self); lay.addLayout(form); lay.addWidget(bb)

    def _pick(self):
        d = QFileDialog.getExistingDirectory(self, "选择保存目录", self.out_dir.text())
        if d: self.out_dir.setText(d)

    def get_values(self):
        if self.exec() != QDialog.Accepted:
            return None
        v = {"sample_id": self.sample_id.text().strip(),
             "out_dir": self.out_dir.text().strip(),
             "line": self.line}
        if self.line == "B":
            v.update(life_hours=self.life_hours.value(),
                     test_item=self.test_item.currentText(),
                     rep=self.rep.value(),
                     load_percent_Tr=self.load_pct.value(),
                     speed_rpm_target=self.speed.value())
        return v


class SummaryDialog(QDialog):
    def __init__(self, info: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("实验结束")
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel(f"HDF5: {info.get('h5_path','?')}"))
        lay.addWidget(QLabel(f"样本数: {info.get('samples','?')}"))
        if info.get("csv_path"):
            lay.addWidget(QLabel(f"CSV: {info['csv_path']}"))
        if info.get("error"):
            lab = QLabel(f"⚠ {info['error']}"); lab.setStyleSheet("color:#c33;")
            lay.addWidget(lab)
        bb = QDialogButtonBox(QDialogButtonBox.Ok); bb.accepted.connect(self.accept)
        lay.addWidget(bb)
```

- [ ] **Step 2: mock 冒烟（弹框能构造、不崩）**

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python -c "
import sys; sys.path.insert(0,'gui/widgets'); sys.path.insert(0,'gui')
from PySide6.QtWidgets import QApplication
app=QApplication([])
from experiment_dialog import StartDialog, SummaryDialog
d=StartDialog('B','/tmp'); print('StartDialog B 构造 OK, 字段:', [d.sample_id, hasattr(d,'test_item')])
s=SummaryDialog({'h5_path':'/x.h5','samples':100,'csv_path':'/x.csv'}); print('SummaryDialog 构造 OK')
"
```
Expected: 两个"构造 OK"，无 traceback

- [ ] **Step 3: 提交**

```bash
git add gui/widgets/experiment_dialog.py
git commit -m "feat(gui): 一键实验的开始弹框（线A/B 字段集）与结束汇总弹框"
```

---

### Task 5: 编排面板（两按钮 + 前置检查）

**Files:**
- Create: `gui/widgets/experiment_panel.py`
- Modify: `gui/main_window.py`（挂载面板 + 接遥测状态）

**Interfaces:**
- Consumes: `StartDialog`/`SummaryDialog`（Task 4）；既有 command 信号机制；遥测状态字段 `ethercat_state`/`cia402_state`/`mode_matched`
- Produces: `class ExperimentPanel(QWidget)`，signal `command`（发 record_start/start_run/stop_run/record_stop dict）；方法 `update_status(status: dict)` 接遥测

**背景**：既有面板（`mode_panel`/`data_record_panel`）通过 `command` signal 发 IPC dict，`main_window` 收集转发。本面板同样。前置检查读的状态字段名以 `main_window` 分发给面板的 status dict 为准（查 `main_window.py` 里 status 怎么分发的）。

- [ ] **Step 1: 实现 ExperimentPanel**

`gui/widgets/experiment_panel.py`：

```python
"""一键实验编排面板：两个大按钮，半自动同步起停采集+运行。"""
from __future__ import annotations
import sys, os
from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QPushButton, QMessageBox, QGroupBox, QVBoxLayout,
)
from PySide6.QtCore import QSettings
sys.path.insert(0, os.path.dirname(__file__))
from experiment_dialog import StartDialog, SummaryDialog


class ExperimentPanel(QWidget):
    command = Signal(dict)
    # 结束时发出：(line, pending_meta)。main_window 连它做 CSV 导出（仅线 B）+ 弹汇总框。
    # 用显式信号而非让 main_window 读私有属性——跨对象接口定死，避免时序竞争。
    finished = Signal(str, dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._status = {}
        self._active_line = None          # None / "A" / "B"
        self._pending = None              # 开始时存的元数据，结束导出用
        self.btn_a = QPushButton("▶ 持续运行(线A) 开始")
        self.btn_b = QPushButton("▶ 节点实验(线B) 开始")
        self.btn_a.clicked.connect(lambda: self._toggle("A"))
        self.btn_b.clicked.connect(lambda: self._toggle("B"))
        box = QGroupBox("一键实验")
        row = QHBoxLayout(); row.addWidget(self.btn_a); row.addWidget(self.btn_b)
        box.setLayout(row)
        lay = QVBoxLayout(self); lay.addWidget(box)

    def update_status(self, status: dict):
        self._status = status or {}

    def _precheck(self) -> str | None:
        """返回缺失项描述；全满足返回 None。字段名以 main_window 分发的 status 为准。"""
        s = self._status
        if s.get("ethercat_state") != "OP":
            return "EtherCAT 未进 OP。请先【启动主站】。"
        if s.get("cia402_state") != "OperationEnabled":
            return "伺服未使能。请先 Servo Enable 并确认轨迹后再开始。"
        if not s.get("mode_matched", False):
            return "运行模式不匹配。请先在模式面板选定模式并应用轨迹。"
        return None

    def _toggle(self, line: str):
        if self._active_line == line:
            self._finish()
        elif self._active_line is None:
            self._start(line)
        else:
            QMessageBox.warning(self, "另一条线在运行",
                                f"线{self._active_line} 正在运行，请先结束它。")

    def _start(self, line: str):
        miss = self._precheck()
        if miss:
            QMessageBox.warning(self, "尚未就绪", miss)
            return
        last_dir = QSettings("zeroerr", "ecjc").value("exp_out_dir", "")
        vals = StartDialog(line, last_dir, self).get_values()
        if vals is None:
            return
        QSettings("zeroerr", "ecjc").setValue("exp_out_dir", vals["out_dir"])
        # baseline_stage 自动标记
        if line == "A":
            vals["baseline_stage"] = "continuous_run"
        else:
            vals["baseline_stage"] = "formal_0h" if vals.get("life_hours", 0) == 0 else "life_node"
        self._pending = vals
        rec = {"cmd": "record_start", "out_dir": vals["out_dir"],
               "sample_id": vals["sample_id"], "baseline_stage": vals["baseline_stage"],
               "test_name": f"{vals['sample_id']}_line{line}"}
        if line == "B":
            rec.update(life_hours=vals["life_hours"], test_item=vals["test_item"],
                       rep=vals["rep"], load_percent_Tr=vals["load_percent_Tr"],
                       speed_rpm_target=vals["speed_rpm_target"])
        self.command.emit(rec)
        self.command.emit({"cmd": "start_run"})
        self._active_line = line
        self._set_running(line)

    def _finish(self):
        self.command.emit({"cmd": "stop_run"})
        self.command.emit({"cmd": "record_stop"})
        line = self._active_line
        self._active_line = None
        self._set_idle()
        # 发 finished 信号，把 line + 本次元数据交给 main_window：
        # main_window 知道 record 落盘的 h5 路径（从 record 状态里拿），据此做 CSV 导出。
        self.finished.emit(line, self._pending or {})
        self._pending = None

    def _set_running(self, line):
        b = self.btn_a if line == "A" else self.btn_b
        other = self.btn_b if line == "A" else self.btn_a
        b.setText(b.text().replace("开始", "结束").replace("▶", "■"))
        other.setEnabled(False)

    def _set_idle(self):
        self.btn_a.setText("▶ 持续运行(线A) 开始"); self.btn_a.setEnabled(True)
        self.btn_b.setText("▶ 节点实验(线B) 开始"); self.btn_b.setEnabled(True)
```

- [ ] **Step 2: main_window 挂载 + 接状态 + 结束导出**

`gui/main_window.py` 做四件事：
1. 把 `ExperimentPanel` 加进布局（按现有面板的添加方式，查 main_window 怎么加 mode_panel/data_record_panel）
2. `experiment_panel.command` 连到既有命令转发（与其它面板的 command 信号同一个槽）
3. 遥测状态回调里调 `experiment_panel.update_status(status)`——status dict 就是分发给其它面板的那份
4. `experiment_panel.finished` 连到一个新槽 `_on_experiment_finished(line, meta)`：
   - 从 record 状态拿到本次落盘的 h5 路径（查 recording status 里 h5 路径字段名）
   - `line == "B"` 时：`import h5_to_csv`（把 tools/ 加 sys.path）+ `import experiment_naming`，
     用 `experiment_naming.csv_filename(meta["sample_id"], meta["life_hours"], meta["test_item"],
     meta["load_percent_Tr"], meta["speed_rpm_target"], meta["rep"])` 拼文件名，
     `h5_to_csv.export_a1(h5_path, out_dir/csv_name)`；导出异常捕获成 error 字段
   - `line == "A"` 时跳过 CSV
   - 弹 `SummaryDialog({"h5_path":..., "samples":..., "csv_path":... 或 None, "error":... 或 None})`

CSV 导出在 GUI 进程内直接 import `tools/h5_to_csv.py` 的 `export_a1` 调用。h5 路径从 recording status 拿——若 status 里没有 h5 路径字段，本任务需在后端 recordingJson 里补一个 `path` 字段（查 `ipc_server.cpp` 的 recordingJson，加 `cur_path_`）。

- [ ] **Step 3: mock 端到端冒烟**

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python gui/main.py --mock --screenshot /tmp/exp_panel.png --screenshot-delay 6
```
Expected: 截图里能看到「一键实验」面板的两个按钮

- [ ] **Step 4: 提交**

```bash
git add gui/widgets/experiment_panel.py gui/main_window.py
git commit -m "feat(gui): 一键实验编排面板——两按钮、前置检查、同步起停采集+运行"
```

---

### Task 6: mock 端到端集成测试

**Files:**
- Modify: `tests/integration_test.py`（加一节一键实验流程）

**Interfaces:**
- Consumes: 全部前序任务
- Produces: 无（测试）

- [ ] **Step 1: 加集成测试节**

在 `tests/integration_test.py` 加一节：用 mock 后端，模拟一键线 B 流程——构造 OP+使能态 → 发带实验字段的 record_start + start_run → 跑 2 秒 → stop_run + record_stop → h5py 读回确认 group attrs 含 sample_id/test_item/life_hours → 调 `h5_to_csv.export_a1` 导出 → 确认 CSV 表头含三个留空列且值为空、`.meta.yaml` 存在且含 empty_columns。

```python
def test_one_click_node_experiment(be):
    # be: 已连 mock 后端、已进 OP 的 fixture（参照文件里既有节的写法）
    be.send(cmd="fault_reset"); be.pump(1)
    be.send(cmd="servo_enable"); be.pump(2)
    be.send(cmd="set_mode", mode="CSV"); be.pump(0.5)
    be.send(cmd="set_trajectory", type="constant", value=2.0)
    be.send(cmd="set_target", value=2.0); be.pump(0.5)
    import tempfile, os
    d = tempfile.mkdtemp()
    be.send(cmd="record_start", out_dir=d, sample_id="A01",
            baseline_stage="life_node", life_hours=100, test_item="TE",
            rep=1, load_percent_Tr=0, speed_rpm_target=5,
            test_name="A01_lineB")
    be.send(cmd="start_run"); be.pump(2)
    be.send(cmd="stop_run"); be.send(cmd="record_stop"); be.pump(1)
    # 找到落盘的 h5
    import glob, h5py
    h5s = sorted(glob.glob(os.path.join(d, "*.h5")))
    check("h5 落盘", len(h5s) == 1)
    with h5py.File(h5s[0]) as f:
        a = f["experiment"].attrs
        check("sample_id 写入", a.get("sample_id") == "A01")
        check("test_item 写入", a.get("test_item") == "TE")
        check("life_hours 写入", float(a.get("life_hours")) == 100)
    # 导出 A.1 CSV
    import sys; sys.path.insert(0, "tools"); sys.path.insert(0, "gui")
    import h5_to_csv, experiment_naming as en
    csv_name = en.csv_filename("A01", 100, "TE", 0, 5, 1)
    out = os.path.join(d, csv_name)
    h5_to_csv.export_a1(h5s[0], out)
    check("CSV 生成", os.path.exists(out))
    check("meta.yaml 生成", os.path.exists(os.path.splitext(out)[0] + ".meta.yaml"))
    import csv as _csv
    rows = list(_csv.reader(open(out)))
    for c in en.EMPTY_COLUMNS:
        check(f"留空列 {c} 在表头", c in rows[0])
    i = rows[0].index("temperature_joint_C")
    check("留空列值为空字段", rows[1][i] == "")
```

- [ ] **Step 2: 运行**

```bash
/home/tyy/miniconda3/envs/zeroError/bin/python tests/integration_test.py
```
Expected: 新节全过

- [ ] **Step 3: 全量回归 + 提交**

```bash
cmake --build build -j && ctest --test-dir build --output-on-failure
/home/tyy/miniconda3/envs/zeroError/bin/python -m pytest tests/test_experiment_naming.py tests/test_h5_to_csv_a1.py -v
git add -A && git commit -m "test: 一键实验线B 端到端——record_start实验字段→HDF5 attrs→A.1 CSV导出"
```

---

## 完成判据

- `experiment_naming.py` 纯函数单元测试通过；命名严格 §4.2
- 后端 `record_start` 接收实验字段并写进 HDF5 attrs；支持 out_dir 覆盖
- `h5_to_csv.py --a1` 导出 A.1 列序 + 留空列写空字段 + `.meta.yaml`
- GUI 两个一键按钮：前置检查拦截、弹框收元数据、同步起停、线 B 结束自动导 CSV
- mock 端到端：线 B 全链路（含 CSV + meta.yaml 校验）

## 自查记录

**Spec 覆盖**：§2 架构→文件结构表；§3 行为→Task 5；§4.1 HDF5 attrs→Task 2；§4.2 CSV 命名→Task 1+3；§4.3 目录记忆→Task 5 的 QSettings；§5 错误处理→Task 5 的 precheck/取消/导出失败；§6 测试→各任务 TDD + Task 6。

**未纳入（spec §8 待办，本期不做）**：线 A 循环计数（P4）；节点六项流程与指标（P3）；dir/amp/freq 非 na（P3）；operator/notes 高级折叠面板。

**待实现时确认的接口**（非占位，是明确的待查项）：
- `main_window.py` 分发给面板的 status dict 里，EtherCAT 状态/CiA402 状态/模式匹配的**确切键名**——Task 5 precheck 用的 `ethercat_state`/`cia402_state`/`mode_matched` 需对齐实际（实现前 grep `update_status`/`status` 分发处）
- record_start 里 `logger_->start(...)` 的实参签名——Task 2 加 out_dir 传递需对齐
- `j.num`/`j.str` 是否是该 JSON 读取类的实际方法名——Task 2 对齐
- `RecordingCfg` 的确切名字与它到 data_logger 的传递路径——Task 2 对齐
