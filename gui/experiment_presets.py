"""实验配置（preset）加载：一键实验面板的"配置列表"数据源。

每个配置是 experiments/presets/ 下的一个 YAML 文件，字段：

    name: 周期测试1_电流摩擦_空载        # 必填。数据文件名与它同步：
                                          # record test_name = <样机号>_<name>
    kind: node | continuous               # 必填。node → 线B，continuous → 线A
    description: 给测试员看的操作提示      # 必填（加载装置怎么设等）
    mode: CSV | CSP | CST                 # 必填
    trajectory:                           # 必填。字段名与 IPC set_trajectory 一致
      type: csv | sine | constant | ...   # csv_path 相对路径按 presets 文件所在
      csv_path: ../工况/xxx.csv           # 目录解析成绝对路径
    record:                               # 线B 的 record_start 预填字段，线A 可省
      test_item: current                  # 必须在 experiment_naming 受控词表内
      load_percent_Tr: 0
      speed_rpm_target: 5
    merged_from:                          # 可选。被本配置合并掉的旧配置名列表
      - 周期测试1_电流摩擦_xxx           # （轨迹只差时长，一次运行两用），
                                          # 只影响列表显示名（含 …），不进文件名

文件名以数字前缀排序（10_xxx.yaml, 21_xxx.yaml…），列表按此顺序显示。
纯逻辑、不依赖 Qt，单测在 tests/test_experiment_presets.py。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

import experiment_naming as en


@dataclass
class Preset:
    name: str
    kind: str                      # "node" / "continuous"
    description: str
    mode: str
    trajectory: dict
    record: dict = field(default_factory=dict)
    merged_from: list = field(default_factory=list)
    path: Path | None = None

    @property
    def line(self) -> str:
        return "A" if self.kind == "continuous" else "B"

    @property
    def display_name(self) -> str:
        """列表/提示里用的名字：被合并配置的名字放进括号（2026-08-14 用户需求）。
        数据文件名（record test_name）仍用短的 name，不带括号。"""
        if not self.merged_from:
            return self.name
        return f"{self.name}（含 {'、'.join(self.merged_from)}）"


class PresetError(ValueError):
    pass


_REQUIRED = ("name", "kind", "mode", "trajectory", "description")
_KINDS = {"node", "continuous"}
_MODES = {"CSP", "CSV", "CST", "PP", "PV", "PT"}


def load_preset(path: Path) -> Preset:
    with open(path, encoding="utf-8") as f:
        d = yaml.safe_load(f)
    if not isinstance(d, dict):
        raise PresetError(f"{path.name}: 不是 YAML 映射")
    for k in _REQUIRED:
        if k not in d:
            raise PresetError(f"{path.name}: 缺少必填字段 {k}")
    if d["kind"] not in _KINDS:
        raise PresetError(f"{path.name}: kind 必须是 {sorted(_KINDS)}，得到 {d['kind']!r}")
    if d["mode"] not in _MODES:
        raise PresetError(f"{path.name}: mode 必须是 {sorted(_MODES)}，得到 {d['mode']!r}")
    traj = dict(d["trajectory"] or {})
    if "type" not in traj:
        raise PresetError(f"{path.name}: trajectory 缺少 type")
    if traj["type"] == "csv":
        p = Path(str(traj.get("csv_path", "")))
        if not str(p):
            raise PresetError(f"{path.name}: csv 轨迹缺少 csv_path")
        if not p.is_absolute():
            p = (path.parent / p).resolve()
        if not p.is_file():
            raise PresetError(f"{path.name}: 轨迹文件不存在: {p}")
        traj["csv_path"] = str(p)
    rec = dict(d.get("record") or {})
    ti = rec.get("test_item")
    if ti is not None and ti not in en.VALID_TEST_ITEMS:
        raise PresetError(f"{path.name}: test_item '{ti}' 不在受控词表 "
                          f"{sorted(en.VALID_TEST_ITEMS)}")
    merged = d.get("merged_from") or []
    if not isinstance(merged, list) or not all(isinstance(m, str) for m in merged):
        raise PresetError(f"{path.name}: merged_from 必须是字符串列表")
    return Preset(name=str(d["name"]), kind=d["kind"], description=str(d["description"]),
                  mode=d["mode"], trajectory=traj, record=rec,
                  merged_from=[str(m) for m in merged], path=path)


def scan_presets(presets_dir: Path) -> tuple[list[Preset], list[str]]:
    """扫描目录，按文件名排序。返回 (成功列表, 错误信息列表)——
    单个文件坏了不拖垮整个列表，错误显示在面板上让人能修。"""
    presets, errors = [], []
    if not presets_dir.is_dir():
        return presets, [f"配置目录不存在: {presets_dir}"]
    for p in sorted(presets_dir.glob("*.yaml")):
        try:
            presets.append(load_preset(p))
        except Exception as e:      # noqa: BLE001 —— 坏文件只报错不崩
            errors.append(str(e))
    return presets, errors


def trajectory_command(pre: Preset) -> dict:
    """set_trajectory 的 IPC 消息体。"""
    msg = {"cmd": "set_trajectory", "type": pre.trajectory["type"]}
    for k, v in pre.trajectory.items():
        if k != "type":
            msg[k] = v
    return msg
