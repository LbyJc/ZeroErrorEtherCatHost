"""一键实验编排面板：配置列表 + 两个大按钮，半自动同步起停采集+运行。

前置检查读的状态字段名以 main_window._on_status 分发给各面板的 status dict 为准
（实测：backend IpcServer::statusJson 里 EtherCAT 状态字段是 "ethercat"，不是
"ethercat_state"；伺服状态字段是 "servo"，值是 cia402.cpp::toString(Cia402State)
的输出，OperationEnabled -> "Operation Enabled"）。

配置列表（2026-08-13 现场需求）：experiments/presets/*.yaml 是预定义实验配置
（持续运行 + 节点实验 1..N）。测试员选中 →【加载所选配置】→ 面板代发
set_mode / set_trajectory，并把 record 字段预填进开始弹框；数据文件名与配置名
同步（record test_name = <样机号>_<配置名>）。加载配置只写模式与轨迹参数，
不使能、不运行——运行仍走原有两个大按钮。
"""
from __future__ import annotations

import os
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal, QSettings
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QMessageBox,
    QPushButton, QGroupBox, QVBoxLayout,
)

import experiment_naming as en
import experiment_presets as ep
from .common import COLOR_ERR, COLOR_OK
from .experiment_dialog import StartDialog


class ExperimentPanel(QWidget):
    command = Signal(dict)
    # 结束时发出：(line, pending_meta)。main_window 连它做 CSV 导出（仅线 B）+ 弹汇总框。
    # 用显式信号而非让 main_window 读私有属性——跨对象接口定死，避免时序竞争。
    finished = Signal(str, dict)

    _NAMES = {"A": "持续运行(线A)", "B": "节点实验(线B)"}

    def __init__(self, cfg=None, parent=None):
        super().__init__(parent)
        self._status = {}
        self._active_line = None          # None / "A" / "B"（record_start 已 ack、在跑）
        self._awaiting_line = None        # None / "A" / "B"（record_start 已发，等 ack）
        self._pending = None              # 开始时存的元数据，结束导出用
        self._loaded_preset = None        # 当前已加载的 Preset（None = 手动模式）
        self._b_was_running = False       # 线B 自动收尾：见过 running=True 才算跑起来过
        # 节点实验轨迹播完自动软停后，等减速斜坡走完再停采集（5rpm@1.65rpm/s≈3s，
        # 取 5s 裕量）。测试可把它设成 0 立即收尾。
        self._auto_finish_delay_ms = 5000

        self.btn_a = QPushButton(self._label("A", "start"))
        self.btn_b = QPushButton(self._label("B", "start"))
        self.btn_a.clicked.connect(lambda: self._toggle("A"))
        self.btn_b.clicked.connect(lambda: self._toggle("B"))

        # ── 配置列表 ────────────────────────────────────────────────
        pbox = QGroupBox("实验配置")
        pv = QVBoxLayout(pbox)
        self.preset_list = QListWidget()
        self.preset_list.currentItemChanged.connect(self._on_preset_selected)
        pv.addWidget(self.preset_list)
        self.preset_desc = QLabel("选择一个配置查看操作提示")
        self.preset_desc.setWordWrap(True)
        self.preset_desc.setStyleSheet("color:#666; font-size:11px;")
        pv.addWidget(self.preset_desc)
        self.btn_load = QPushButton("加载所选配置")
        self.btn_load.clicked.connect(self._load_selected)
        pv.addWidget(self.btn_load)
        self.loaded_label = QLabel("当前配置：（未加载，手动模式）")
        self.loaded_label.setWordWrap(True)
        pv.addWidget(self.loaded_label)

        box = QGroupBox("一键实验")
        row = QHBoxLayout()
        row.addWidget(self.btn_a)
        row.addWidget(self.btn_b)
        box.setLayout(row)

        lay = QVBoxLayout(self)
        lay.addWidget(pbox)
        lay.addWidget(box)

        self._populate_presets(cfg)

    # ── 配置列表 ────────────────────────────────────────────────────────
    def _populate_presets(self, cfg):
        root = Path(cfg.root) if cfg is not None else None
        self._presets, errors = ([], ["未传入 cfg，配置列表不可用"]) if root is None \
            else ep.scan_presets(root / "experiments" / "presets")
        for pre in self._presets:
            tag = "线A·持续" if pre.line == "A" else "线B·节点"
            QListWidgetItem(f"[{tag}] {pre.display_name}", self.preset_list)
        for e in errors:
            it = QListWidgetItem(f"⚠ {e}", self.preset_list)
            it.setFlags(it.flags() & ~Qt.ItemFlag.ItemIsSelectable)
        if not self._presets and not errors:
            self.preset_list.addItem("（experiments/presets/ 下没有配置文件）")

    def _current_preset(self):
        i = self.preset_list.currentRow()
        return self._presets[i] if 0 <= i < len(self._presets) else None

    def _on_preset_selected(self, *_):
        pre = self._current_preset()
        self.preset_desc.setText(pre.description if pre else "选择一个配置查看操作提示")

    def _load_selected(self):
        pre = self._current_preset()
        if pre is None:
            QMessageBox.information(self, "未选择", "请先在列表里选中一个配置。")
            return
        if self._active_line or self._awaiting_line:
            QMessageBox.warning(self, "正在运行", "实验进行中，先点结束再换配置。")
            return
        # 只下发模式与轨迹参数——不使能、不运行
        self.command.emit({"cmd": "set_mode", "mode": pre.mode})
        self.command.emit(ep.trajectory_command(pre))
        self._loaded_preset = pre
        self.loaded_label.setText(
            f"当前配置：{pre.display_name}  →  点【{self._NAMES[pre.line]} 开始】")
        self.loaded_label.setStyleSheet(f"color:{COLOR_OK}; font-weight:bold;")
        self._set_idle()   # 刷新按钮文字（带配置名）

    def update_status(self, status: dict):
        self._status = status or {}
        # ── 线B 自动收尾（2026-08-13 现场需求）────────────────────────────
        # 节点实验的工况轨迹播完会自动软停（running 变 False）。检测到
        # "跑起来过 → 停了" 的下降沿，延迟几秒（等减速斜坡走完把尾巴录全）
        # 自动做与点【结束】完全相同的事：stop_run + record_stop + 落盘 +
        # 弹"实验完成"汇总框。持续运行（线A）不自动收尾——寿命运行由人停。
        if self._active_line == "B":
            if self._status.get("running"):
                self._b_was_running = True
            elif self._b_was_running:
                self._b_was_running = False
                QTimer.singleShot(self._auto_finish_delay_ms, self._auto_finish)

    def _auto_finish(self):
        # 延迟期间用户可能已手动点了结束/后端断连，状态复位过就不再收尾
        if self._active_line == "B" and self._awaiting_line is None:
            self._finish()

    def _precheck(self) -> str | None:
        """返回缺失项描述；全满足返回 None。字段名以 main_window 分发的 status 为准。"""
        s = self._status
        if s.get("ethercat") != "OP":
            return "EtherCAT 未进 OP。请先【启动主站】。"
        if s.get("servo") != "Operation Enabled":
            return "伺服未使能。请先 Servo Enable 并确认轨迹后再开始。"
        if not s.get("mode_matched", False):
            return "运行模式不匹配。请先在模式面板选定模式并应用轨迹。"
        return None

    def _toggle(self, line: str):
        if self._awaiting_line is not None:
            QMessageBox.information(self, "请稍候", "正在等待记录确认，请稍候…")
            return
        if self._active_line == line:
            self._finish()
        elif self._active_line is None:
            self._start(line)
        else:
            QMessageBox.warning(self, "另一条线在运行",
                                 f"线{self._active_line} 正在运行，请先结束它。")

    def _start(self, line: str):
        pre = self._loaded_preset
        if pre is not None and pre.line != line:
            QMessageBox.warning(
                self, "配置与按钮不匹配",
                f"当前配置「{pre.name}」应该用【{self._NAMES[pre.line]}】按钮。")
            return
        miss = self._precheck()
        if miss:
            QMessageBox.warning(self, "尚未就绪", miss)
            return
        last_dir = QSettings("zeroerr", "ecjc").value("exp_out_dir", "")
        # 线B：record 字段进弹框做只读预填（仅标注）。线A 恒速配置（摆臂持续
        # 运行）：speed_rpm_target 进弹框做【可编辑】预填，确认后真实下发覆盖。
        prefill = None
        if pre is not None:
            if line == "B" or pre.trajectory.get("type") == "constant":
                prefill = dict(pre.record)
        vals = StartDialog(line, last_dir, self, prefill=prefill).get_values()
        if vals is None:
            return
        # 线A 转速覆盖：以弹框值重发恒速轨迹（伺服可能已使能但尚未运行，
        # set_trajectory 走原子指针交接，安全）。record 标注同步用该值。
        if (line == "A" and pre is not None
                and pre.trajectory.get("type") == "constant"
                and "speed_rpm_target" in vals):
            traj = ep.trajectory_command(pre)
            traj["value"] = vals["speed_rpm_target"]
            self.command.emit(traj)
        # 真机 bug（2026-08-13）：out_dir 不存在时由 root 后端创建 → root 属主，
        # 采集正常但收尾 CSV 导出 EACCES。开始前以当前用户建目录并验写权限，
        # 有问题当场拦下，别让实验跑完 100s 才发现导不出。
        dir_err = self._ensure_writable(vals["out_dir"])
        if dir_err:
            QMessageBox.warning(self, "输出目录不可用", dir_err)
            return
        QSettings("zeroerr", "ecjc").setValue("exp_out_dir", vals["out_dir"])
        # baseline_stage 自动标记
        if line == "A":
            vals["baseline_stage"] = "continuous_run"
        else:
            vals["baseline_stage"] = "formal_0h" if vals.get("life_hours", 0) == 0 else "life_node"
        if pre is not None:
            vals["config_name"] = pre.name    # 结束弹框标题用
        self._pending = vals
        self._b_was_running = False
        # 数据文件名与配置名同步：加载了配置就用 <样机号>_<配置名>，
        # 手动模式保持原来的 <样机号>_line<A/B>。再拼寿命标注（2026-08-14
        # 需求：从文件名区分时间点/时间段）——线A 是自由文本区间（0-5h），
        # 线B 是数字节点小时（0h）；后端 DataLogger 会再补 _<时间戳>。
        test_name = (f"{vals['sample_id']}_{pre.name}" if pre is not None
                     else f"{vals['sample_id']}_line{line}")
        if line == "A":
            if vals.get("life_label"):
                test_name += f"_{vals['life_label']}"
        else:
            test_name += f"_{en._norm_num(vals['life_hours'])}h"
        rec = {"cmd": "record_start", "out_dir": vals["out_dir"],
               "sample_id": vals["sample_id"], "baseline_stage": vals["baseline_stage"],
               "test_name": test_name}
        if line == "B":
            rec.update(life_hours=vals["life_hours"], test_item=vals["test_item"],
                       rep=vals["rep"], load_percent_Tr=vals["load_percent_Tr"],
                       speed_rpm_target=vals["speed_rpm_target"])
        else:
            # 线A：寿命区间是自由文本（0-5h），装不进后端的 life_hours(double)，
            # 除文件名外再写进 notes 元数据留档
            if vals.get("life_label"):
                rec["notes"] = f"寿命区间: {vals['life_label']}"
            if "speed_rpm_target" in vals:
                rec["speed_rpm_target"] = vals["speed_rpm_target"]
        # I3（spec §5）：record_start 可能被后端拒绝（比如磁盘不足——DataLogger::start
        # 的真实拒绝路径），不能发了就当已经开始。这里只发 record_start，进"等待 ack"
        # 中间态；start_run 挪到 on_record_ack(ok=True) 里，ack 失败则整条回 idle。
        self._awaiting_line = line
        self.command.emit(rec)
        self._set_waiting(line)

    @staticmethod
    def _ensure_writable(out_dir: str) -> str | None:
        """以当前用户创建 out_dir 并确认可写。返回错误描述；一切正常返回 None。"""
        try:
            os.makedirs(out_dir, exist_ok=True)
        except OSError as e:
            return (f"无法创建输出目录 {out_dir}：{e}\n"
                    f"请换一个当前用户有权限的目录。")
        if not os.access(out_dir, os.W_OK):
            return (f"输出目录 {out_dir} 当前用户没有写权限"
                    f"（可能是之前由 root 后端创建的）。\n"
                    f"请换一个目录，或先修复属主：pkexec chown -R $USER {out_dir}")
        return None

    def on_record_ack(self, ok: bool, msg: str = "") -> bool:
        """main_window 收到 record_start 的 ack 后转发到这里（见 main_window._on_ack）。
        若这次 record_start 不是本面板发起的（比如 record_panel 手动触发），
        _awaiting_line 是 None，直接忽略——不误动本面板状态机，也不吞掉
        main_window 自己的失败提示（返回 False，main_window 该弹的框照常弹）。

        返回 True 表示这条 ack 确实是本面板发起、已经处理完（失败已弹过一次
        "记录未能启动"）——main_window 不应该再为同一条 ack 弹第二个框
        （复审发现的破坏 #2：record_start 失败时弹两个框）。
        """
        line = self._awaiting_line
        if line is None:
            return False
        self._awaiting_line = None
        if not ok:
            self._pending = None
            self._set_idle()
            QMessageBox.warning(self, "记录未能启动",
                                 msg or "record_start 被拒绝，未开始运行。")
            return True
        self.command.emit({"cmd": "start_run"})
        self._active_line = line
        self._set_running(line)
        return True

    def on_disconnected(self):
        """后端断连时复位——不会再有 record_start 的 ack 事件了，_awaiting_line
        若不清掉会永久卡在等待态，两个按钮永久禁用，只能重启 GUI（复审发现的
        破坏 #1，跟 P0 终审抓到的 cia402_panel._running 断连不复位同一类问题）。
        主站/伺服状态本身也已经不可信了，索性整条状态机回到干净的初始态。
        """
        self._awaiting_line = None
        self._active_line = None
        self._pending = None
        self._b_was_running = False
        self._set_idle()

    def _finish(self):
        self.command.emit({"cmd": "stop_run"})
        self.command.emit({"cmd": "record_stop"})
        line = self._active_line
        self._active_line = None
        self._b_was_running = False
        self._set_idle()
        # 发 finished 信号，把 line + 本次元数据交给 main_window：
        # main_window 知道 record 落盘的 h5 路径（从 recording 状态里拿），据此做 CSV 导出。
        self.finished.emit(line, self._pending or {})
        self._pending = None

    def _label(self, line: str, state: str) -> str:
        name = self._NAMES[line]
        pre = self._loaded_preset
        if pre is not None and pre.line == line:
            name = f"{name}·{pre.name}"
        if state == "start":
            return f"▶ {name} 开始"
        if state == "waiting":
            return f"⏳ {name} 等待记录确认…"
        return f"■ {name} 结束"   # running

    def _set_waiting(self, line):
        b = self.btn_a if line == "A" else self.btn_b
        other = self.btn_b if line == "A" else self.btn_a
        b.setText(self._label(line, "waiting"))
        b.setEnabled(False)
        other.setEnabled(False)
        self.btn_load.setEnabled(False)

    def _set_running(self, line):
        b = self.btn_a if line == "A" else self.btn_b
        other = self.btn_b if line == "A" else self.btn_a
        b.setText(self._label(line, "running"))
        b.setEnabled(True)
        other.setEnabled(False)
        self.btn_load.setEnabled(False)

    def _set_idle(self):
        self.btn_a.setText(self._label("A", "start"))
        self.btn_a.setEnabled(True)
        self.btn_b.setText(self._label("B", "start"))
        self.btn_b.setEnabled(True)
        self.btn_load.setEnabled(True)
