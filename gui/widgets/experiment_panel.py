"""一键实验编排面板：两个大按钮，半自动同步起停采集+运行。

前置检查读的状态字段名以 main_window._on_status 分发给各面板的 status dict 为准
（实测：backend IpcServer::statusJson 里 EtherCAT 状态字段是 "ethercat"，不是
"ethercat_state"；伺服状态字段是 "servo"，值是 cia402.cpp::toString(Cia402State)
的输出，OperationEnabled -> "Operation Enabled"）。
"""
from __future__ import annotations

from PySide6.QtCore import Signal, QSettings
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QPushButton, QMessageBox, QGroupBox, QVBoxLayout,
)

from .experiment_dialog import StartDialog


class ExperimentPanel(QWidget):
    command = Signal(dict)
    # 结束时发出：(line, pending_meta)。main_window 连它做 CSV 导出（仅线 B）+ 弹汇总框。
    # 用显式信号而非让 main_window 读私有属性——跨对象接口定死，避免时序竞争。
    finished = Signal(str, dict)

    _NAMES = {"A": "持续运行(线A)", "B": "节点实验(线B)"}

    def __init__(self, parent=None):
        super().__init__(parent)
        self._status = {}
        self._active_line = None          # None / "A" / "B"（record_start 已 ack、在跑）
        self._awaiting_line = None        # None / "A" / "B"（record_start 已发，等 ack）
        self._pending = None              # 开始时存的元数据，结束导出用
        self.btn_a = QPushButton(self._label("A", "start"))
        self.btn_b = QPushButton(self._label("B", "start"))
        self.btn_a.clicked.connect(lambda: self._toggle("A"))
        self.btn_b.clicked.connect(lambda: self._toggle("B"))
        box = QGroupBox("一键实验")
        row = QHBoxLayout()
        row.addWidget(self.btn_a)
        row.addWidget(self.btn_b)
        box.setLayout(row)
        lay = QVBoxLayout(self)
        lay.addWidget(box)

    def update_status(self, status: dict):
        self._status = status or {}

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
        # I3（spec §5）：record_start 可能被后端拒绝（比如磁盘不足——DataLogger::start
        # 的真实拒绝路径），不能发了就当已经开始。这里只发 record_start，进"等待 ack"
        # 中间态；start_run 挪到 on_record_ack(ok=True) 里，ack 失败则整条回 idle。
        self._awaiting_line = line
        self.command.emit(rec)
        self._set_waiting(line)

    def on_record_ack(self, ok: bool, msg: str = ""):
        """main_window 收到 record_start 的 ack 后转发到这里（见 main_window._on_ack）。
        若这次 record_start 不是本面板发起的（比如 record_panel 手动触发），
        _awaiting_line 是 None，直接忽略——不误动本面板状态机。
        """
        line = self._awaiting_line
        if line is None:
            return
        self._awaiting_line = None
        if not ok:
            self._pending = None
            self._set_idle()
            QMessageBox.warning(self, "记录未能启动",
                                 msg or "record_start 被拒绝，未开始运行。")
            return
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
        # main_window 知道 record 落盘的 h5 路径（从 recording 状态里拿），据此做 CSV 导出。
        self.finished.emit(line, self._pending or {})
        self._pending = None

    def _label(self, line: str, state: str) -> str:
        name = self._NAMES[line]
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

    def _set_running(self, line):
        b = self.btn_a if line == "A" else self.btn_b
        other = self.btn_b if line == "A" else self.btn_a
        b.setText(self._label(line, "running"))
        b.setEnabled(True)
        other.setEnabled(False)

    def _set_idle(self):
        self.btn_a.setText(self._label("A", "start"))
        self.btn_a.setEnabled(True)
        self.btn_b.setText(self._label("B", "start"))
        self.btn_b.setEnabled(True)
