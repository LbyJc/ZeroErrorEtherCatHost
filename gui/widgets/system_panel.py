"""左侧：EtherCAT 主站管理 + 系统信息（任务书第七、五十一节）。"""
from __future__ import annotations

import subprocess
from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from .common import COLOR_ERR, COLOR_OK, COLOR_WARN, ValueRow

# 【启动主站】的九步进度（任务书第八节）。顺序即显示顺序。
STARTUP_STEPS = [
    "Configuration Loaded",
    "Network Interface Found",
    "IgH Master Ready",
    "Backend Running",
    "EtherCAT Slave Detected",
    "PDO Configured",
    "Distributed Clock Configured",
    "Startup Parameters",
    "Master Activated",
    "SAFEOP",
    "OP",
    "Realtime Task Running",
]


class SystemPanel(QWidget):
    command = Signal(dict)

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._running = False
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── 系统信息 ────────────────────────────────────────────────
        info = QGroupBox("系统")
        ig = QVBoxLayout(info)
        ig.setSpacing(1)
        self.r_backend = ValueRow("Backend")
        self.r_master = ValueRow("Master")
        self.r_iface = ValueRow("Interface")
        self.r_link = ValueRow("Link")
        self.r_slave = ValueRow("Slave")
        self.r_ec = ValueRow("EtherCAT")
        self.r_wc = ValueRow("Working Counter")
        self.r_dc = ValueRow("DC Sync")
        self.r_cycle = ValueRow("Cycle", "µs")
        for r in (self.r_backend, self.r_master, self.r_iface, self.r_link,
                  self.r_slave, self.r_ec, self.r_wc, self.r_dc, self.r_cycle):
            ig.addWidget(r)
        root.addWidget(info)

        # ── 主站控制 ────────────────────────────────────────────────
        ctl = QGroupBox("EtherCAT 主站")
        cg = QGridLayout(ctl)
        self.btn_start = QPushButton("启动主站")
        self.btn_stop = QPushButton("停止主站")
        self.btn_reconnect = QPushButton("重新连接")
        self.btn_start.setStyleSheet("font-weight:bold;")
        cg.addWidget(self.btn_start, 0, 0)
        cg.addWidget(self.btn_stop, 0, 1)
        cg.addWidget(self.btn_reconnect, 1, 0, 1, 2)
        self.btn_start.clicked.connect(lambda: self._cmd("connect_bus"))
        self.btn_stop.clicked.connect(self._on_stop_master)
        self.btn_reconnect.clicked.connect(lambda: self._cmd("reconnect"))
        root.addWidget(ctl)

        # ── 启动进度 ────────────────────────────────────────────────
        prog = QGroupBox("启动进度")
        pg = QVBoxLayout(prog)
        self.steps = QListWidget()
        self.steps.setAlternatingRowColors(True)
        self.steps.setMinimumHeight(190)
        pg.addWidget(self.steps)
        self._got_steps = False
        self.reset_steps()
        root.addWidget(prog)

        # ── 快捷入口 ────────────────────────────────────────────────
        open_box = QGroupBox("打开")
        og = QGridLayout(open_box)
        b_cfg = QPushButton("配置目录")
        b_data = QPushButton("数据目录")
        b_log = QPushButton("日志目录")
        og.addWidget(b_cfg, 0, 0)
        og.addWidget(b_data, 0, 1)
        og.addWidget(b_log, 1, 0, 1, 2)
        b_cfg.clicked.connect(lambda: self._open(self.cfg.config_dir))
        b_data.clicked.connect(lambda: self._open(self.cfg.data_dir))
        b_log.clicked.connect(lambda: self._open(self.cfg.log_dir))
        root.addWidget(open_box)

        root.addStretch(1)

    def _on_stop_master(self):
        if self._running:
            QMessageBox.warning(
                self, "正在运行中",
                "请先【停止运行】或【安全停机】，再停止主站。\n"
                "停主站会等待软停完成（最长 20 s），运行中直接停会拉长该等待。")
            return
        self._cmd("disconnect_bus")

    def _cmd(self, name, **kw):
        self.command.emit({"cmd": name, **kw})
        if name in ("connect_bus", "reconnect"):
            self.reset_steps()

    @staticmethod
    def _open(path: Path):
        path = Path(path)
        path.mkdir(parents=True, exist_ok=True)
        subprocess.Popen(["xdg-open", str(path)])

    # ── 启动进度 ────────────────────────────────────────────────────────
    def reset_steps(self):
        self.steps.clear()
        self._got_steps = False
        for s in STARTUP_STEPS:
            it = QListWidgetItem(f"[ ] {s}")
            it.setForeground(_brush("#888"))
            self.steps.addItem(it)

    def mark_already_running(self):
        """GUI 连上时主站已经在跑（启动事件发生在本次连接之前）。

        原来这种情况下进度列表全是空的 [ ]，看着像"启动失败"，
        实际是这些事件 GUI 根本没机会收到。这里按当前状态补齐，
        并明确标注是"本次连接前完成的"，不冒充本次观测到的结果。
        """
        if self._got_steps:
            return
        for i in range(self.steps.count()):
            it = self.steps.item(i)
            name = it.text()[4:].split("  —")[0]
            it.setText(f"[✓] {name}  — 本次连接前已完成")
            it.setForeground(_brush("#2e7d32"))
            it.setToolTip("GUI 连接时主站已处于运行状态，"
                          "该步骤的进度事件发生在本次连接之前。")

    def set_step(self, step: str, ok: bool, msg: str):
        self._got_steps = True
        for i in range(self.steps.count()):
            it = self.steps.item(i)
            if it.text()[4:].split("  —")[0] == step:
                mark = "✓" if ok else "✗"
                text = f"[{mark}] {step}"
                if msg:
                    text += f"  — {msg}"
                it.setText(text)
                it.setForeground(_brush(COLOR_OK if ok else COLOR_ERR))
                if msg:
                    it.setToolTip(msg)
                self.steps.scrollToItem(it)
                return
        # 未在预置列表里的步骤：追加显示，不要静默吞掉
        it = QListWidgetItem(f"[{'✓' if ok else '✗'}] {step}  — {msg}")
        it.setForeground(_brush(COLOR_OK if ok else COLOR_ERR))
        self.steps.addItem(it)

    def set_disconnected(self):
        """与后端断开时调用。

        必须显式重置按钮状态：断开后不会再有 status 事件进来，
        如果不重置，按钮会**冻结在断开前的状态**——后端崩溃前主站是 OP，
        【启动主站】是灰的，崩溃后它就一直灰着，点了没反应，
        连"自动拉起后端"的路径都走不到。这正是现场遇到的现象。
        """
        for r in (self.r_backend, self.r_master, self.r_link, self.r_slave,
                  self.r_ec, self.r_wc, self.r_dc):
            r.set_text("—")
            r.set_color("#888")
        self._running = False
        self.r_backend.set_text("Stopped")
        self.r_backend.set_color(COLOR_ERR)
        # 断开时唯一该亮的就是【启动主站】——它负责把后端拉起来
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_reconnect.setEnabled(False)

    # ── 状态刷新 ────────────────────────────────────────────────────────
    def update_status(self, st, ipc_connected: bool):
        g = st.get
        self._running = g("running", False)
        if g("slave_online", False) and g("ethercat", "") == "OP":
            self.mark_already_running()
        self.r_backend.set_text("Running" if ipc_connected else "Stopped")
        self.r_backend.set_color(COLOR_OK if ipc_connected else COLOR_ERR)

        online = g("slave_online", False)
        self.r_master.set_text("Running" if online else "Stopped")
        self.r_master.set_color(COLOR_OK if online else "#888")

        self.r_iface.set_text(str(g("interface", "—")))
        link = g("link_up", False)
        self.r_link.set_text("Up" if link else "Down")
        self.r_link.set_color(COLOR_OK if link else COLOR_ERR)

        cnt = g("slave_count", 0)
        self.r_slave.set_text(f"{cnt} 个" if cnt else "未检测到")
        self.r_slave.set_color(COLOR_OK if cnt else "#888")

        ec = g("ethercat", "—")
        self.r_ec.set_text(ec)
        self.r_ec.set_color(COLOR_OK if ec == "OP" else COLOR_WARN)

        wc, wcs = g("wc", 0), g("wc_state", 0)
        self.r_wc.set_text(f"{wc} ({['ZERO','INCOMPLETE','COMPLETE'][wcs] if 0<=wcs<3 else '?'})")
        self.r_wc.set_color(COLOR_OK if wcs == 2 else COLOR_ERR)

        self.r_dc.set_text("启用" if g("dc_ok", False) else "关闭")
        self.r_cycle.set_text(str(g("cycle_us", "—")))

        # 按钮可用性跟随实际状态
        self.btn_start.setEnabled(ipc_connected and not online)
        self.btn_stop.setEnabled(ipc_connected and online)
        self.btn_reconnect.setEnabled(ipc_connected)


def _brush(color: str):
    from PySide6.QtGui import QBrush, QColor
    return QBrush(QColor(color))
