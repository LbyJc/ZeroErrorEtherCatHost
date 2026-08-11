"""数据采集页（任务书第三十、三十一、四十九节）。

采集与 Run 完全独立：可以先开采集再运行，也可以只采集不运行。
"""
from __future__ import annotations

import subprocess
from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPlainTextEdit, QProgressBar,
    QPushButton, QVBoxLayout, QWidget, QFormLayout,
)

from .common import COLOR_ERR, COLOR_OK, COLOR_WARN, ValueRow


class DataRecordPanel(QWidget):
    command = Signal(dict)

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        # ── 实验元信息 ──────────────────────────────────────────────
        meta = QGroupBox("实验信息（写入 HDF5 metadata，便于论文数据追溯）")
        mf = QFormLayout(meta)
        self.test_name = QLineEdit("exp")
        self.desc = QPlainTextEdit()
        self.desc.setMaximumHeight(70)
        self.desc.setPlaceholderText("实验目的、被试件、工况等")
        mf.addRow("测试名称", self.test_name)
        mf.addRow("描述", self.desc)
        root.addWidget(meta)

        # ── 控制 ────────────────────────────────────────────────────
        ctl = QGroupBox("采集控制")
        cl = QHBoxLayout(ctl)
        self.btn_start = QPushButton("开始数据采集")
        self.btn_stop = QPushButton("停止数据采集")
        self.btn_start.setMinimumHeight(38)
        self.btn_stop.setMinimumHeight(38)
        self.btn_start.setStyleSheet("font-weight:bold;")
        self.btn_open = QPushButton("打开数据目录")
        cl.addWidget(self.btn_start)
        cl.addWidget(self.btn_stop)
        cl.addWidget(self.btn_open)
        self.btn_start.clicked.connect(self._start)
        self.btn_stop.clicked.connect(lambda: self.command.emit({"cmd": "record_stop"}))
        self.btn_open.clicked.connect(self._open_dir)
        root.addWidget(ctl)

        # ── 状态 ────────────────────────────────────────────────────
        st = QGroupBox("采集状态")
        sg = QVBoxLayout(st)
        sg.setSpacing(1)
        self.r_state = ValueRow("状态")
        self.r_file = ValueRow("当前文件")
        self.r_start = ValueRow("开始时间")
        self.r_now = ValueRow("当前时间")
        self.r_elapsed = ValueRow("已采集时长", "s")
        self.r_rate = ValueRow("采样率", "Hz")
        self.r_count = ValueRow("样本数")
        self.r_dropped = ValueRow("丢弃样本")
        self.r_size = ValueRow("文件大小", "MB")
        self.r_disk = ValueRow("磁盘剩余", "GB")
        for r in (self.r_state, self.r_file, self.r_start, self.r_now, self.r_elapsed,
                  self.r_rate, self.r_count, self.r_dropped, self.r_size, self.r_disk):
            sg.addWidget(r)

        sg.addWidget(QLabel("Ring Buffer 占用"))
        self.buf = QProgressBar()
        self.buf.setRange(0, 100)
        sg.addWidget(self.buf)

        self.note = QLabel(
            "长时间实验说明：数据经无锁 ring buffer 交给独立 Logger 线程批量写 HDF5，"
            "不占用实时线程，也不全量驻留内存。1 kHz × 23 字段约 190 kB/s，"
            "开 gzip 后 10 小时约 2~3 GB。\n"
            "「丢弃样本」非零说明磁盘写入跟不上采样率——这个数字会如实记录，"
            "不会被悄悄抹掉。")
        self.note.setWordWrap(True)
        self.note.setStyleSheet("color:#666; font-size:11px;")
        sg.addWidget(self.note)
        root.addWidget(st)
        root.addStretch(1)

        self._active = False
        self.update_recording({})

    def _start(self):
        self.command.emit({
            "cmd": "record_start",
            "test_name": self.test_name.text().strip() or "exp",
            "description": self.desc.toPlainText(),
        })

    def _open_dir(self):
        d = Path(self.cfg.data_dir)
        d.mkdir(parents=True, exist_ok=True)
        subprocess.Popen(["xdg-open", str(d)])

    def update_recording(self, rec: dict):
        active = bool(rec.get("active"))
        self._active = active
        self.r_state.set_text("Recording" if active else "Stopped")
        self.r_state.set_color(COLOR_OK if active else "#888")
        self.r_file.set_text(Path(rec.get("file", "") or "—").name)
        self.r_elapsed.set_value(rec.get("elapsed_s", 0.0), "{:.1f}")
        self.r_count.set_text(f"{int(rec.get('samples', 0)):,}")

        dropped = int(rec.get("dropped", 0))
        self.r_dropped.set_text(f"{dropped:,}")
        self.r_dropped.set_color(COLOR_ERR if dropped else "#888")

        self.r_size.set_value(rec.get("bytes", 0) / 1e6, "{:.2f}")
        disk = rec.get("disk_free_gb", 0.0)
        self.r_disk.set_value(disk, "{:.1f}")
        self.r_disk.set_color(COLOR_ERR if 0 <= disk < 5 else "#888")

        usage = float(rec.get("buffer_usage", 0.0)) * 100
        self.buf.setValue(int(usage))
        self.buf.setFormat(f"{usage:.1f}%")

        self.btn_start.setEnabled(not active)
        self.btn_stop.setEnabled(active)

        ns = int(rec.get("start_time_ns", 0))
        if ns:
            import datetime
            self.r_start.set_text(
                datetime.datetime.fromtimestamp(ns / 1e9).strftime("%Y-%m-%d %H:%M:%S"))

    def update_status(self, st):
        cyc = st.get("cycle_us", 1000) or 1000
        self.r_rate.set_value(1e6 / cyc, "{:.0f}")
        import datetime
        self.r_now.set_text(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
