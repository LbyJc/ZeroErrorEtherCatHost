"""日志页（任务书第四十二节）。GUI 显示 + 同时写文件。"""
from __future__ import annotations

import datetime
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont, QTextCharFormat, QTextCursor
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QHBoxLayout, QLabel, QPushButton, QPlainTextEdit,
    QVBoxLayout, QWidget,
)

LEVELS = ["DEBUG", "INFO", "WARNING", "ERROR"]
LEVEL_COLOR = {
    "DEBUG": "#888888",
    "INFO": "#000000",
    "WARNING": "#ed6c02",
    "ERROR": "#c62828",
}
MAX_LINES = 5000     # GUI 里只留最近 5000 行，完整日志在文件里


class LogPanel(QWidget):
    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._min_level = 1     # 默认 INFO

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        bar = QHBoxLayout()
        bar.addWidget(QLabel("最低级别"))
        self.level = QComboBox()
        self.level.addItems(LEVELS)
        self.level.setCurrentIndex(1)
        self.level.currentIndexChanged.connect(self._on_level)
        bar.addWidget(self.level)

        self.autoscroll = QCheckBox("自动滚动")
        self.autoscroll.setChecked(True)
        bar.addWidget(self.autoscroll)

        b_clear = QPushButton("清空显示")
        b_clear.clicked.connect(lambda: self.view.clear())
        bar.addWidget(b_clear)

        b_open = QPushButton("打开日志文件")
        b_open.clicked.connect(self._open_log)
        bar.addWidget(b_open)

        self.file_label = QLabel("")
        self.file_label.setStyleSheet("color:#888; font-size:11px;")
        bar.addWidget(self.file_label)
        bar.addStretch(1)
        root.addLayout(bar)

        self.view = QPlainTextEdit()
        self.view.setReadOnly(True)
        self.view.setMaximumBlockCount(MAX_LINES)
        f = QFont("monospace")
        f.setStyleHint(QFont.Monospace)
        f.setPointSize(9)
        self.view.setFont(f)
        root.addWidget(self.view, 1)

        # 日志文件
        self._path = None
        try:
            d = Path(cfg.log_dir)
            d.mkdir(parents=True, exist_ok=True)
            self._path = d / f"gui_{datetime.datetime.now():%Y%m%d_%H%M%S}.log"
            self._fh = open(self._path, "a", encoding="utf-8", buffering=1)
            self.file_label.setText(str(self._path))
        except Exception as e:
            self._fh = None
            self.file_label.setText(f"（日志文件不可写: {e}）")

    def _on_level(self, i):
        self._min_level = i

    def _open_log(self):
        if self._path:
            import subprocess
            subprocess.Popen(["xdg-open", str(self._path)])

    def append(self, level: str, msg: str):
        level = level.upper()
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"{ts} [{level:<7}] {msg}"

        if self._fh:
            try:
                self._fh.write(line + "\n")
            except Exception:
                pass

        try:
            idx = LEVELS.index(level)
        except ValueError:
            idx = 1
        if idx < self._min_level:
            return

        fmt = QTextCharFormat()
        fmt.setForeground(QColor(LEVEL_COLOR.get(level, "#000")))
        if level == "ERROR":
            fmt.setFontWeight(QFont.Bold)

        cur = self.view.textCursor()
        cur.movePosition(QTextCursor.End)
        cur.insertText(line + "\n", fmt)
        if self.autoscroll.isChecked():
            self.view.verticalScrollBar().setValue(
                self.view.verticalScrollBar().maximum())
