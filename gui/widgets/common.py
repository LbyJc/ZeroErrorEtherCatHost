"""GUI 通用小部件。

状态指示一律采用「形状 + 文字 + 颜色」三重编码。
任务书第十七节明确要求"不能只用颜色表示"——色觉障碍者约占男性人口 8%，
而且实验室投影和拍照时颜色也常常失真。形状和文字在任何条件下都读得出来。
"""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QFrame, QGroupBox, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)

# 语义色
COLOR_OK = "#2e7d32"
COLOR_WARN = "#ed6c02"
COLOR_ERR = "#c62828"
COLOR_IDLE = "#616161"
COLOR_ACTIVE = "#0277bd"

# 形状：不同状态用不同符号，黑白打印也能区分
GLYPH = {
    "ok": "●",
    "warn": "▲",
    "error": "■",
    "idle": "○",
    "active": "◆",
}


class StatusLamp(QWidget):
    """形状 + 文字 + 颜色 的状态灯。"""

    def __init__(self, caption: str, parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)

        self._cap = QLabel(caption)
        self._cap.setStyleSheet("color:#666;")
        self._glyph = QLabel(GLYPH["idle"])
        self._text = QLabel("—")
        f = QFont()
        f.setBold(True)
        self._text.setFont(f)
        self._glyph.setFont(f)

        lay.addWidget(self._cap)
        lay.addWidget(self._glyph)
        lay.addWidget(self._text)
        lay.addStretch(1)
        self.set_state("idle", "—")

    def set_state(self, kind: str, text: str, tooltip: str = ""):
        color = {
            "ok": COLOR_OK, "warn": COLOR_WARN, "error": COLOR_ERR,
            "idle": COLOR_IDLE, "active": COLOR_ACTIVE,
        }.get(kind, COLOR_IDLE)
        self._glyph.setText(GLYPH.get(kind, GLYPH["idle"]))
        self._glyph.setStyleSheet(f"color:{color};")
        self._text.setText(text)
        self._text.setStyleSheet(f"color:{color};")
        if tooltip:
            self.setToolTip(tooltip)


class ValueRow(QWidget):
    """一行「名称 —— 数值 单位」。数值用等宽字体，跳动时不会左右晃。"""

    def __init__(self, label: str, unit: str = "", parent=None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(2, 1, 2, 1)
        self._name = QLabel(label)
        self._value = QLabel("—")
        self._unit = QLabel(unit)
        f = QFont("monospace")
        f.setStyleHint(QFont.Monospace)
        self._value.setFont(f)
        self._value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._value.setMinimumWidth(110)
        self._unit.setStyleSheet("color:#666;")
        self._unit.setMinimumWidth(34)
        lay.addWidget(self._name)
        lay.addStretch(1)
        lay.addWidget(self._value)
        lay.addWidget(self._unit)

    def set_value(self, v, fmt: str = "{:.3f}"):
        self._value.setText("—" if v is None else fmt.format(v))

    def set_text(self, s: str):
        self._value.setText(s)

    def set_color(self, color: str):
        self._value.setStyleSheet(f"color:{color};")


class BitField(QWidget):
    """16 位控制字/状态字的逐位显示。

    悬停每一位会给出该位的含义，不用去翻手册。
    """

    def __init__(self, names: dict[int, str], parent=None):
        super().__init__(parent)
        self._names = names
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(1)
        self._bits = []
        f = QFont("monospace")
        f.setPointSize(8)
        for i in range(15, -1, -1):
            lb = QLabel("0")
            lb.setFont(f)
            lb.setAlignment(Qt.AlignCenter)
            lb.setFixedWidth(14)
            lb.setFrameShape(QFrame.Box)
            lb.setToolTip(f"bit{i}: {names.get(i, '—')}")
            lay.addWidget(lb)
            self._bits.append((i, lb))
        lay.addStretch(1)

    def set_value(self, v: int):
        for i, lb in self._bits:
            on = bool(v & (1 << i))
            lb.setText("1" if on else "0")
            named = i in self._names
            if on:
                lb.setStyleSheet(
                    f"background:{COLOR_ACTIVE};color:white;"
                    if named else "background:#bbb;color:#fff;")
            else:
                lb.setStyleSheet("background:transparent;color:#999;")


def fmt_hms(seconds: float) -> str:
    """秒 → "H:MM:SS"。负数/None 容错为 0（后端旧版本没有该字段时显示 0:00:00）。"""
    s = int(seconds) if seconds and seconds > 0 else 0
    return f"{s // 3600}:{s % 3600 // 60:02d}:{s % 60:02d}"


def group(title: str, widget: QWidget) -> QGroupBox:
    g = QGroupBox(title)
    lay = QVBoxLayout(g)
    lay.setContentsMargins(6, 6, 6, 6)
    lay.addWidget(widget)
    return g


def vbox(*widgets, spacing=4, margins=(6, 6, 6, 6)) -> QWidget:
    w = QWidget()
    lay = QVBoxLayout(w)
    lay.setContentsMargins(*margins)
    lay.setSpacing(spacing)
    for x in widgets:
        if x is None:
            lay.addStretch(1)
        else:
            lay.addWidget(x)
    return w


CONTROLWORD_BITS = {
    0: "Switch On",
    1: "Enable Voltage",
    2: "Quick Stop (低有效)",
    3: "Enable Operation",
    4: "Mode Specific 1",
    5: "Mode Specific 2",
    6: "Mode Specific 3",
    7: "Fault Reset (上升沿)",
    8: "Halt",
}

STATUSWORD_BITS = {
    0: "Ready to Switch On",
    1: "Switched On",
    2: "Operation Enabled",
    3: "Fault",
    4: "Voltage Enabled",
    5: "Quick Stop (低有效)",
    6: "Switch On Disabled",
    7: "Warning",
    9: "Remote",
    10: "Target Reached",
    11: "Internal Limit Active",
    12: "Mode Specific 1",
    13: "Mode Specific 2",
}
