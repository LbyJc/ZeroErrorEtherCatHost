"""在线调参页（任务书第三十九节）。

关键设计：**控件是根据 Backend 上报的参数表动态生成的**。
控制器在 C++ 里 declareParams() 声明了什么，这里就长出什么。
以后加 RBFNN / Backstepping 控制器，只写 C++，这个文件一行都不用改。
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QScrollArea, QSlider, QVBoxLayout, QWidget,
)


class ParamRow(QWidget):
    """一个参数 = 滑块 + 数值框 + 单位。滑块用于快速扫，数值框用于精确设。"""

    changed = Signal(str, float)

    def __init__(self, meta: dict, parent=None):
        super().__init__(parent)
        self.name = meta["name"]
        self.lo = float(meta.get("min", 0))
        self.hi = float(meta.get("max", 1))
        if self.hi <= self.lo:
            self.hi = self.lo + 1.0
        step = float(meta.get("step", 0.01)) or 0.01

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(self.lo, self.hi)
        self.spin.setSingleStep(step)
        self.spin.setDecimals(max(0, min(6, _decimals(step))))
        self.spin.setKeyboardTracking(False)
        self.spin.setMaximumWidth(110)

        unit = QLabel(meta.get("unit", ""))
        unit.setStyleSheet("color:#666;")
        unit.setMinimumWidth(70)

        lay.addWidget(self.slider, 1)
        lay.addWidget(self.spin)
        lay.addWidget(unit)

        self.set_value(float(meta.get("value", meta.get("default", self.lo))))
        self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)

    def _to_slider(self, v):
        return int(round((v - self.lo) / (self.hi - self.lo) * 1000))

    def _from_slider(self, s):
        return self.lo + (self.hi - self.lo) * s / 1000.0

    def set_value(self, v):
        v = max(self.lo, min(self.hi, v))
        for w in (self.slider, self.spin):
            w.blockSignals(True)
        self.spin.setValue(v)
        self.slider.setValue(self._to_slider(v))
        for w in (self.slider, self.spin):
            w.blockSignals(False)

    def _on_slider(self, s):
        v = self._from_slider(s)
        self.spin.blockSignals(True)
        self.spin.setValue(v)
        self.spin.blockSignals(False)
        self.changed.emit(self.name, v)

    def _on_spin(self, v):
        self.slider.blockSignals(True)
        self.slider.setValue(self._to_slider(v))
        self.slider.blockSignals(False)
        self.changed.emit(self.name, v)


def _decimals(step: float) -> int:
    s = f"{step:.10f}".rstrip("0")
    return len(s.split(".")[1]) if "." in s else 0


class ParameterPanel(QWidget):
    command = Signal(dict)

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._rows: dict[str, ParamRow] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        head = QLabel(
            "参数修改路径：GUI → IPC → 线程安全参数块（双缓冲 + 原子代号）→ 实时控制器。\n"
            "实时线程每周期只读一次代号，变了才拷贝，全程不加锁，最坏延迟 1 个控制周期。")
        head.setWordWrap(True)
        head.setStyleSheet("color:#666; font-size:11px;")
        root.addWidget(head)

        self.title = QLabel("当前控制器：—")
        self.title.setStyleSheet("font-weight:bold; font-size:13px;")
        root.addWidget(self.title)

        area = QScrollArea()
        area.setWidgetResizable(True)
        self.host = QWidget()
        self.form = QFormLayout(self.host)
        area.setWidget(self.host)
        root.addWidget(area, 1)

        self.empty = QLabel("当前控制器没有可调参数。\n"
                            "（直通控制器把轨迹给定原样下发，本身不含参数。）")
        self.empty.setStyleSheet("color:#888;")
        self.form.addRow(self.empty)

        btns = QHBoxLayout()
        b_reload = QPushButton("重新读取参数")
        b_reload.clicked.connect(lambda: self.command.emit({"cmd": "get_params"}))
        btns.addWidget(b_reload)
        btns.addStretch(1)
        root.addLayout(btns)

    def set_params(self, obj: dict):
        items = obj.get("items", [])

        # 清空旧控件
        while self.form.rowCount():
            self.form.removeRow(0)
        self._rows.clear()

        if not items:
            self.empty = QLabel("当前控制器没有可调参数。")
            self.empty.setStyleSheet("color:#888;")
            self.form.addRow(self.empty)
            return

        for m in items:
            row = ParamRow(m)
            row.changed.connect(self._on_change)
            self._rows[m["name"]] = row
            label = QLabel(f"{m.get('label', m['name'])}\n{m['name']}")
            label.setStyleSheet("font-size:11px;")
            self.form.addRow(label, row)

    def set_controller_name(self, name: str):
        self.title.setText(f"当前控制器：{name}")

    def _on_change(self, name: str, value: float):
        self.command.emit({"cmd": "set_param", "name": name, "value": value})
