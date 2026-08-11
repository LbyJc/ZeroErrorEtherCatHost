"""实时曲线（任务书第二十六~二十九节）。

内存策略是这里最要紧的事：**Plot 只保留当前时间窗需要的数据**。
缓冲长度 = 最大时间窗 × 遥测频率，启动时一次性分配 numpy 数组，
之后只在里面滚动写。10 小时实验和 10 秒实验占用完全一样的内存。
完整数据由 Backend 的 Logger 单独落盘，GUI 不承担存储职责。
"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
    QVBoxLayout, QWidget,
)

pg.setConfigOptions(antialias=False, background="w", foreground="k")

# 曲线颜色：目标值用虚线，实测用实线，避免只靠颜色区分
SERIES_STYLE = {
    "target_position_deg": ("#c62828", Qt.DashLine, "目标位置"),
    "motor_position_deg": ("#1565c0", Qt.SolidLine, "电机位置"),
    "output_position_deg": ("#2e7d32", Qt.SolidLine, "输出位置"),
    "target_velocity_rpm": ("#c62828", Qt.DashLine, "目标转速"),
    "motor_velocity_rpm": ("#1565c0", Qt.SolidLine, "电机转速"),
    "output_velocity_rpm": ("#2e7d32", Qt.SolidLine, "输出转速"),
    "motor_current_A": ("#6a1b9a", Qt.SolidLine, "电机电流"),
    "target_torque_Nm": ("#c62828", Qt.DashLine, "目标力矩"),
    "actual_torque_Nm": ("#ef6c00", Qt.SolidLine, "实际力矩"),
    "position_error_deg": ("#00838f", Qt.SolidLine, "位置误差"),
    "velocity_error_rpm": ("#00838f", Qt.SolidLine, "速度误差"),
}


class SinglePlot(QWidget):
    def __init__(self, spec: dict, capacity: int, parent=None):
        super().__init__(parent)
        self.spec = spec
        self.capacity = capacity
        self.series = [s for s in spec.get("series", []) if s in SERIES_STYLE]

        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(2)

        # ── 轴控制条 ────────────────────────────────────────────────
        bar = QHBoxLayout()
        bar.setSpacing(4)
        self.auto_y = QCheckBox("Auto Y")
        self.auto_y.setChecked(bool(spec.get("auto_y", True)))
        self.ymin = QDoubleSpinBox()
        self.ymax = QDoubleSpinBox()
        for s, v in ((self.ymin, spec.get("y_min", -1.0)), (self.ymax, spec.get("y_max", 1.0))):
            s.setRange(-1e9, 1e9)
            s.setDecimals(3)
            s.setValue(float(v))
            s.setMaximumWidth(90)
            s.setKeyboardTracking(False)
        bar.addWidget(QLabel(spec.get("title", "")))
        bar.addStretch(1)
        bar.addWidget(self.auto_y)
        bar.addWidget(QLabel("Y min"))
        bar.addWidget(self.ymin)
        bar.addWidget(QLabel("Y max"))
        bar.addWidget(self.ymax)
        lay.addLayout(bar)

        # ── 绘图 ────────────────────────────────────────────────────
        self.pw = pg.PlotWidget()
        self.pw.showGrid(x=True, y=True, alpha=0.25)
        self.pw.setLabel("bottom", "时间", units="s")
        self.pw.addLegend(offset=(-10, 10))
        lay.addWidget(self.pw, 1)

        self.curves = {}
        for name in self.series:
            color, style, label = SERIES_STYLE[name]
            pen = pg.mkPen(color=color, width=2, style=style)
            # connect="finite"：遇到 NaN 断开线段。
            # wrapped 位置在 360°→0° 处会回绕，直接连线会画出一条竖直的假跳变，
            # 让人以为关节瞬间转了 360 度。在回绕处插 NaN 把线断开才是诚实的画法。
            self.curves[name] = self.pw.plot([], [], pen=pen, name=label,
                                             connect="finite")

        # 环形缓冲：固定长度，永不增长
        self._t = np.zeros(capacity, dtype=np.float64)
        self._y = {n: np.zeros(capacity, dtype=np.float64) for n in self.series}
        self._n = 0          # 已写入总数（可超过 capacity）

        self.auto_y.toggled.connect(self._apply_y)
        self.ymin.valueChanged.connect(self._apply_y)
        self.ymax.valueChanged.connect(self._apply_y)
        self._apply_y()

    @staticmethod
    def _is_wrapped(name: str) -> bool:
        """是否是 [0,360) 的 wrapped 位置（回绕处要断线，不能直连）。"""
        return name.endswith("position_deg") and "unwrapped" not in name

    def _apply_y(self):
        if self.auto_y.isChecked():
            self.pw.enableAutoRange(axis="y")
            self.ymin.setEnabled(False)
            self.ymax.setEnabled(False)
        else:
            self.pw.disableAutoRange(axis="y")
            lo, hi = self.ymin.value(), self.ymax.value()
            if hi <= lo:
                hi = lo + 1e-6
            self.pw.setYRange(lo, hi, padding=0)
            self.ymin.setEnabled(True)
            self.ymax.setEnabled(True)

    def append(self, t: np.ndarray, arr: np.ndarray):
        k = len(t)
        if k == 0:
            return
        if k >= self.capacity:
            # 单批就超过缓冲：只留最后 capacity 个
            t = t[-self.capacity:]
            arr = arr[-self.capacity:]
            k = self.capacity

        idx = self._n % self.capacity
        end = idx + k
        if end <= self.capacity:
            self._t[idx:end] = t
            for n in self.series:
                self._y[n][idx:end] = arr[n]
        else:
            first = self.capacity - idx
            self._t[idx:] = t[:first]
            self._t[:end - self.capacity] = t[first:]
            for n in self.series:
                self._y[n][idx:] = arr[n][:first]
                self._y[n][:end - self.capacity] = arr[n][first:]
        self._n += k

    def redraw(self, window_s: float):
        if self._n == 0:
            return
        count = min(self._n, self.capacity)
        if self._n <= self.capacity:
            t = self._t[:count]
            get = lambda n: self._y[n][:count]
        else:
            idx = self._n % self.capacity
            t = np.concatenate((self._t[idx:], self._t[:idx]))
            get = lambda n: np.concatenate((self._y[n][idx:], self._y[n][:idx]))

        t_end = t[-1]
        t0 = t_end - window_s
        m = t >= t0
        tv = t[m]
        for n, c in self.curves.items():
            y = get(n)[m]
            if self._is_wrapped(n) and y.size > 1:
                # 相邻两点差超过半圈 = 发生了回绕，在该处断开
                y = y.copy()
                jump = np.abs(np.diff(y)) > 180.0
                y[1:][jump] = np.nan
            c.setData(tv, y)
        self.pw.setXRange(t0, t_end, padding=0)

    def clear(self):
        self._n = 0
        for c in self.curves.values():
            c.setData([], [])


class PlotPanel(QWidget):
    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._window_s = float(cfg.default_x_window)

        max_window = max(cfg.x_window_choices) if cfg.x_window_choices else 300
        # 缓冲只按最大时间窗算，不按实验时长——这是内存不随时间增长的关键
        capacity = int(max_window * cfg.telemetry_hz) + 16

        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(4)

        bar = QHBoxLayout()
        bar.addWidget(QLabel("时间窗"))
        self.window_combo = QComboBox()
        for w in cfg.x_window_choices:
            self.window_combo.addItem(f"{w} s", float(w))
        self.window_combo.addItem("Custom…", -1.0)
        i = self.window_combo.findData(float(cfg.default_x_window))
        if i >= 0:
            self.window_combo.setCurrentIndex(i)
        self.window_combo.currentIndexChanged.connect(self._on_window)
        bar.addWidget(self.window_combo)

        self.custom_window = QDoubleSpinBox()
        self.custom_window.setRange(0.5, 3600)
        self.custom_window.setValue(self._window_s)
        self.custom_window.setSuffix(" s")
        self.custom_window.setVisible(False)
        self.custom_window.valueChanged.connect(self._on_custom)
        bar.addWidget(self.custom_window)

        self.mem_label = QLabel("")
        self.mem_label.setStyleSheet("color:#888; font-size:11px;")
        bar.addWidget(self.mem_label)
        bar.addStretch(1)
        root.addLayout(bar)

        grid = QGridLayout()
        grid.setSpacing(4)
        self.plots = []
        specs = cfg.plots or _default_specs()
        for i, spec in enumerate(specs):
            p = SinglePlot(spec, capacity)
            grid.addWidget(p, i // 2, i % 2)
            self.plots.append(p)
        root.addLayout(grid, 1)

        nbytes = capacity * 8 * (1 + sum(len(p.series) for p in self.plots))
        self.mem_label.setText(
            f"绘图缓冲固定 {capacity:,} 点 / 曲线（约 {nbytes/1e6:.1f} MB），"
            "不随实验时长增长")

        self._t0 = None

    def _on_window(self, _):
        v = self.window_combo.currentData()
        if v is not None and v < 0:
            self.custom_window.setVisible(True)
            self._window_s = self.custom_window.value()
        else:
            self.custom_window.setVisible(False)
            self._window_s = float(v)

    def _on_custom(self, v):
        self._window_s = float(v)

    def append(self, arr: np.ndarray):
        # 用 system_time_ns 做时间轴，并以第一帧为原点，
        # 避免直接显示 1.7e18 这种没法读的绝对纳秒
        ts = arr["system_time_ns"].astype(np.float64) / 1e9
        if self._t0 is None:
            self._t0 = ts[0]
        t = ts - self._t0
        for p in self.plots:
            p.append(t, arr)

    def redraw(self):
        for p in self.plots:
            p.redraw(self._window_s)

    def clear(self):
        self._t0 = None
        for p in self.plots:
            p.clear()


def _default_specs():
    return [
        {"title": "位置 / deg", "series": ["target_position_deg", "motor_position_deg",
                                          "output_position_deg"], "auto_y": True},
        {"title": "转速 / rpm", "series": ["target_velocity_rpm", "motor_velocity_rpm",
                                          "output_velocity_rpm"], "auto_y": True},
        {"title": "电机电流 / A", "series": ["motor_current_A"], "auto_y": True},
        {"title": "力矩 / Nm", "series": ["target_torque_Nm", "actual_torque_Nm"],
         "auto_y": True},
    ]
