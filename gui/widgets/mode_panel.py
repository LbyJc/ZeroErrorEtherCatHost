"""中间：运行模式 + 轨迹 + 目标 + 控制器 + 开始/停止运行。

任务书第二十三节：用户只输入真实物理单位，单位随模式自动切换，
Raw 值的换算由 Backend 负责，GUI 不做也不显示 Raw。
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QButtonGroup, QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QLineEdit, QPushButton, QRadioButton,
    QStackedWidget, QVBoxLayout, QWidget,
)

from .common import COLOR_ERR, COLOR_OK, COLOR_WARN

MODES = ["PP", "PV", "PT", "Homing", "CSP", "CSV", "CST"]
# 各模式下 Target 的物理单位
MODE_UNIT = {
    "PP": ("deg", "输出侧角度"), "CSP": ("deg", "输出侧角度"),
    "PV": ("rpm", "电机侧转速"), "CSV": ("rpm", "电机侧转速"),
    "PT": ("Nm", "力矩"), "CST": ("Nm", "力矩"),
    "Homing": ("—", ""),
}

TRAJ_TYPES = [
    ("constant", "Constant 常值"),
    ("sine", "Sine 正弦"),
    ("ramp", "Ramp 斜坡"),
    ("triangle", "Triangle 三角"),
    ("trapezoidal", "Trapezoidal 梯形(位置)"),
    ("csv", "Custom CSV 文件"),
]


def _spin(lo, hi, dec, val, step=1.0):
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(dec)
    s.setValue(val)
    s.setSingleStep(step)
    s.setKeyboardTracking(False)
    return s


class ModePanel(QWidget):
    command = Signal(dict)

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._mode = "CSV"
        self._running = False
        self._can_run = False

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── 运行模式 ────────────────────────────────────────────────
        mbox = QGroupBox("运行模式 (0x6060 / 0x6061)")
        mg = QGridLayout(mbox)
        self.mode_group = QButtonGroup(self)
        self._mode_buttons = {}
        for i, m in enumerate(MODES):
            rb = QRadioButton(m)
            if m == "Homing" and not cfg.supports_homing:
                rb.setEnabled(False)
                rb.setToolTip("本驱动器 0x6502=0x38D，不支持 Homing")
            if m == "CSV":
                rb.setChecked(True)
            self.mode_group.addButton(rb, i)
            self._mode_buttons[m] = rb
            mg.addWidget(rb, i // 4, i % 4)
        self.mode_group.buttonClicked.connect(self._on_mode)

        self.mode_check = QLabel("0x6060 = —   0x6061 = —")
        self.mode_check.setStyleSheet("color:#666; font-size:11px;")
        mg.addWidget(self.mode_check, 2, 0, 1, 4)
        root.addWidget(mbox)

        # ── 轨迹 ────────────────────────────────────────────────────
        tbox = QGroupBox("轨迹发生器")
        tg = QVBoxLayout(tbox)
        self.traj_combo = QComboBox()
        for k, label in TRAJ_TYPES:
            self.traj_combo.addItem(label, k)
        self.traj_combo.currentIndexChanged.connect(self._on_traj_type)
        tg.addWidget(self.traj_combo)

        self.traj_stack = QStackedWidget()
        self._build_traj_pages()
        tg.addWidget(self.traj_stack)

        self.btn_apply_traj = QPushButton("应用轨迹设置")
        self.btn_apply_traj.clicked.connect(self._apply_traj)
        tg.addWidget(self.btn_apply_traj)
        root.addWidget(tbox)

        # ── 目标值 ──────────────────────────────────────────────────
        tarbox = QGroupBox("目标值")
        tar = QHBoxLayout(tarbox)
        self.target = _spin(-100000, 100000, 3, 100.0, 1.0)
        self.target.valueChanged.connect(
            lambda v: self.command.emit({"cmd": "set_target", "value": v}))
        self.target_unit = QLabel("rpm")
        self.target_unit.setStyleSheet("font-weight:bold;")
        self.target_hint = QLabel("电机侧转速")
        self.target_hint.setStyleSheet("color:#666;")
        tar.addWidget(QLabel("Target"))
        tar.addWidget(self.target, 1)
        tar.addWidget(self.target_unit)
        tar.addWidget(self.target_hint)
        root.addWidget(tarbox)

        # ── 控制器 ──────────────────────────────────────────────────
        cbox = QGroupBox("控制器")
        cg = QVBoxLayout(cbox)
        self.ctrl_combo = QComboBox()
        self.ctrl_combo.currentIndexChanged.connect(self._on_controller)
        cg.addWidget(self.ctrl_combo)
        self.ctrl_hint = QLabel("参数在【Parameter Tuning】页调整")
        self.ctrl_hint.setStyleSheet("color:#666; font-size:11px;")
        cg.addWidget(self.ctrl_hint)
        root.addWidget(cbox)

        # ── 运行 ────────────────────────────────────────────────────
        rbox = QGroupBox("运行")
        rg = QVBoxLayout(rbox)
        row = QHBoxLayout()
        self.btn_run = QPushButton("开始运行")
        self.btn_stop = QPushButton("停止运行")
        self.btn_safe_stop = QPushButton("安全停机")
        self.btn_run.setMinimumHeight(40)
        self.btn_stop.setMinimumHeight(40)
        self.btn_safe_stop.setMinimumHeight(40)
        self.btn_run.setStyleSheet("font-weight:bold; font-size:14px;")
        self.btn_stop.setStyleSheet("font-size:14px;")
        self.btn_safe_stop.setStyleSheet("font-size:14px; color:#b71c1c; font-weight:bold;")
        self.btn_safe_stop.setToolTip(
            "软停至 2.5 rpm 以下后自动撤使能。\n"
            "手册 §7.1：高于该转速抱闸会永久损坏运动组件。")
        row.addWidget(self.btn_run)
        row.addWidget(self.btn_stop)
        row.addWidget(self.btn_safe_stop)
        rg.addLayout(row)
        self.run_hint = QLabel("")
        self.run_hint.setWordWrap(True)
        self.run_hint.setStyleSheet("color:#888; font-size:11px;")
        rg.addWidget(self.run_hint)
        self.btn_run.clicked.connect(lambda: self.command.emit({"cmd": "start_run"}))
        self.btn_stop.clicked.connect(lambda: self.command.emit({"cmd": "stop_run"}))
        self.btn_safe_stop.clicked.connect(lambda: self.command.emit({"cmd": "safe_stop"}))
        root.addWidget(rbox)

        root.addStretch(1)
        self._sync_units()

    # ── 轨迹参数页 ──────────────────────────────────────────────────────
    def _build_traj_pages(self):
        t = self.cfg.trajectory

        # constant
        p = QWidget(); f = QFormLayout(p)
        f.addRow(QLabel("使用上方【目标值】作为常值，可在运行中实时修改。"))
        self.traj_stack.addWidget(p)

        # sine
        p = QWidget(); f = QFormLayout(p)
        s = t.get("sine", {})
        self.sin_offset = _spin(-100000, 100000, 3, float(s.get("offset", 0)))
        self.sin_amp = _spin(0, 100000, 3, float(s.get("amplitude", 10)))
        self.sin_freq = _spin(0.001, 100, 3, float(s.get("frequency_hz", 0.5)), 0.1)
        self.sin_phase = _spin(-360, 360, 1, float(s.get("phase_deg", 0)))
        self.sin_dur = _spin(-1, 100000, 1, float(s.get("duration_s", 20)))
        f.addRow("Offset", self.sin_offset)
        f.addRow("Amplitude", self.sin_amp)
        f.addRow("Frequency (Hz)", self.sin_freq)
        f.addRow("Phase (deg)", self.sin_phase)
        f.addRow("Duration (s, -1=无限)", self.sin_dur)
        self.traj_stack.addWidget(p)

        # ramp
        p = QWidget(); f = QFormLayout(p)
        r = t.get("ramp", {})
        self.ramp_i = _spin(-100000, 100000, 3, float(r.get("initial", 0)))
        self.ramp_f = _spin(-100000, 100000, 3, float(r.get("final", 100)))
        self.ramp_d = _spin(0.01, 100000, 2, float(r.get("duration_s", 5)))
        f.addRow("Initial", self.ramp_i)
        f.addRow("Final", self.ramp_f)
        f.addRow("Duration (s)", self.ramp_d)
        self.traj_stack.addWidget(p)

        # triangle
        p = QWidget(); f = QFormLayout(p)
        tr = t.get("triangle", {})
        self.tri_offset = _spin(-100000, 100000, 3, float(tr.get("offset", 0)))
        self.tri_amp = _spin(0, 100000, 3, float(tr.get("amplitude", 10)))
        self.tri_freq = _spin(0.001, 100, 3, float(tr.get("frequency_hz", 0.2)), 0.1)
        f.addRow("Offset", self.tri_offset)
        f.addRow("Amplitude", self.tri_amp)
        f.addRow("Frequency (Hz)", self.tri_freq)
        self.traj_stack.addWidget(p)

        # trapezoidal
        p = QWidget(); f = QFormLayout(p)
        z = t.get("trapezoidal", {})
        self.tz_target = _spin(-100000, 100000, 2, float(z.get("target_position_deg", 90)))
        self.tz_vmax = _spin(0.01, 100000, 2, float(z.get("max_velocity_rpm", 50)))
        self.tz_acc = _spin(0.01, 100000, 2, float(z.get("acceleration", 200)))
        self.tz_dec = _spin(0.01, 100000, 2, float(z.get("deceleration", 200)))
        f.addRow("Target Position (deg)", self.tz_target)
        f.addRow("Max Velocity (rpm,输出侧)", self.tz_vmax)
        f.addRow("Acceleration (deg/s²)", self.tz_acc)
        f.addRow("Deceleration (deg/s²)", self.tz_dec)
        self.traj_stack.addWidget(p)

        # csv
        p = QWidget(); f = QFormLayout(p)
        row = QHBoxLayout()
        self.csv_path = QLineEdit()
        self.csv_path.setPlaceholderText("time,target  或  time,target_position,target_velocity,target_torque")
        b = QPushButton("浏览…")
        b.clicked.connect(self._pick_csv)
        row.addWidget(self.csv_path, 1)
        row.addWidget(b)
        w = QWidget(); w.setLayout(row)
        f.addRow("轨迹文件", w)
        self.traj_stack.addWidget(p)

    def _pick_csv(self):
        fn, _ = QFileDialog.getOpenFileName(self, "选择轨迹文件", str(self.cfg.root),
                                            "CSV 文件 (*.csv);;所有文件 (*)")
        if fn:
            self.csv_path.setText(fn)

    def _on_traj_type(self, idx):
        self.traj_stack.setCurrentIndex(idx)

    def _apply_traj(self):
        k = self.traj_combo.currentData()
        msg = {"cmd": "set_trajectory", "type": k}
        if k == "constant":
            msg["value"] = self.target.value()
        elif k == "sine":
            msg.update(offset=self.sin_offset.value(), amplitude=self.sin_amp.value(),
                       frequency_hz=self.sin_freq.value(), phase_deg=self.sin_phase.value(),
                       duration_s=self.sin_dur.value())
        elif k == "ramp":
            msg.update(ramp_initial=self.ramp_i.value(), ramp_final=self.ramp_f.value(),
                       ramp_duration_s=self.ramp_d.value())
        elif k == "triangle":
            msg.update(offset=self.tri_offset.value(), amplitude=self.tri_amp.value(),
                       frequency_hz=self.tri_freq.value())
        elif k == "trapezoidal":
            msg.update(trapz_target_deg=self.tz_target.value(),
                       trapz_vmax_rpm=self.tz_vmax.value(),
                       trapz_acc=self.tz_acc.value(), trapz_dec=self.tz_dec.value())
        elif k == "csv":
            msg["csv_path"] = self.csv_path.text()
        self.command.emit(msg)

    # ── 模式 / 控制器 ───────────────────────────────────────────────────
    def _on_mode(self, btn):
        self._mode = btn.text()
        self._sync_units()
        self.command.emit({"cmd": "set_mode", "mode": self._mode})

    def _sync_units(self):
        unit, hint = MODE_UNIT.get(self._mode, ("—", ""))
        self.target_unit.setText(unit)
        self.target_hint.setText(hint)
        if unit == "rpm":
            self.target.setRange(-self.cfg.scaling.get("limits", {}).get(
                "motor_velocity_rpm_max", 3000),
                self.cfg.scaling.get("limits", {}).get("motor_velocity_rpm_max", 3000))
        elif unit == "Nm":
            lim = self.cfg.scaling.get("limits", {}).get("torque_Nm_max", 20)
            self.target.setRange(-lim, lim)
        else:
            self.target.setRange(-1e6, 1e6)

    def push_initial(self):
        """连上 Backend 后把界面上显示的初值真正下发一次。

        QDoubleSpinBox 初始化时不会触发 valueChanged，所以如果不显式推一次，
        Target 框上写着 100 而 Backend 里其实是 0——
        界面显示与实际下发不一致，调伺服时这种不一致是会出事的。
        """
        self.command.emit({"cmd": "set_mode", "mode": self._mode})
        self._apply_traj()
        self.command.emit({"cmd": "set_target", "value": self.target.value()})

    def set_controllers(self, items: list[dict]):
        cur = self.ctrl_combo.currentData()
        self.ctrl_combo.blockSignals(True)
        self.ctrl_combo.clear()
        for c in items:
            self.ctrl_combo.addItem(c.get("name", c.get("id")), c.get("id"))
        if cur:
            i = self.ctrl_combo.findData(cur)
            if i >= 0:
                self.ctrl_combo.setCurrentIndex(i)
        self.ctrl_combo.blockSignals(False)

    def _on_controller(self, _):
        cid = self.ctrl_combo.currentData()
        if cid:
            self.command.emit({"cmd": "set_controller", "id": cid})

    # ── 刷新 ────────────────────────────────────────────────────────────
    def update_status(self, st):
        g = st.get
        mode = g("mode", "CSV")
        if mode != self._mode:
            self._mode = mode
            if mode in self._mode_buttons:
                self._mode_buttons[mode].setChecked(True)
            self._sync_units()

        md = g("mode_display", 0)
        matched = g("mode_matched", False)
        self.mode_check.setText(f"0x6060 = {mode}   0x6061 = {md}   "
                                f"{'✓ 一致' if matched else '✗ 未生效'}")
        self.mode_check.setStyleSheet(
            f"color:{COLOR_OK if matched else COLOR_WARN}; font-size:11px;")

        self._running = g("running", False)
        ec_op = g("ethercat", "") == "OP"
        enabled = g("servo", "") == "Operation Enabled"
        faulted = g("servo", "") in ("Fault", "Fault Reaction Active")

        # 任务书第四十节：任何一条不满足，【开始运行】必须禁用
        self._can_run = ec_op and enabled and matched and not faulted
        self.btn_run.setEnabled(self._can_run and not self._running)
        self.btn_stop.setEnabled(self._running)

        # 运行中禁止切模式/控制器/轨迹（任务书第四十一节）
        for rb in self._mode_buttons.values():
            if rb.text() != "Homing" or self.cfg.supports_homing:
                rb.setEnabled(not self._running)
        self.ctrl_combo.setEnabled(not self._running)
        self.btn_apply_traj.setEnabled(not self._running)

        if self._running:
            self.run_hint.setText(f"运行中 {g('run_time_s', 0):.1f} s")
            self.run_hint.setStyleSheet(f"color:{COLOR_OK}; font-size:11px;")
        elif self._can_run:
            self.run_hint.setText("条件满足，可以开始运行")
            self.run_hint.setStyleSheet(f"color:{COLOR_OK}; font-size:11px;")
        else:
            missing = []
            if not ec_op:
                missing.append(f"EtherCAT 未进 OP（当前 {g('ethercat','?')}）")
            if not enabled:
                missing.append(f"伺服未使能（当前 {g('servo','?')}）")
            if not matched:
                missing.append("运行模式未生效（0x6061 ≠ 0x6060）")
            if faulted:
                missing.append("驱动器处于故障状态")
            self.run_hint.setText("不能运行：" + "；".join(missing))
            self.run_hint.setStyleSheet(f"color:{COLOR_ERR}; font-size:11px;")
