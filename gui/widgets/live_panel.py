"""右侧：实时数值（任务书第二十四节）。

位置同时给出 wrapped [0,360) 和 unwrapped 多圈值——
只显示 wrapped 会在连续旋转实验里丢掉圈数信息，
只显示 unwrapped 又不便于看当前姿态，两个都要。
"""
from __future__ import annotations

from PySide6.QtWidgets import QGroupBox, QLabel, QVBoxLayout, QWidget

from .common import COLOR_ERR, COLOR_WARN, ValueRow


class LivePanel(QWidget):
    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── 电机侧 ──────────────────────────────────────────────────
        m = QGroupBox("电机侧 (Motor Side)")
        mg = QVBoxLayout(m)
        mg.setSpacing(1)
        self.m_pos = ValueRow("位置", "deg")
        self.m_pos_uw = ValueRow("多圈位置", "deg")
        self.m_vel = ValueRow("转速", "rpm")
        self.m_cur = ValueRow("电流", "A")
        self.m_raw = ValueRow("原始计数", "cnt")
        for r in (self.m_pos, self.m_pos_uw, self.m_vel, self.m_cur, self.m_raw):
            mg.addWidget(r)
        root.addWidget(m)

        # ── 输出侧 ──────────────────────────────────────────────────
        o = QGroupBox("输出侧 (Output Side)")
        og = QVBoxLayout(o)
        og.setSpacing(1)
        self.o_pos = ValueRow("位置", "deg")
        self.o_pos_uw = ValueRow("多圈位置", "deg")
        self.o_vel = ValueRow("转速", "rpm")
        self.o_raw = ValueRow("原始计数", "cnt")
        for r in (self.o_pos, self.o_pos_uw, self.o_vel, self.o_raw):
            og.addWidget(r)
        root.addWidget(o)

        # ── 力矩与误差 ──────────────────────────────────────────────
        t = QGroupBox("力矩 / 目标 / 误差")
        tg = QVBoxLayout(t)
        tg.setSpacing(1)
        self.trq = ValueRow("实际力矩", "Nm")
        self.trq_motor = ValueRow("电机轴力矩", "Nm")
        self.trq_est = ValueRow("估计力矩(厂商)", "Nm")
        self.trq_tgt = ValueRow("目标力矩", "Nm")
        self.pos_tgt = ValueRow("目标位置", "deg")
        self.vel_tgt = ValueRow("目标转速", "rpm")
        self.pos_err = ValueRow("位置误差", "deg")
        self.vel_err = ValueRow("速度误差", "rpm")
        for r in (self.trq, self.trq_motor, self.trq_est, self.trq_tgt,
                  self.pos_tgt, self.vel_tgt, self.pos_err, self.vel_err):
            tg.addWidget(r)
        root.addWidget(t)

        # 标定可信度提示：不把未验证的数字装成确定值
        if not cfg.encoder_verified:
            warn = QLabel(
                "⚠ 输出侧编码器分辨率未经物理转角验证。\n"
                "若实为 2^18，上面所有 rpm 数值需翻倍。\n"
                "验证方法见 config/scaling.yaml 注释。")
            warn.setWordWrap(True)
            warn.setStyleSheet(
                f"color:{COLOR_WARN}; font-size:11px; "
                "background:#fff8e1; border:1px solid #ffe082; padding:4px;")
            root.addWidget(warn)

        root.addStretch(1)

    def update_sample(self, s):
        """s 是 numpy 结构化数组的最后一行。"""
        self.m_pos.set_value(float(s["motor_position_deg"]))
        self.m_pos_uw.set_value(float(s["motor_position_unwrapped_deg"]), "{:.2f}")
        self.m_vel.set_value(float(s["motor_velocity_rpm"]), "{:.2f}")
        self.m_cur.set_value(float(s["motor_current_A"]))
        self.m_raw.set_text(f"{int(s['motor_position_raw']):,}")

        self.o_pos.set_value(float(s["output_position_deg"]))
        self.o_pos_uw.set_value(float(s["output_position_unwrapped_deg"]), "{:.2f}")
        self.o_vel.set_value(float(s["output_velocity_rpm"]), "{:.4f}")
        self.o_raw.set_text(f"{int(s['output_position_raw']):,}")

        self.trq.set_value(float(s["actual_torque_Nm"]))
        self.trq_motor.set_value(float(s["motor_torque_Nm"]), "{:.4f}")
        self.trq_est.set_value(float(s["torque_est_Nm"]))
        self.trq_tgt.set_value(float(s["target_torque_Nm"]))
        self.pos_tgt.set_value(float(s["target_position_deg"]), "{:.2f}")
        self.vel_tgt.set_value(float(s["target_velocity_rpm"]), "{:.2f}")

        pe = float(s["position_error_deg"])
        ve = float(s["velocity_error_rpm"])
        self.pos_err.set_value(pe, "{:.3f}")
        self.vel_err.set_value(ve, "{:.2f}")
