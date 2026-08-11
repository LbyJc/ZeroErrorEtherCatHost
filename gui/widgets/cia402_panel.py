"""CiA402 状态显示 + 语义化伺服控制（任务书第十四~十七节）。

用户永远不需要自己算控制字。这里给的是语义按钮，
控制字由 Backend 的状态机按 0x06 → 0x07 → 0x0F 逐级推进。
"""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QGridLayout, QGroupBox, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from .common import (
    COLOR_ERR, COLOR_IDLE, COLOR_OK, COLOR_WARN, CONTROLWORD_BITS,
    STATUSWORD_BITS, BitField, StatusLamp, ValueRow,
)

# 0x3B68 警告码。来源：eRob CANopen and EtherCAT 用户手册 8.2.51 节。
# 与错误码不同，警告不改变 PDS 状态机、不停机，但说明有事情不对。
WARNING_CODES = {
    0x0000: "无警告",
    0xFF00: "软速度误差警告 —— 实际速度跟不上指令。"
            "常见于目标速度阶跃变化，可调低 controller.yaml 的 "
            "velocity_rate_rpm_per_s，或放宽 0x3B61 的阈值。",
    0xFF01: "软位置误差警告 —— 实际位置跟不上指令。",
    0xFF02: "软堵转保护警告 —— 长时间低速大电流，检查负载是否卡住。",
}

# CiA402 八个状态 → 显示样式
STATE_STYLE = {
    "Operation Enabled": ("ok", "伺服已使能，可以运行"),
    "Switched On": ("active", "已上电，尚未使能运行"),
    "Ready to Switch On": ("idle", "就绪，等待 Switch On"),
    "Switch On Disabled": ("idle", "主电未使能"),
    "Quick Stop Active": ("warn", "快停生效中"),
    "Fault": ("error", "驱动器故障，需要 Fault Reset"),
    "Fault Reaction Active": ("error", "正在执行故障反应"),
    "Not Ready to Switch On": ("idle", "驱动器尚未就绪"),
}


class Cia402Panel(QWidget):
    command = Signal(dict)

    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self._error_code = 0
        self._running = False
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── 状态 ────────────────────────────────────────────────────
        box = QGroupBox("CiA402 状态")
        g = QVBoxLayout(box)
        self.lamp = StatusLamp("状态")
        g.addWidget(self.lamp)

        self.r_sw = ValueRow("Statusword")
        self.r_cw = ValueRow("Controlword")
        self.r_err = ValueRow("Error Code")
        self.r_warn = ValueRow("Warning Code")
        g.addWidget(self.r_sw)
        g.addWidget(BitFieldLabel("状态字位"))
        self.sw_bits = BitField(STATUSWORD_BITS)
        g.addWidget(self.sw_bits)
        g.addWidget(self.r_cw)
        g.addWidget(BitFieldLabel("控制字位"))
        self.cw_bits = BitField(CONTROLWORD_BITS)
        g.addWidget(self.cw_bits)
        g.addWidget(self.r_err)
        g.addWidget(self.r_warn)

        self.hint = QLabel("")
        self.hint.setWordWrap(True)
        self.hint.setStyleSheet(f"color:{COLOR_ERR}; font-size:11px;")
        g.addWidget(self.hint)
        root.addWidget(box)

        # ── 伺服控制 ────────────────────────────────────────────────
        ctl = QGroupBox("伺服控制")
        cg = QGridLayout(ctl)
        self.btn_enable = QPushButton("Servo Enable")
        self.btn_disable = QPushButton("Servo Disable")
        self.btn_fault = QPushButton("Fault Reset")
        self.btn_quick = QPushButton("Quick Stop")
        self.btn_homing = QPushButton("Homing")
        self.btn_reset_enc = QPushButton("重置负载端编码器")

        self.btn_enable.setStyleSheet("font-weight:bold;")
        cg.addWidget(self.btn_enable, 0, 0)
        cg.addWidget(self.btn_disable, 0, 1)
        cg.addWidget(self.btn_fault, 1, 0)
        cg.addWidget(self.btn_quick, 1, 1)
        cg.addWidget(self.btn_homing, 2, 0)
        cg.addWidget(self.btn_reset_enc, 2, 1)

        self.btn_enable.clicked.connect(lambda: self.command.emit({"cmd": "servo_enable"}))
        self.btn_disable.clicked.connect(self._on_servo_disable)
        self.btn_fault.clicked.connect(lambda: self.command.emit({"cmd": "fault_reset"}))
        self.btn_quick.clicked.connect(lambda: self.command.emit({"cmd": "quick_stop"}))
        self.btn_homing.clicked.connect(self._homing)
        self.btn_reset_enc.clicked.connect(self._reset_encoder)

        if not cfg.supports_homing:
            self.btn_homing.setToolTip(
                "本驱动器 0x6502 = 0x38D，不含 Homing 位，硬件层面不支持该模式。")
        root.addWidget(ctl)
        root.addStretch(1)

    # ── 动作 ────────────────────────────────────────────────────────────
    def _on_servo_disable(self):
        if self._running:
            box = QMessageBox(self)
            box.setIcon(QMessageBox.Warning)
            box.setWindowTitle("正在运行中")
            box.setText("关节正在运行。直接撤使能会在高速下抱闸。")
            box.setInformativeText(
                "手册 §7.1：制动器只许在 2.5 rpm（输出侧）以下承受动态制动，"
                "高转速下触发会对运动组件造成永久性损坏。\n\n"
                "建议改用【安全停机】：先软停到 2.5 rpm 以下再撤使能。")
            safe = box.addButton("安全停机", QMessageBox.AcceptRole)
            box.addButton("取消", QMessageBox.RejectRole)
            box.exec()
            if box.clickedButton() is safe:
                self.command.emit({"cmd": "safe_stop"})
            return
        self.command.emit({"cmd": "servo_disable"})

    def _homing(self):
        if not self.cfg.supports_homing:
            QMessageBox.information(
                self, "不支持 Homing",
                "本驱动器不支持 Homing 模式。\n\n"
                "0x6502 (Supported Drive Modes) = 0x38D，其中 Homing 位为 0，"
                "这是驱动器固件的能力限制，不是配置问题。\n\n"
                "如需回零，可在 CSP 模式下用【梯形轨迹】走到目标位置。")
            return
        self.command.emit({"cmd": "homing"})

    def _reset_encoder(self):
        r = QMessageBox.question(
            self, "重置负载端编码器",
            "此操作向 0x2242 写 1，用于清除 0x730F（负载端编码器电池欠压）。\n\n"
            "⚠ 会重置负载端编码器的多圈计数，绝对零位参考将改变。\n"
            "⚠ 需要主站处于停止状态（该操作使用阻塞式 SDO）。\n\n"
            "确认执行？")
        if r == QMessageBox.Yes:
            self.command.emit({"cmd": "reset_load_encoder"})

    # ── 刷新 ────────────────────────────────────────────────────────────
    def set_disconnected(self):
        """与 Backend 断开时调用。

        与 system_panel.set_disconnected() 同理：断开后不会再有 status 事件，
        _running 必须显式复位，否则运行中断连会让它冻结在 True——虽然失效方向
        是安全侧（多弹一次确认框，不会漏掉该弹的确认），但仍是过期状态，
        应该跟真实情况一致。
        """
        self._running = False

    def update_status(self, st):
        g = st.get
        self._running = g("running", False)
        state = g("servo", "Unknown")
        kind, desc = STATE_STYLE.get(state, ("idle", ""))
        self.lamp.set_state(kind, state, desc)

        sw = int(g("statusword", 0) or 0)
        cw = int(g("controlword", 0) or 0)
        self._error_code = int(g("error_code", 0) or 0)

        self.r_sw.set_text(f"0x{sw:04X}")
        self.r_cw.set_text(f"0x{cw:04X}")
        self.r_err.set_text(f"0x{self._error_code:04X}")
        self.r_err.set_color(COLOR_ERR if self._error_code else COLOR_IDLE)

        wc = int(g("warning_code", 0) or 0)
        warn_bit = bool(g("warning", False))
        self.r_warn.set_text(f"0x{wc:08X}" + ("  (bit7 已置位)" if warn_bit else ""))
        self.r_warn.set_color(COLOR_WARN if (wc or warn_bit) else COLOR_IDLE)

        self.sw_bits.set_value(sw)
        self.cw_bits.set_value(cw)

        # 已知故障码给出可执行的处置建议，而不是让人去翻手册
        if self._error_code == 0x730F:
            self.hint.setText(
                "0x730F = 负载端编码器多圈保持电池电压低于 3.05V。\n"
                "Fault Reset 清不掉（故障条件持续存在）。请先【停止主站】，"
                "再点【重置负载端编码器】(向 0x2242 写 1)；或装上 3.6V 多圈电池。")
        elif self._error_code == 0xA000:
            self.hint.setText(
                "0xA000 = 主站 deactivate 造成的通讯中断记录，不置 FAULT 位、"
                "不锁存，属正常现象，可忽略。")
        elif self._error_code:
            self.hint.setText(f"驱动器错误码 0x{self._error_code:04X}，请查厂商手册。")
        elif warn_bit or wc:
            # Warning 不阻止运行，但必须让人看见并知道是什么
            self.hint.setText(
                f"状态字 bit7 Warning 置位，0x3B68 = 0x{wc:08X}：\n"
                f"{WARNING_CODES.get(wc, '未知警告码，请查厂商手册 8.2.51 节')}\n"
                "错误码为 0，驱动器未进入故障态，运行不受阻。"
                "（向 0x3B63 写 1 可清除警告位）")
            self.hint.setStyleSheet(f"color:{COLOR_WARN}; font-size:11px;")
        else:
            self.hint.setText("")
            self.hint.setStyleSheet(f"color:{COLOR_ERR}; font-size:11px;")

        online = g("slave_online", False)
        ec_op = g("ethercat", "") == "OP"
        enabled = state == "Operation Enabled"
        faulted = state in ("Fault", "Fault Reaction Active")

        # EtherCAT 未进 OP 时禁止使能（任务书第十三节）
        self.btn_enable.setEnabled(online and ec_op and not enabled and not faulted)
        self.btn_disable.setEnabled(online and enabled)
        self.btn_fault.setEnabled(online and faulted)
        self.btn_quick.setEnabled(online and enabled)
        self.btn_reset_enc.setEnabled(not online)   # 阻塞 SDO 只能在非 Active 相位


class BitFieldLabel(QLabel):
    def __init__(self, text):
        super().__init__(text + "  (bit15 → bit0)")
        self.setStyleSheet("color:#888; font-size:10px;")
