"""顶部状态栏：任务书第十三节要求始终可见的九项。"""
from __future__ import annotations

from PySide6.QtWidgets import QFrame, QHBoxLayout, QWidget

from .common import StatusLamp


class TopStatusBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameStyleHack()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 4, 8, 4)
        lay.setSpacing(14)

        self.master = StatusLamp("Master")
        self.ethercat = StatusLamp("EtherCAT")
        self.slave = StatusLamp("Slave")
        self.servo = StatusLamp("Servo")
        self.mode = StatusLamp("Mode")
        self.run = StatusLamp("Run")
        self.record = StatusLamp("Record")
        self.cycle = StatusLamp("Cycle")
        self.jitter = StatusLamp("Jitter")

        for w in (self.master, self.ethercat, self.slave, self.servo, self.mode,
                  self.run, self.record, self.cycle, self.jitter):
            lay.addWidget(w)
        lay.addStretch(1)

    def setFrameStyleHack(self):
        self.setStyleSheet("background:#f5f5f5; border-bottom:1px solid #ddd;")

    # ── 更新 ────────────────────────────────────────────────────────────
    def update_status(self, st):
        g = st.get

        online = g("slave_online", False)
        self.master.set_state("ok" if online else "idle",
                              "Running" if online else "Stopped")

        ec = g("ethercat", "UNKNOWN")
        self.ethercat.set_state("ok" if ec == "OP" else
                                ("warn" if ec in ("SAFEOP", "PREOP") else "error"), ec)

        op = g("slave_operational", False)
        cnt = g("slave_count", 0)
        self.slave.set_state("ok" if op else ("warn" if cnt else "error"),
                             f"{ec}" if cnt else "未检测到",
                             f"从站数量: {cnt}\n名称: {g('slave_name','')}")

        servo = g("servo", "Unknown")
        if servo == "Operation Enabled":
            self.servo.set_state("ok", servo)
        elif servo in ("Fault", "Fault Reaction Active"):
            code = g("error_code", 0)
            hint = ""
            if code == 0x730F:
                hint = "\n0x730F = 负载端编码器电池欠压。Fault Reset 清不掉，" \
                       "需用【重置负载端编码器】(向 0x2242 写 1)。"
            self.servo.set_state("error", f"{servo} (0x{code:04X})",
                                 f"错误码 0x{code:04X}{hint}")
        elif g("warning", False) or g("warning_code", 0):
            # Warning 不阻止运行，但顶栏必须让它可见，否则得切面板才发现
            self.servo.set_state(
                "warn", f"{servo} ⚠",
                f"状态字 bit7 Warning 置位\n警告码 0x{int(g('warning_code',0)):08X} (0x3B68)")
        else:
            self.servo.set_state("idle", servo)

        mode = g("mode", "None")
        matched = g("mode_matched", False)
        self.mode.set_state("ok" if matched else "warn",
                            mode if matched else f"{mode}(未生效)",
                            f"0x6060 写入 {mode}，0x6061 回读 {g('mode_display', '?')}")

        running = g("running", False)
        self.run.set_state("active" if running else "idle",
                           "Running" if running else "Stopped")

        # Recording 由 recording 事件单独更新
        cyc = g("cycle_us", 0)
        self.cycle.set_state("ok" if cyc else "idle",
                             f"{cyc/1000.0:.3f} ms" if cyc else "—")

        jmax = g("jitter_max_us", 0.0) or 0.0
        miss = g("deadline_miss", 0) or 0
        # 本机是 PREEMPT_DYNAMIC 内核，几十微秒抖动属正常，不该报红
        kind = "ok" if jmax < 200 else ("warn" if jmax < 1000 else "error")
        if miss > 0:
            kind = "warn" if miss < 10 else "error"
        self.jitter.set_state(
            kind, f"{jmax:.0f} µs",
            f"本周期 {g('jitter_us', 0):.1f} µs\n"
            f"均值 {g('jitter_mean_us', 0):.1f} µs\n"
            f"最大 {jmax:.1f} µs\n"
            f"错过周期 {miss} 次 / 共 {g('cycles', 0)} 周期\n"
            "（本机内核为 PREEMPT_DYNAMIC，非 PREEMPT_RT，"
            "几十微秒抖动属正常范围）")

    def update_recording(self, rec: dict):
        active = bool(rec.get("active"))
        dropped = int(rec.get("dropped", 0))
        if active:
            kind = "error" if dropped else "active"
            self.record.set_state(kind, f"ON ({rec.get('samples',0):,})",
                                  f"文件: {rec.get('file','')}\n"
                                  f"丢样: {dropped}\n"
                                  f"缓冲占用: {rec.get('buffer_usage',0)*100:.1f}%")
        else:
            self.record.set_state("idle", "Stopped")
