"""系统配置页：只读展示当前生效的配置，并说清每个数字的来源与可信度。

不提供在线编辑——配置改动需要重启 Backend 才生效，
做成可编辑反而会让人误以为改完就生效了。这里给的是"打开文件"。
"""
from __future__ import annotations

import subprocess
from pathlib import Path

from PySide6.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QTabWidget,
    QVBoxLayout, QWidget,
)

from .common import COLOR_OK, COLOR_WARN, ValueRow

CONFIG_FILES = ["app.yaml", "ethercat.yaml", "slave.yaml", "pdo.yaml",
                "scaling.yaml", "gui.yaml", "trajectory.yaml", "controller.yaml"]


class ConfigPanel(QWidget):
    def __init__(self, cfg, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)

        # ── 标定摘要 ────────────────────────────────────────────────
        cal = QGroupBox("标定参数（实测值，来源见 docs/ARCHITECTURE.md §17）")
        cg = QVBoxLayout(cal)
        cg.setSpacing(1)
        s = cfg.scaling
        rows = [
            ("电机侧编码器", f"{s.get('motor_encoder_counts_per_rev', 0):,.0f} counts/rev (2^17)"),
            ("输出侧编码器", f"{s.get('output_encoder_counts_per_rev', 0):,.0f} counts/rev (2^19)"),
            ("减速比", f"{s.get('gear_ratio', 0)} : 1  (铭牌标称 120)"),
            ("额定电流", f"{s.get('rated_current_mA', 0)/1000:.1f} A"),
            ("额定力矩", f"{s.get('rated_torque_mNm', 0)/1000:.1f} Nm"),
            ("0x60FF 单位", "0x6064 的 counts/s（velocity_gain_correction 现为 1.0，未应用任何实测速度标定）"),
            ("电机侧 100 rpm", "0x60FF ≈ 7207（实测 99.89 rpm）"),
            ("速度目标基准",
             ("电机侧" if s.get("target_velocity_is_motor_side", True) else "输出侧")
             + " rpm（target_velocity_is_motor_side）"),
        ]
        for k, v in rows:
            r = ValueRow(k)
            r.set_text(v)
            cg.addWidget(r)

        verified = cfg.encoder_verified
        warn = QLabel()
        if verified:
            warn.setText("✓ 编码器分辨率已通过物理转角验证")
            warn.setStyleSheet(f"color:{COLOR_OK};")
        else:
            warn.setText(
                "⚠ 输出侧分辨率 524288 沿用厂商 demo 的 ENCODER_RES，尚未用物理转角验证。\n"
                "两侧分辨率是绑定的：若输出侧实为 2^18，电机侧同步变 2^16，"
                "所有 rpm 数值翻倍、目标值变 3604。\n"
                "验证方法：输出法兰做记号 → CSV 模式 0x60FF=7207 跑 72.7 秒恒速 → "
                "看记号是否正好回到原位（回 1 圈证实 2^19，回 2 圈则是 2^18）。\n"
                "验证后请把 scaling.yaml 的 encoder_resolution_verified 改为 true。")
            warn.setStyleSheet(
                f"color:{COLOR_WARN}; background:#fff8e1; "
                "border:1px solid #ffe082; padding:6px;")
        warn.setWordWrap(True)
        cg.addWidget(warn)
        root.addWidget(cal)

        # ── 驱动器能力 ──────────────────────────────────────────────
        cap = QGroupBox("驱动器能力")
        pg = QVBoxLayout(cap)
        pg.setSpacing(1)
        for k, v in [
            ("从站", str(cfg.slave.get("name", "—"))),
            ("Vendor ID", f"0x{int(str(cfg.slave.get('vendor_id', '0')), 0):08X}"
                          if cfg.slave.get("vendor_id") else "—"),
            ("Product Code", f"0x{int(str(cfg.slave.get('product_code', '0')), 0):08X}"
                             if cfg.slave.get("product_code") else "—"),
            ("支持模式 0x6502", f"0x{int(str(cfg.slave.get('supported_modes_raw', '0')), 0):X} "
                                "→ pp / pv / tq / csp / csv / cst"),
            ("Homing", "不支持（0x6502 无 hm 位）" if not cfg.supports_homing else "支持"),
        ]:
            r = ValueRow(k)
            r.set_text(v)
            pg.addWidget(r)
        root.addWidget(cap)

        # ── 配置文件 ────────────────────────────────────────────────
        files = QGroupBox(f"配置文件（{cfg.config_dir}）")
        fg = QVBoxLayout(files)
        bar = QHBoxLayout()
        b_open = QPushButton("打开配置目录")
        b_open.clicked.connect(
            lambda: subprocess.Popen(["xdg-open", str(cfg.config_dir)]))
        bar.addWidget(b_open)
        bar.addWidget(QLabel("修改配置后需要重启 Backend 才生效"))
        bar.addStretch(1)
        fg.addLayout(bar)

        tabs = QTabWidget()
        for name in CONFIG_FILES:
            p = Path(cfg.config_dir) / name
            v = QPlainTextEdit()
            v.setReadOnly(True)
            from PySide6.QtGui import QFont
            f = QFont("monospace")
            f.setPointSize(9)
            v.setFont(f)
            try:
                v.setPlainText(p.read_text(encoding="utf-8"))
            except Exception as e:
                v.setPlainText(f"（无法读取 {p}: {e}）")
            tabs.addTab(v, name)
        fg.addWidget(tabs, 1)
        root.addWidget(files, 1)
