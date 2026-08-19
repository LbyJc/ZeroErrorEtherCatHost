"""一键实验的开始弹框（收集元数据）与结束汇总弹框。

字段名严格对齐 Backend `record_start` 收的字段
（`experiment_naming.py`/`data_logger.hpp::RecordingMeta`）：
sample_id / life_hours / test_item / rep / load_percent_Tr / speed_rpm_target，
好让上层（Task 5）把 `get_values()` 的返回值直接透传给 record_start 命令。
"""
from __future__ import annotations

from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFileDialog, QFormLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QSpinBox, QVBoxLayout,
)

import experiment_naming as en


class StartDialog(QDialog):
    """点「持续运行(线A)」/「节点实验(线B)」时弹出，收集本次实验的元数据。"""

    def __init__(self, line: str, last_dir: str = "", parent=None,
                 prefill: dict | None = None):
        """prefill：实验配置（preset）的 record 字段预填，键名与 record_start 一致
        （test_item / load_percent_Tr / speed_rpm_target）。测试员只需确认目录。"""
        super().__init__(parent)
        self.line = line
        self.setWindowTitle("持续运行(线A)" if line == "A" else "节点实验(线B)")
        prefill = prefill or {}

        form = QFormLayout()
        self.sample_id = QLineEdit("A01")
        form.addRow("样机编号", self.sample_id)

        # 线A 收"寿命区间"标注（2026-08-14 需求）：摆臂持续运行跨节点，用户填
        # 0-5h 这种时间段（自由文本），拼进 h5 文件名（…_0-5h_<时间戳>.h5）
        # 并写进 record notes，让每段数据从文件名就能区分。
        if line == "A":
            self.life_label = QLineEdit()
            self.life_label.setPlaceholderText("例：0-5h（进文件名，可留空）")
            form.addRow("寿命区间", self.life_label)

        # 线A 恒速配置（摆臂持续运行）：转速框【可编辑且真实下发】——与线B 的
        # 仅标注字段相反。确认后面板会用它重发恒速轨迹覆盖配置默认值，
        # 数据标注 speed_rpm_target 同步用该值，标注不说谎。
        if line == "A" and "speed_rpm_target" in prefill:
            self.speed = QDoubleSpinBox()
            self.speed.setRange(-25, 25)     # 输出侧 rpm，负值=反转（额定 ~25）
            self.speed.setValue(float(prefill["speed_rpm_target"]))
            self.speed.setSuffix(" rpm")
            form.addRow("目标转速", self.speed)
            note_a = QLabel("此转速为输出侧、会实际下发并覆盖配置默认值；"
                            "负值=反转。启动按斜坡加速。")
            note_a.setWordWrap(True)
            note_a.setStyleSheet("color:#666; font-size:11px;")
            form.addRow(note_a)

        if line == "B":
            self.life_hours = QDoubleSpinBox()
            self.life_hours.setRange(0, 100000)
            self.life_hours.setSuffix(" h")

            self.test_item = QComboBox()
            self.test_item.addItems(sorted(en.VALID_TEST_ITEMS))
            if prefill.get("test_item") in en.VALID_TEST_ITEMS:
                self.test_item.setCurrentText(prefill["test_item"])
                self.test_item.setEnabled(False)

            self.rep = QSpinBox()
            self.rep.setRange(1, 99)
            self.rep.setValue(1)

            self.load_pct = QDoubleSpinBox()
            self.load_pct.setRange(0, 100)
            self.load_pct.setSuffix(" %Tr")
            if "load_percent_Tr" in prefill:
                self.load_pct.setValue(float(prefill["load_percent_Tr"]))
                self.load_pct.setEnabled(False)

            self.speed = QDoubleSpinBox()
            self.speed.setRange(0, 100)
            self.speed.setValue(float(prefill.get("speed_rpm_target", 5)))
            self.speed.setSuffix(" rpm")
            if "speed_rpm_target" in prefill:
                self.speed.setEnabled(False)

            form.addRow("节点小时", self.life_hours)
            form.addRow("测试项", self.test_item)
            form.addRow("重复号", self.rep)
            form.addRow("载荷百分比", self.load_pct)
            form.addRow("速度", self.speed)

            # 真机踩坑（2026-08-14）：这些字段只是写进数据文件的标注，实际转速
            # 由已加载配置的工况轨迹决定——之前可编辑，测试员改成 10rpm 以为
            # 能变速，实际还是按工况跑，且数据标注跟着说谎。加载了配置就锁死
            # 由配置决定的三项，只留真正因节点而异的（节点小时/重复号）可编辑。
            note = QLabel("测试项/载荷/速度由所选配置决定（仅数据标注，不控制运行）；"
                          "要换转速请回列表选对应配置。" if prefill else
                          "注意：本弹框只是数据标注，不下发运行参数；"
                          "实际轨迹以模式面板已应用的为准。")
            note.setWordWrap(True)
            note.setStyleSheet("color:#666; font-size:11px;")
            form.addRow(note)

        # 结束后是否自动导出 A.1 CSV（2026-08-14 需求：CSV 比 h5 大得多，
        # 长时间采集导出既慢又占盘，改为按次勾选）。线B 默认勾（节点测试
        # 数据小、分析要 CSV），线A 默认不勾（持续运行数据大，h5 始终保存）。
        self.export_csv = QCheckBox(
            "结束后自动导出 CSV（A.1 格式）" +
            ("——持续运行数据量大，非必要不勾；h5 始终保存" if line == "A" else ""))
        self.export_csv.setChecked(line == "B")
        form.addRow(self.export_csv)

        # 保存目录
        drow = QHBoxLayout()
        self.out_dir = QLineEdit(last_dir)
        btn = QPushButton("浏览…")
        btn.clicked.connect(self._pick)
        drow.addWidget(self.out_dir, 1)
        drow.addWidget(btn)
        form.addRow("保存目录", drow)

        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)

        lay = QVBoxLayout(self)
        lay.addLayout(form)
        lay.addWidget(bb)

    def _pick(self):
        d = QFileDialog.getExistingDirectory(self, "选择保存目录", self.out_dir.text())
        if d:
            self.out_dir.setText(d)

    def get_values(self):
        """弹框并返回填写结果；取消返回 None。

        返回字典字段名与 Backend record_start 一致（export_csv/life_label
        除外，它们是 GUI 侧决策）。两线都含 sample_id/out_dir/line/export_csv；
        线 A 另加 life_label（恒速配置再加 speed_rpm_target=实际下发转速）；
        线 B 另加 life_hours/test_item/rep/load_percent_Tr/speed_rpm_target，
        供上层直接透传。
        """
        if self.exec() != QDialog.Accepted:
            return None
        v = {
            "sample_id": self.sample_id.text().strip(),
            "out_dir": self.out_dir.text().strip(),
            "line": self.line,
            "export_csv": self.export_csv.isChecked(),
        }
        if self.line == "A":
            v["life_label"] = self.life_label.text().strip()
        if self.line == "A" and hasattr(self, "speed"):
            v["speed_rpm_target"] = self.speed.value()
        if self.line == "B":
            v.update(
                life_hours=self.life_hours.value(),
                test_item=self.test_item.currentText(),
                rep=self.rep.value(),
                load_percent_Tr=self.load_pct.value(),
                speed_rpm_target=self.speed.value(),
            )
        return v


class SummaryDialog(QDialog):
    """实验结束时弹出，只读展示存哪、多少样本、CSV 路径。"""

    def __init__(self, info: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle(info.get("title", "实验结束"))
        lay = QVBoxLayout(self)
        if info.get("title"):
            head = QLabel(info["title"])
            head.setStyleSheet("font-weight:bold; font-size:14px;")
            lay.addWidget(head)
        lay.addWidget(QLabel(f"HDF5: {info.get('h5_path', '?')}"))
        lay.addWidget(QLabel(f"样本数: {info.get('samples', '?')}"))
        if info.get("csv_path"):
            lay.addWidget(QLabel(f"CSV: {info['csv_path']}"))
        if info.get("error"):
            lab = QLabel(f"⚠ {info['error']}")
            lab.setStyleSheet("color:#c33;")
            lay.addWidget(lab)
        bb = QDialogButtonBox(QDialogButtonBox.Ok)
        bb.accepted.connect(self.accept)
        lay.addWidget(bb)
