"""一键实验的开始弹框（收集元数据）与结束汇总弹框。

字段名严格对齐 Backend `record_start` 收的字段
（`experiment_naming.py`/`data_logger.hpp::RecordingMeta`）：
sample_id / life_hours / test_item / rep / load_percent_Tr / speed_rpm_target，
好让上层（Task 5）把 `get_values()` 的返回值直接透传给 record_start 命令。
"""
from __future__ import annotations

from PySide6.QtWidgets import (
    QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFileDialog,
    QFormLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QSpinBox,
    QVBoxLayout,
)

import experiment_naming as en


class StartDialog(QDialog):
    """点「持续运行(线A)」/「节点实验(线B)」时弹出，收集本次实验的元数据。"""

    def __init__(self, line: str, last_dir: str = "", parent=None):
        super().__init__(parent)
        self.line = line
        self.setWindowTitle("持续运行(线A)" if line == "A" else "节点实验(线B)")

        form = QFormLayout()
        self.sample_id = QLineEdit("A01")
        form.addRow("样机编号", self.sample_id)

        if line == "B":
            self.life_hours = QDoubleSpinBox()
            self.life_hours.setRange(0, 100000)
            self.life_hours.setSuffix(" h")

            self.test_item = QComboBox()
            self.test_item.addItems(sorted(en.VALID_TEST_ITEMS))

            self.rep = QSpinBox()
            self.rep.setRange(1, 99)
            self.rep.setValue(1)

            self.load_pct = QDoubleSpinBox()
            self.load_pct.setRange(0, 100)
            self.load_pct.setSuffix(" %Tr")

            self.speed = QDoubleSpinBox()
            self.speed.setRange(0, 100)
            self.speed.setValue(5)
            self.speed.setSuffix(" rpm")

            form.addRow("节点小时", self.life_hours)
            form.addRow("测试项", self.test_item)
            form.addRow("重复号", self.rep)
            form.addRow("载荷百分比", self.load_pct)
            form.addRow("速度", self.speed)

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

        返回字典字段名与 Backend record_start 一致，线 A 只含
        sample_id/out_dir/line；线 B 另加 life_hours/test_item/rep/
        load_percent_Tr/speed_rpm_target，供上层直接透传。
        """
        if self.exec() != QDialog.Accepted:
            return None
        v = {
            "sample_id": self.sample_id.text().strip(),
            "out_dir": self.out_dir.text().strip(),
            "line": self.line,
        }
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
        self.setWindowTitle("实验结束")
        lay = QVBoxLayout(self)
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
