"""复审破坏 #1 的回归测试：experiment_panel 的 "_awaiting_line" 中间态（record_start
已发、还没收到 ack）在后端断连时必须复位，否则两个一键按钮永久禁用，只能重启 GUI。

跟 P0 终审抓到的 cia402_panel._running 断连不复位是同一类 bug——P2 这次新加的
"等待 ack" 中间态（I3 修复）重新引入了它，所以补一条专门测试防它再犯。

跑在 offscreen Qt 平台下，不需要真实显示环境。
"""
from __future__ import annotations

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gui"))

import pytest
from PySide6.QtWidgets import QApplication, QMessageBox

from widgets.experiment_panel import ExperimentPanel


@pytest.fixture(scope="module")
def app():
    existing = QApplication.instance()
    return existing or QApplication([])


@pytest.fixture(autouse=True)
def _no_modal_dialogs(monkeypatch):
    # QMessageBox.warning/.information 默认是阻塞的 exec()，headless 测试里
    # 会挂死；stub 掉，只记录调用参数。
    calls = []
    monkeypatch.setattr(QMessageBox, "warning",
                        staticmethod(lambda *a, **k: calls.append(("warning", a[1:]))))
    monkeypatch.setattr(QMessageBox, "information",
                        staticmethod(lambda *a, **k: calls.append(("information", a[1:]))))
    return calls


def test_on_disconnected_clears_awaiting_state_and_reenables_buttons(app):
    """record_start 已发、ack 还没回来时断连——_awaiting_line 必须复位，
    两个按钮必须恢复可用（不是永久卡在"等待记录确认…"）。"""
    p = ExperimentPanel()
    p._status = {"ethercat": "OP", "servo": "Operation Enabled", "mode_matched": True}
    # 手动模拟 _start() 发出 record_start 之后、ack 到达之前的中间态，
    # 不经过真实 StartDialog（避免弹模态框）。
    p._pending = {"sample_id": "A01", "out_dir": "/tmp"}
    p._awaiting_line = "B"
    p._set_waiting("B")

    assert p.btn_b.isEnabled() is False
    assert p.btn_a.isEnabled() is False

    p.on_disconnected()

    assert p._awaiting_line is None
    assert p._active_line is None
    assert p._pending is None
    assert p.btn_a.isEnabled() is True
    assert p.btn_b.isEnabled() is True
    assert p.btn_b.text() == p._label("B", "start")
    assert p.btn_a.text() == p._label("A", "start")


def test_on_disconnected_clears_active_running_state_too(app):
    """断连发生在"已经在跑"（不是等待态）时同样要回到干净的 idle。"""
    p = ExperimentPanel()
    p._pending = {"sample_id": "A01", "out_dir": "/tmp"}
    p._active_line = "A"
    p._set_running("A")

    p.on_disconnected()

    assert p._active_line is None
    assert p._awaiting_line is None
    assert p._pending is None
    assert p.btn_a.isEnabled() is True
    assert p.btn_b.isEnabled() is True


def test_record_start_failure_ack_is_handled_exactly_once(app, _no_modal_dialogs):
    """复审破坏 #2 的旁证：on_record_ack 对本面板发起的失败 ack 返回 True
    （main_window 据此跳过第二个通用失败弹框），对非本面板发起的 ack 返回 False。"""
    p = ExperimentPanel()
    p._pending = {"sample_id": "A01", "out_dir": "/tmp"}
    p._awaiting_line = "B"

    handled = p.on_record_ack(False, "磁盘空间不足")

    assert handled is True
    assert p._awaiting_line is None
    assert p._active_line is None
    assert [c for c in _no_modal_dialogs if c[0] == "warning"]

    # 不是本面板发起的（比如 record_panel 手动触发）：_awaiting_line 是 None，
    # 必须原样忽略，返回 False，不弹自己的框，也不吞掉 main_window 的通用弹框。
    _no_modal_dialogs.clear()
    handled2 = p.on_record_ack(False, "任意原因")
    assert handled2 is False
    assert not _no_modal_dialogs
