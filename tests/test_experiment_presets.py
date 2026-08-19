#!/usr/bin/env python3
"""experiment_presets 加载逻辑 + 一键实验面板配置列表的离屏测试。

不碰真实主站、不使能——只测软件：
  [1] 纯逻辑：扫描仓库自带的 presets，校验字段/路径解析/坏文件不拖垮列表
  [2] 离屏 Qt：面板列表填充、加载配置发出的 IPC 命令、按钮/配置匹配、
      record_start 的 test_name 与配置名同步
运行：QT_QPA_PLATFORM=offscreen python tests/test_experiment_presets.py
"""
from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "gui"))

failures = []


def check(cond, msg):
    print(f"  {'[ OK ]' if cond else '[FAIL]'} {msg}")
    if not cond:
        failures.append(msg)


def test_loader():
    import experiment_presets as ep

    print("\n[1] 纯逻辑：扫描仓库 presets")
    presets, errors = ep.scan_presets(ROOT / "experiments" / "presets")
    check(len(presets) == 14, f"14 个配置全部加载成功（实际 {len(presets)}，错误 {errors}）")
    check(not errors, "没有坏文件")
    kinds = [p.kind for p in presets]
    check(kinds.count("continuous") == 1, "恰好 1 个持续运行配置")
    check(kinds.count("node") == 13,
          "13 个节点实验配置（4 摩擦10rpm + 6 TE + 3 RP；5rpm 摩擦已并入 TE）")

    # ── 重复定位精度 RP（2026-08-18 第四轮设计，41~43）──
    rps = [p for p in presets if p.record.get("test_item") == "RP"]
    check(len(rps) == 3, f"3 个 RP 配置（0/10/30Tr，实际 {len(rps)}）")
    rp = next(p for p in presets if p.name == "重复定位精度_空载0Tr")
    check(rp.line == "B" and rp.mode == "CSP", "RP → 线B、CSP 位置模式")
    check(rp.record == {"test_item": "RP", "load_percent_Tr": 0,
                        "speed_rpm_target": 2.5},
          f"RP record 预填正确（逼近 15°/s = 2.5 rpm，{rp.record}）")
    rp_csv = Path(rp.trajectory["csv_path"])
    check(rp_csv.is_file(), "RP 工况文件存在")
    lines = [ln for ln in rp_csv.read_text(encoding="utf-8").splitlines()
             if ln and not ln.startswith("#")]
    check(lines[0] == "time,target" and lines[1].split(",") == ["0.0000", "0"],
          f"RP 工况是两列位置格式且首点 0（就地起测靠后端起点平移，{lines[:2]}）")
    check("参考位" in rp.description and "预跑" in rp.description,
          "RP 说明含参考位要求与预跑提示（测试员唯一说明）")
    cont = next(p for p in presets if p.kind == "continuous")
    check(cont.line == "A" and cont.mode == "CSV", "持续运行 → 线A、CSV（恒速回转）")
    check(cont.trajectory["type"] == "constant" and cont.trajectory["value"] == 25
          and cont.trajectory["duration_s"] == -1,
          "寿命恒速：输出侧 25 rpm 无限时长（2026-08-18 由 20 改 25）")
    check(cont.record.get("speed_rpm_target") == 25,
          "record 预填默认转速 25（供开始弹框覆盖）")
    node1 = next(p for p in presets if p.name == "周期测试1_电流摩擦_空载0Tr_10rpm正转")
    check(node1.line == "B" and node1.mode == "CSV", "节点1 → 线B、CSV")
    check(Path(node1.trajectory["csv_path"]).is_file(), "csv_path 解析为存在的绝对路径")
    check(node1.record == {"test_item": "current", "load_percent_Tr": 0,
                           "speed_rpm_target": 10}, "节点1 record 预填字段正确")
    check(node1.merged_from == [] and node1.display_name == node1.name,
          "未合并配置：merged_from 空、显示名 = 原名")
    merged = next(p for p in presets if p.name == "周期测试2_传动误差_空载0Tr_5rpm正转")
    check(merged.merged_from == ["周期测试1_电流摩擦_空载0Tr_5rpm正转"],
          f"合并配置 merged_from 正确（{merged.merged_from}）")
    check(merged.display_name ==
          "周期测试2_传动误差_空载0Tr_5rpm正转（含 周期测试1_电流摩擦_空载0Tr_5rpm正转）",
          f"显示名带括号（{merged.display_name}）")
    check(not any(p.name.endswith("5rpm正转") and "摩擦" in p.name for p in presets),
          "5rpm 摩擦配置已删除（由合并工况覆盖）")
    msg = ep.trajectory_command(node1)
    check(msg["cmd"] == "set_trajectory" and msg["type"] == "csv"
          and msg["csv_path"] == node1.trajectory["csv_path"], "set_trajectory 消息体正确")

    print("\n[2] 纯逻辑：坏文件只报错不拖垮")
    with tempfile.TemporaryDirectory() as d:
        Path(d, "01_good.yaml").write_text(
            "name: X\nkind: node\nmode: CSV\ndescription: d\n"
            "trajectory: {type: constant}\n", encoding="utf-8")
        Path(d, "02_bad.yaml").write_text("name: Y\nkind: 不存在\n", encoding="utf-8")
        Path(d, "03_badpath.yaml").write_text(
            "name: Z\nkind: node\nmode: CSV\ndescription: d\n"
            "trajectory: {type: csv, csv_path: 不存在.csv}\n", encoding="utf-8")
        ps, errs = ep.scan_presets(Path(d))
        check(len(ps) == 1 and ps[0].name == "X", "好文件照常加载")
        check(len(errs) == 2, f"两个坏文件各报一条错（{len(errs)}）")


def test_panel():
    print("\n[3] 离屏 Qt：面板行为")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])

    from config import GuiConfig
    from widgets.experiment_panel import ExperimentPanel

    cfg = GuiConfig()
    panel = ExperimentPanel(cfg)
    check(panel.preset_list.count() == 14, f"列表显示 14 个配置（{panel.preset_list.count()}）")
    merged_idx = next(i for i, p in enumerate(panel._presets) if p.merged_from)
    check("（含 周期测试1_电流摩擦" in panel.preset_list.item(merged_idx).text(),
          "合并配置的列表条目带（含 …）后缀")

    sent = []
    panel.command.connect(lambda m: sent.append(m))

    # 选中节点1并加载
    idx = next(i for i, p in enumerate(panel._presets)
               if p.name == "周期测试1_电流摩擦_空载0Tr_10rpm正转")
    panel.preset_list.setCurrentRow(idx)
    check("机械脱开" in panel.preset_desc.text(), "选中后显示操作提示（含加载装置说明）")
    panel._load_selected()
    check([m["cmd"] for m in sent] == ["set_mode", "set_trajectory"],
          f"加载只发 set_mode+set_trajectory，不使能不运行（{[m['cmd'] for m in sent]}）")
    check(sent[0]["mode"] == "CSV", "set_mode = CSV")
    check(sent[1]["type"] == "csv" and sent[1]["csv_path"].endswith(".csv"),
          "set_trajectory 带绝对路径 csv_path")
    check("周期测试1_电流摩擦_空载0Tr_10rpm正转" in panel.btn_b.text(), "线B按钮文字带配置名")
    check(panel._loaded_preset is not None, "配置已记录为当前配置")

    # 配置(线B)与按钮(线A)不匹配 → _start 直接拒绝(弹框离屏自动关) ,不发命令
    sent.clear()
    from PySide6 import QtWidgets
    orig = QtWidgets.QMessageBox.warning
    warned = []
    QtWidgets.QMessageBox.warning = staticmethod(
        lambda *a, **k: warned.append(a[2] if len(a) > 2 else ""))
    try:
        panel._start("A")
        check(len(warned) == 1 and "应该用" in warned[0], "线不匹配被拦截并提示")
        check(not sent, "拦截时不发出任何命令")

        # test_name 同步:绕过弹框,直接构造 record 消息路径——
        # 模拟 precheck 通过 + 对话框返回值
        panel._status = {"ethercat": "OP", "servo": "Operation Enabled",
                         "mode_matched": True}
        import widgets.experiment_panel as epn
        class FakeDialog:
            def __init__(self, line, last_dir, parent, prefill=None):
                self.prefill = prefill or {}
            def get_values(self):
                v = {"sample_id": "A01", "out_dir": "/tmp", "line": "B",
                     "life_hours": 100, "rep": 1}
                v.update(test_item=self.prefill.get("test_item", "current"),
                         load_percent_Tr=self.prefill.get("load_percent_Tr", 0),
                         speed_rpm_target=self.prefill.get("speed_rpm_target", 5))
                return v
        orig_dialog = epn.StartDialog
        epn.StartDialog = FakeDialog
        try:
            sent.clear()
            panel._start("B")
            rec = next(m for m in sent if m["cmd"] == "record_start")
            check(rec["test_name"] == "A01_周期测试1_电流摩擦_空载0Tr_10rpm正转_100h",
                  f"test_name 与配置名同步且带节点小时（{rec['test_name']}）")
            check(rec["test_item"] == "current" and rec["load_percent_Tr"] == 0
                  and rec["speed_rpm_target"] == 10, "record 字段来自配置预填")
            check(not any(m["cmd"] == "start_run" for m in sent),
                  "start_run 仍等 record ack，未直接发出")
            check(panel._loaded_preset.name in rec["test_name"]
                  and panel._pending.get("config_name") == panel._loaded_preset.name,
                  "config_name 进了 pending 元数据（完成弹框标题用）")
            panel.on_record_ack(True)
            check(any(m["cmd"] == "start_run" for m in sent), "ack 后才发 start_run")

            # ── 线B 自动收尾：running True→False 下降沿触发 _finish ──
            panel._auto_finish_delay_ms = 0
            done = []
            panel.finished.connect(lambda line, meta: done.append((line, meta)))
            base = {"ethercat": "OP", "servo": "Operation Enabled",
                    "mode_matched": True}
            panel.update_status({**base, "running": True})
            check(not done, "运行中不收尾")
            sent.clear()
            panel.update_status({**base, "running": False})   # 轨迹播完自动软停
            for _ in range(10):
                app.processEvents()                            # 让 singleShot(0) 跑
            check(len(done) == 1 and done[0][0] == "B", "运行结束自动触发收尾")
            check(done[0][1].get("config_name") == "周期测试1_电流摩擦_空载0Tr_10rpm正转",
                  "finished 元数据带配置名")
            check([m["cmd"] for m in sent] == ["stop_run", "record_stop"],
                  f"自动收尾发 stop_run+record_stop（{[m['cmd'] for m in sent]}）")
            check(panel._active_line is None, "收尾后状态机回到 idle")
            # 再来一拍 running=False 不应重复收尾
            panel.update_status({**base, "running": False})
            for _ in range(10):
                app.processEvents()
            check(len(done) == 1, "不重复收尾")
        finally:
            epn.StartDialog = orig_dialog
    finally:
        QtWidgets.QMessageBox.warning = orig


def test_export_waits_for_file_close():
    """真机 bug 回归：点结束后立刻读 h5 撞 HDF5 写锁（errno=11）。
    修复后 _on_experiment_finished 只挂待办，等 recording active:false
    （后端同步关完文件才发）再导出+弹框。"""
    print("\n[4] 导出等待录制关闭（h5 锁竞态回归）")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])

    from config import GuiConfig
    import main_window as mw

    win = mw.MainWindow(GuiConfig())
    shown = []
    class FakeSummary:
        def __init__(self, info, parent=None):
            shown.append(info)
        def exec(self):
            return 0
    import widgets.experiment_dialog as ed
    orig = ed.SummaryDialog
    ed.SummaryDialog = FakeSummary
    try:
        win._last_rec_file = ""      # 无 h5 → 走"跳过导出"分支，不真读文件
        win._on_experiment_finished("B", {"config_name": "X", "sample_id": "A01",
                                          "export_csv": True})
        check(win._pending_export is not None, "收尾后先挂待办，不立刻导出")
        check(not shown, "recording 未关闭前不弹框")
        win._on_recording({"active": True, "file": "", "samples": 1})
        check(not shown, "active:true 事件不触发导出")
        win._on_recording({"active": False, "file": "", "samples": 1})
        check(len(shown) == 1, "active:false 后才导出并弹框")
        check(shown[0].get("title") == "实验完成：X", "弹框标题带配置名")
        check(win._pending_export is None, "待办已清空（兜底定时器晚到也不会重复）")
        win._flush_pending_export()
        check(len(shown) == 1, "重复 flush 不再弹框")
    finally:
        ed.SummaryDialog = orig


def test_export_runs_off_main_thread():
    """真机 bug 回归（2026-08-13）：线B 收尾导出曾在主线程逐元素读 h5，
    大文件堵住事件循环几分钟，GNOME 弹"python 无响应"。修复后导出在
    工作线程跑，主线程只轮询收尾——弹框异步到达，期间事件循环保持存活。"""
    print("\n[5] 导出在工作线程（python 无响应回归）")
    import tempfile, time as _t
    import h5py, numpy as np
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])

    from config import GuiConfig
    import main_window as mw

    win = mw.MainWindow(GuiConfig())
    shown = []
    class FakeSummary:
        def __init__(self, info, parent=None):
            shown.append(info)
        def exec(self):
            return 0
    import widgets.experiment_dialog as ed
    orig = ed.SummaryDialog
    ed.SummaryDialog = FakeSummary
    tmp = tempfile.mkdtemp(prefix="ecjc_export_test_")
    h5_path = os.path.join(tmp, "rec.h5")
    with h5py.File(h5_path, "w") as f:
        g = f.create_group("experiment")
        g.create_dataset("elapsed_time_s", data=np.arange(100) / 2000.0)
    try:
        win._last_rec_file = h5_path
        win._last_rec_samples = 100
        meta = {"config_name": "Y", "sample_id": "A01", "life_hours": 0,
                "test_item": "TE", "load_percent_Tr": 0, "speed_rpm_target": 5,
                "rep": 1, "out_dir": tmp, "export_csv": True}
        win._on_experiment_finished("B", meta)
        win._on_recording({"active": False, "file": h5_path, "samples": 100})
        check(not shown, "flush 立即返回，弹框不在同一调用栈里（主线程未被堵住）")
        deadline = _t.monotonic() + 10
        while not shown and _t.monotonic() < deadline:
            app.processEvents()
            _t.sleep(0.02)
        check(len(shown) == 1, "导出线程结束后弹出汇总框")
        csv_path = shown[0].get("csv_path")
        check(bool(csv_path) and os.path.isfile(csv_path), "CSV 已真实落盘")
        check("__t_" in os.path.basename(csv_path),
              f"线B CSV 文件名带时间点（{os.path.basename(csv_path)}）")
        check(shown[0].get("error") is None, f"无导出错误: {shown[0].get('error')}")
    finally:
        ed.SummaryDialog = orig


def test_out_dir_writable_preflight():
    """真机 bug 回归（2026-08-13）：out_dir 若不存在，root 后端会把它建成
    root 属主，采集正常但收尾 CSV 导出 EACCES。修复：开始前 GUI 以当前用户
    先建目录并检查写权限，不可写就当场拦下，别让实验跑完才失败。"""
    print("\n[6] 输出目录写权限预检")
    import stat, tempfile
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    import widgets.experiment_panel as epn

    panel = epn.ExperimentPanel(cfg=None)
    tmp = tempfile.mkdtemp(prefix="ecjc_preflight_")

    nested = os.path.join(tmp, "a", "b")
    check(panel._ensure_writable(nested) is None, "不存在的目录:自动创建并通过")
    check(os.path.isdir(nested), "目录确实建出来了(用户属主)")

    ro = os.path.join(tmp, "ro")
    os.mkdir(ro)
    os.chmod(ro, stat.S_IRUSR | stat.S_IXUSR)   # r-x:模拟 root 属主目录不可写
    err = panel._ensure_writable(ro)
    check(err is not None and "写" in err, f"只读目录被拦下并说明原因: {err}")
    os.chmod(ro, 0o755)

    inside_ro = os.path.join(tmp, "ro2", "sub")
    os.mkdir(os.path.join(tmp, "ro2"))
    os.chmod(os.path.join(tmp, "ro2"), stat.S_IRUSR | stat.S_IXUSR)
    err2 = panel._ensure_writable(inside_ro)
    check(err2 is not None, f"建不出来的目录也被拦下: {err2}")
    os.chmod(os.path.join(tmp, "ro2"), 0o755)


def test_start_dialog_locks_preset_fields():
    """[7] 开始弹框：加载配置后，配置定死的三项(测试项/载荷/速度)锁定不可编辑——
    真机踩坑(2026-08-14)：这些只是数据标注，改了不影响实际转速，还会让标注说谎。"""
    print("\n[7] 开始弹框字段锁定")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    QApplication.instance() or QApplication([])
    from widgets.experiment_dialog import StartDialog

    d = StartDialog("B", prefill={"test_item": "TE", "load_percent_Tr": 10,
                                  "speed_rpm_target": 5})
    check(not d.speed.isEnabled() and not d.load_pct.isEnabled()
          and not d.test_item.isEnabled(), "预填字段(速度/载荷/测试项)已锁定")
    check(d.life_hours.isEnabled() and d.rep.isEnabled(),
          "节点小时/重复号仍可编辑（真正因节点而异）")
    check(d.speed.value() == 5 and d.load_pct.value() == 10
          and d.test_item.currentText() == "TE", "锁定字段值来自配置")

    m = StartDialog("B")   # 手动模式：全部可编辑
    check(m.speed.isEnabled() and m.load_pct.isEnabled() and m.test_item.isEnabled(),
          "手动模式（未加载配置）字段可编辑")

    a = StartDialog("A")   # 线A 手动模式没有这些字段
    check(not hasattr(a, "speed"), "线A 手动模式弹框无速度字段")


def test_start_dialog_line_a_speed():
    """[8] 线A 弹框：加载恒速配置（prefill 带 speed_rpm_target）时出现可编辑的
    转速框——与线B 不同，这个值会真实下发覆盖配置默认值（2026-08-14 需求：
    摆臂实验只跑一个匀速转速，默认 20 rpm，现场可改）。"""
    print("\n[8] 线A 弹框转速覆盖框")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication, QDialog
    QApplication.instance() or QApplication([])
    from widgets.experiment_dialog import StartDialog

    d = StartDialog("A", prefill={"speed_rpm_target": 20})
    check(hasattr(d, "speed") and d.speed.isEnabled(),
          "prefill 带转速 → 出现可编辑的转速框")
    check(d.speed.value() == 20, f"默认值来自配置（{getattr(d, 'speed', None) and d.speed.value()}）")
    check(d.speed.minimum() < 0, "允许负值（反转）")
    check(hasattr(d, "life_label") and d.life_label.isEnabled(),
          "线A 弹框有可编辑的寿命区间标注（自由文本，能写 0-5h 这种时间段）")
    check(hasattr(d, "export_csv") and not d.export_csv.isChecked(),
          "线A 有导出 CSV 勾选框且默认不勾（持续运行数据太大）")
    d.speed.setValue(15)
    d.life_label.setText(" 0-5h ")
    d.export_csv.setChecked(True)
    d.exec = lambda: QDialog.Accepted        # 离屏不真弹框
    v = d.get_values()
    check(v["speed_rpm_target"] == 15, f"get_values 带用户改后的转速（{v.get('speed_rpm_target')}）")
    check(v["life_label"] == "0-5h" and v["export_csv"] is True,
          "get_values 带寿命区间（已去首尾空白）与导出选择")

    b = StartDialog("B")
    check(hasattr(b, "export_csv") and b.export_csv.isChecked(),
          "线B 也有导出勾选框且默认勾选（维持既有自动导出行为）")
    b.export_csv.setChecked(False)
    b.exec = lambda: QDialog.Accepted
    check(b.get_values()["export_csv"] is False, "线B 可关掉自动导出")


def test_line_a_speed_override():
    """[9] 线A 开始流程：弹框里改的转速在 record_start 前重发 set_trajectory
    覆盖下发，且 record_start 的 speed_rpm_target 标注 = 实际下发值（标注不说谎）。"""
    print("\n[9] 线A 转速实际下发覆盖")
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    QApplication.instance() or QApplication([])
    from config import GuiConfig
    import widgets.experiment_panel as epn

    panel = epn.ExperimentPanel(GuiConfig())
    sent = []
    panel.command.connect(lambda m: sent.append(m))
    idx = next(i for i, p in enumerate(panel._presets) if p.kind == "continuous")
    panel.preset_list.setCurrentRow(idx)
    panel._load_selected()
    check(sent[-1]["cmd"] == "set_trajectory" and sent[-1]["value"] == 25,
          "加载配置下发默认 25 rpm 恒速轨迹")

    panel._status = {"ethercat": "OP", "servo": "Operation Enabled",
                     "mode_matched": True}

    class FakeDialog:
        def __init__(self, line, last_dir, parent, prefill=None):
            self.prefill = prefill or {}
        def get_values(self):
            return {"sample_id": "A01", "out_dir": "/tmp", "line": "A",
                    "speed_rpm_target": 15, "life_label": "0-5h",
                    "export_csv": False}
    orig_dialog = epn.StartDialog
    epn.StartDialog = FakeDialog
    try:
        sent.clear()
        panel._start("A")
        cmds = [m["cmd"] for m in sent]
        check("set_trajectory" in cmds and "record_start" in cmds
              and cmds.index("set_trajectory") < cmds.index("record_start"),
              f"先重发轨迹再 record_start（{cmds}）")
        traj = next(m for m in sent if m["cmd"] == "set_trajectory")
        check(traj["type"] == "constant" and traj["value"] == 15
              and traj["duration_s"] == -1,
              f"覆盖后的恒速轨迹 value=15、无限时长（{traj}）")
        rec = next(m for m in sent if m["cmd"] == "record_start")
        check(rec["speed_rpm_target"] == 15, "record 标注 = 实际下发转速")
        check(rec["test_name"] == "A01_持续运行_寿命摆臂_0-5h",
              f"线A 文件名带寿命区间（{rec['test_name']}）")
        check("0-5h" in rec.get("notes", ""), "寿命区间同时写进 record notes 元数据")
        panel.on_record_ack(True)
        check(any(m["cmd"] == "start_run" for m in sent), "ack 后发 start_run")
        check(panel._pending.get("export_csv") is False,
              "导出选择进 pending 元数据（收尾时 main_window 据此决定导不导）")
    finally:
        epn.StartDialog = orig_dialog


def test_csv_export_opt_in():
    """[10] CSV 导出改为弹框勾选决定（2026-08-14 需求：数据太大不好存）：
    export_csv=False 不导（哪怕线B）；线A 勾了也导，CSV 与 h5 同名同目录
    （线A 没有 test_item/载荷/重复号，不套 §4.2 节点命名模板）。"""
    print("\n[10] CSV 导出按勾选")
    import tempfile, time as _t
    import h5py, numpy as np
    import experiment_naming as en

    # 纯逻辑：§4.2 模板名可附时间戳段（与 h5 的 fileStamp 同源，用于区分
    # 同工况多次运行，且防重名覆盖）
    check(en.csv_filename("A01", 0, "TE", 0, 5, 1, stamp="20260814_103000")
          .endswith("__rep_01__t_20260814_103000.csv"), "csv_filename 带时间戳段")
    check(en.csv_filename("A01", 0, "TE", 0, 5, 1).endswith("__rep_01.csv"),
          "不传 stamp 时保持原 §4.2 模板名不变")

    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from config import GuiConfig
    import main_window as mw

    win = mw.MainWindow(GuiConfig())
    shown = []
    class FakeSummary:
        def __init__(self, info, parent=None):
            shown.append(info)
        def exec(self):
            return 0
    import widgets.experiment_dialog as ed
    orig = ed.SummaryDialog
    ed.SummaryDialog = FakeSummary
    tmp = tempfile.mkdtemp(prefix="ecjc_optin_test_")
    h5_path = os.path.join(tmp, "A01_持续运行_寿命摆臂_5h_x.h5")
    with h5py.File(h5_path, "w") as f:
        g = f.create_group("experiment")
        g.create_dataset("elapsed_time_s", data=np.arange(50) / 2000.0)
    try:
        # 线B 不勾 → 不导出，直接弹汇总框
        win._last_rec_file = h5_path
        win._last_rec_samples = 50
        win._on_experiment_finished("B", {"config_name": "X", "sample_id": "A01",
                                          "export_csv": False, "out_dir": tmp})
        win._on_recording({"active": False, "file": h5_path, "samples": 50})
        check(len(shown) == 1 and shown[0].get("csv_path") is None
              and shown[0].get("error") is None,
              "线B 不勾选：跳过导出、无报错、直接弹框")

        # 线A 勾了 → 导出，CSV 与 h5 同名
        shown.clear()
        win._on_experiment_finished("A", {"config_name": "持续运行_寿命摆臂",
                                          "sample_id": "A01", "life_label": "0-5h",
                                          "export_csv": True, "out_dir": tmp})
        win._on_recording({"active": False, "file": h5_path, "samples": 50})
        deadline = _t.monotonic() + 10
        while not shown and _t.monotonic() < deadline:
            app.processEvents()
            _t.sleep(0.02)
        check(len(shown) == 1, "线A 勾选后导出完成弹框")
        csv_path = shown[0].get("csv_path")
        check(bool(csv_path) and os.path.isfile(csv_path),
              f"线A CSV 已落盘（{csv_path}）")
        check(csv_path == os.path.join(tmp, "A01_持续运行_寿命摆臂_5h_x.csv"),
              "线A CSV 与 h5 同名同目录（不套节点命名模板）")
        check(shown[0].get("error") is None, f"无导出错误: {shown[0].get('error')}")
    finally:
        ed.SummaryDialog = orig


def main():
    test_loader()
    test_panel()
    test_export_waits_for_file_close()
    test_export_runs_off_main_thread()
    test_out_dir_writable_preflight()
    test_start_dialog_locks_preset_fields()
    test_start_dialog_line_a_speed()
    test_line_a_speed_override()
    test_csv_export_opt_in()
    print("\n" + "=" * 50)
    if failures:
        print(f"{len(failures)} 项失败"); return 1
    print("全部通过"); return 0


if __name__ == "__main__":
    sys.exit(main())
