"""主窗口。布局按任务书第五十节：顶部状态栏 / 左中右三栏 / 下方曲线 / 五个 Tab。"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QHBoxLayout, QMainWindow, QMessageBox, QSplitter, QStatusBar, QTabWidget,
    QVBoxLayout, QWidget,
)

from ipc_client import IpcClient
from widgets.cia402_panel import Cia402Panel
from widgets.config_panel import ConfigPanel
from widgets.data_record_panel import DataRecordPanel
from widgets.experiment_panel import ExperimentPanel
from widgets.live_panel import LivePanel
from widgets.log_panel import LogPanel
from widgets.mode_panel import ModePanel
from widgets.parameter_panel import ParameterPanel
from widgets.plot_panel import PlotPanel
from widgets.status_bar import TopStatusBar
from widgets.system_panel import SystemPanel


class _EmptyStatus:
    """断开时喂给顶栏的空状态，让所有灯回到 idle 而不是停在旧值。"""
    @staticmethod
    def get(k, default=None):
        return default


_EMPTY_STATUS = _EmptyStatus()


class MainWindow(QMainWindow):
    def __init__(self, cfg, mock: bool = False):
        super().__init__()
        self.cfg = cfg
        self.mock = mock
        self._last_status = None
        self._backend_proc = None
        self._pending_cmd = None
        self._perm_warned = False
        self._last_rec_file = ""
        self._pending_export = None   # 一键实验收尾后待办的导出任务，等录制真正关闭再做
        self._last_rec_samples = ""

        self.setWindowTitle(
            f"{cfg.app.get('name', 'EtherCAT Joint Control')} "
            f"v{cfg.app.get('version', '0.1.0')}" + ("  —  MOCK 模式" if mock else ""))
        self.resize(1680, 1000)

        # ── IPC ─────────────────────────────────────────────────────
        self.ipc = IpcClient(self)

        # ── 顶部状态栏 ──────────────────────────────────────────────
        self.top = TopStatusBar()

        # ── Monitor 页 ──────────────────────────────────────────────
        self.system_panel = SystemPanel(cfg)
        self.cia_panel = Cia402Panel(cfg)
        self.mode_panel = ModePanel(cfg)
        self.experiment_panel = ExperimentPanel(cfg)
        self.live_panel = LivePanel(cfg)
        self.plot_panel = PlotPanel(cfg)

        left = QWidget()
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.addWidget(self.system_panel)
        ll.addWidget(self.cia_panel)
        left_scroll = _scroll(left)

        mid = QWidget()
        ml = QVBoxLayout(mid)
        ml.setContentsMargins(0, 0, 0, 0)
        ml.addWidget(self.experiment_panel)
        ml.addWidget(self.mode_panel)

        cols = QSplitter(Qt.Horizontal)
        cols.addWidget(left_scroll)
        cols.addWidget(_scroll(mid))
        cols.addWidget(_scroll(self.live_panel))
        cols.setSizes([380, 420, 340])

        monitor = QSplitter(Qt.Vertical)
        monitor.addWidget(cols)
        monitor.addWidget(self.plot_panel)
        monitor.setSizes([520, 480])

        # ── 其余 Tab ────────────────────────────────────────────────
        self.param_panel = ParameterPanel(cfg)
        self.record_panel = DataRecordPanel(cfg)
        self.log_panel = LogPanel(cfg)
        self.config_panel = ConfigPanel(cfg)

        self.tabs = QTabWidget()
        self.tabs.addTab(monitor, "Monitor")
        self.tabs.addTab(self.param_panel, "Parameter Tuning")
        self.tabs.addTab(self.record_panel, "Data Acquisition")
        self.tabs.addTab(self.log_panel, "Log")
        self.tabs.addTab(self.config_panel, "System Configuration")

        central = QWidget()
        cl = QVBoxLayout(central)
        cl.setContentsMargins(0, 0, 0, 0)
        cl.setSpacing(0)
        cl.addWidget(self.top)
        cl.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("未连接")

        # ── 信号连接 ────────────────────────────────────────────────
        for p in (self.system_panel, self.cia_panel, self.mode_panel,
                  self.param_panel, self.record_panel, self.experiment_panel):
            p.command.connect(self._send)
        self.experiment_panel.finished.connect(self._on_experiment_finished)

        self.ipc.connected.connect(self._on_connected)
        self.ipc.disconnected.connect(self._on_disconnected)
        self.ipc.connect_failed.connect(self._on_connect_failed)
        self.ipc.status_changed.connect(self._on_status)
        self.ipc.telemetry.connect(self._on_telemetry)
        self.ipc.log_message.connect(self.log_panel.append)
        self.ipc.startup_step.connect(self._on_step)
        self.ipc.params_changed.connect(self._on_params)
        self.ipc.recording_changed.connect(self._on_recording)
        self.ipc.ack.connect(self._on_ack)
        self.ipc.handshake_ok.connect(self._on_handshake)

        # ── 绘图刷新：与遥测频率、控制周期完全解耦 ───────────────────
        self.redraw_timer = QTimer(self)
        self.redraw_timer.setInterval(max(10, int(1000 / cfg.plot_fps)))
        self.redraw_timer.timeout.connect(self.plot_panel.redraw)
        self.redraw_timer.start()

        self.log_panel.append("INFO", f"GUI 启动，绘图 {cfg.plot_fps} Hz，"
                                      f"遥测 {cfg.telemetry_hz} Hz，"
                                      f"控制周期 {cfg.ethercat.get('cycle_us', 1000)} µs")
        if not cfg.encoder_verified:
            self.log_panel.append(
                "WARNING",
                "输出侧编码器分辨率未经物理转角验证；若实为 2^18，所有 rpm 数值需翻倍。"
                "详见 System Configuration 页。")

    # ── 启动自检与连接 ──────────────────────────────────────────────────
    def start(self):
        self._preflight()
        paths = self.cfg.candidate_sockets()
        self.log_panel.append("INFO", f"尝试连接 Backend: {paths[0]}")
        self.ipc.connect_to(paths[0])
        # 首选路径连不上就换下一个（root 服务 vs 开发模式）
        QTimer.singleShot(1200, lambda: self._fallback(paths))

    def _fallback(self, paths):
        if self.ipc.is_connected() or len(paths) < 2:
            return
        self.log_panel.append("INFO", f"改用备用路径: {paths[1]}")
        self.ipc.connect_to(paths[1])

    def _preflight(self):
        """任务书第十二节：启动时自检并显示系统状态，但什么都不自动使能。"""
        log = self.log_panel.append
        log("INFO", "—— 启动自检 ——")

        cd = Path(self.cfg.config_dir)
        log("INFO" if cd.is_dir() else "ERROR", f"配置目录: {cd}")

        iface = self.cfg.ethercat.get("interface", "")
        if self.mock:
            log("INFO", "MOCK 模式：跳过网卡与 IgH 检查")
        else:
            if iface and Path(f"/sys/class/net/{iface}").exists():
                try:
                    op = Path(f"/sys/class/net/{iface}/operstate").read_text().strip()
                except Exception:
                    op = "unknown"
                log("INFO", f"网卡 {iface} 存在，operstate={op}")
            else:
                log("ERROR", f"配置的网卡 {iface} 不存在，请检查 config/ethercat.yaml "
                             "的 interface 字段")

            if Path("/dev/EtherCAT0").exists():
                log("INFO", "IgH 主站设备 /dev/EtherCAT0 存在")
            else:
                log("WARNING", "未找到 /dev/EtherCAT0，IgH 内核模块可能未加载。"
                               "点击【启动主站】时会尝试拉起。")

        log("INFO", "自检完成。默认不自动使能伺服、不自动运行、不自动采集。")

    # ── IPC 事件 ────────────────────────────────────────────────────────
    def _send(self, msg: dict):
        # 双击图标启动时后端通常还没跑。【启动主站】要能自己把它拉起来，
        # 否则用户点了没反应，还得去开终端——那就违背了"全程不用终端"的目标。
        if not self.ipc.is_connected() and msg.get("cmd") in ("connect_bus", "reconnect"):
            self._start_backend_then(msg)
            return
        self.ipc.send(msg)

    def _start_backend_then(self, pending: dict):
        from PySide6.QtCore import QProcess

        helper = self.cfg.helper
        if not Path(helper).is_file():
            QMessageBox.warning(
                self, "后端未安装",
                f"后端未运行，且找不到特权助手：\n  {helper}\n\n"
                "开发模式下请先手动启动后端：\n"
                f"  pkexec {self.cfg.root}/build/ecjc-backend --config {self.cfg.config_dir}\n\n"
                "若想双击即用，请先执行安装：\n"
                f"  pkexec {self.cfg.root}/install.sh")
            return
        if self._backend_proc is not None:
            self.log_panel.append("INFO", "后端正在启动中，请稍候…")
            return

        self.log_panel.append("INFO", "后端未运行，正在通过 pkexec 启动（会弹一次密码框）…")
        self.statusBar().showMessage("正在启动后端…")
        self._pending_cmd = pending
        p = QProcess(self)
        p.finished.connect(lambda code, _st: self._on_backend_start_finished(code))
        p.start("pkexec", [helper, "start"])
        self._backend_proc = p

    def _on_backend_start_finished(self, code: int):
        out = ""
        if self._backend_proc is not None:
            out = bytes(self._backend_proc.readAllStandardError()).decode(errors="replace")
            out += bytes(self._backend_proc.readAllStandardOutput()).decode(errors="replace")
        self._backend_proc = None
        if code == 0:
            self.log_panel.append("INFO", "后端已启动，等待连接…")
            # 连上之后由 _on_connected 里的 _pending_cmd 接着走
        else:
            self._pending_cmd = None
            msg = out.strip() or f"pkexec 返回 {code}（可能是取消了授权）"
            self.log_panel.append("ERROR", f"启动后端失败: {msg}")
            self.statusBar().showMessage("启动后端失败", 8000)
            QMessageBox.warning(self, "启动后端失败", msg)

    def _on_connected(self):
        self.statusBar().showMessage("已连接到 Backend")
        self.log_panel.append("INFO", "已连接到 Backend")
        if self._pending_cmd:
            cmd, self._pending_cmd = self._pending_cmd, None
            QTimer.singleShot(300, lambda: self.ipc.send(cmd))

    def _on_disconnected(self):
        self.statusBar().showMessage("与 Backend 断开，正在自动重连…")
        self.log_panel.append("WARNING", "与 Backend 断开连接")
        # 断开后不会再有 status 事件，按钮状态必须在这里显式重置，
        # 否则会冻结在断开前的样子（后端崩溃时最容易踩到）。
        # cia_panel 同理：_running 若不复位，运行中断连会冻结在 True。
        self.system_panel.set_disconnected()
        self.cia_panel.set_disconnected()
        # experiment_panel 同理：record_start 已发、ack 还没回来时断连，
        # _awaiting_line 不复位就永久卡住两个一键按钮（复审发现的破坏 #1）。
        self.experiment_panel.on_disconnected()
        self.top.update_status(_EMPTY_STATUS)

    def _on_connect_failed(self, why: str):
        # 权限被拒是安装后最常见的一种"看起来像坏了"的情况：
        # socket 属主是 root:ethercat 0660，而用户组要重新登录才生效。
        # 静默重试会让人以为软件有问题，所以这里必须说清楚。
        if "ermission" in why or "拒绝" in why or "denied" in why.lower():
            if not self._perm_warned:
                self._perm_warned = True
                hint = ("你的当前登录会话还没有 ethercat 组权限。\n"
                        "安装脚本已把你加进该组，但需要**注销后重新登录一次**才生效。\n\n"
                        "想立刻验证而不注销，可临时放行：\n"
                        "  pkexec setfacl -m u:$USER:rw "
                        "/run/ethercat-joint-control/control.sock")
                self.log_panel.append("ERROR", "连接后端被拒绝（权限）。" +
                                      hint.replace("\n", " "))
                QMessageBox.warning(self, "无法连接后端（权限不足）", f"{why}\n\n{hint}")

    def _on_handshake(self, obj: dict):
        self.log_panel.append(
            "INFO", f"线格式校验通过（Sample {obj.get('sample_size')} 字节，"
                    f"协议 v{obj.get('protocol')}，Backend v{obj.get('version')}）")
        # 把界面上显示的初值下发一次，确保"看到的"就是"发出去的"
        self.mode_panel.push_initial()

    def _on_status(self, st):
        self._last_status = st
        # 后端 last_error 变化时记入日志（只记跳变，状态是 10 Hz 推送的）
        err = st.get("last_error") or ""
        if err != getattr(self, "_last_error_logged", ""):
            if err:
                self.log_panel.append("ERROR", err)
            self._last_error_logged = err
        self.top.update_status(st)
        self.system_panel.update_status(st, self.ipc.is_connected())
        self.cia_panel.update_status(st)
        self.mode_panel.update_status(st)
        self.record_panel.update_status(st)
        self.experiment_panel.update_status(st)

    def _on_telemetry(self, arr):
        self.plot_panel.append(arr)
        self.live_panel.update_sample(arr[-1])

    def _on_step(self, step, ok, msg):
        self.system_panel.set_step(step, ok, msg)
        self.log_panel.append("INFO" if ok else "ERROR",
                              f"[{'✓' if ok else '✗'}] {step}" + (f" — {msg}" if msg else ""))
        if step == "OP" and ok:
            self.log_panel.append("INFO", "EtherCAT System Ready")
            self.statusBar().showMessage("EtherCAT System Ready")

    def _on_params(self, obj):
        self.param_panel.set_params(obj)
        ctrls = obj.get("controllers", [])
        if ctrls:
            self.mode_panel.set_controllers(ctrls)

    def _on_recording(self, rec):
        self.record_panel.update_recording(rec)
        self.top.update_recording(rec)
        # 一键实验结束时导 CSV 要用这份 h5 路径；record_stop 后 recording
        # 事件仍会带着最后一次落盘的 file/samples，所以这里只缓存，不清空。
        f = rec.get("file", "")
        if f:
            self._last_rec_file = f
            self._last_rec_samples = rec.get("samples", "")
        # 一键实验收尾等的就是这个 active:false——后端 DataLogger::stop() 是
        # 同步关文件后才回发该事件，此刻读 h5 不会再撞 HDF5 写锁（真机实测：
        # 早读会报 unable to lock file, errno=11）。
        if self._pending_export is not None and not rec.get("active", True):
            self._flush_pending_export()

    def _on_experiment_finished(self, line: str, meta: dict):
        """一键实验面板点【结束】（或线B自动收尾）后触发。

        不能在这里立刻读 h5：record_stop 命令刚发出去，后端还没关文件，
        h5py 会撞上 HDF5 文件锁（errno=11 资源暂时不可用）。挂成待办，
        等 _on_recording 收到 active:false（文件确认已关）再导出+弹框；
        若事件迟迟不来（如采集早已手动停过，不会再有事件），10 s 兜底照做。
        """
        self._pending_export = {"line": line, "meta": meta}
        QTimer.singleShot(10000, self._flush_pending_export)

    def _flush_pending_export(self):
        if self._pending_export is None:
            return                      # 已做过（事件先到，兜底定时器晚到）
        line = self._pending_export["line"]
        meta = self._pending_export["meta"]
        self._pending_export = None

        from widgets.experiment_dialog import SummaryDialog

        h5_path = self._last_rec_file
        info = {"h5_path": h5_path, "samples": self._last_rec_samples}
        # 加载了实验配置时，完成弹框标题带配置名（2026-08-13 现场需求）
        if meta.get("config_name"):
            info["title"] = f"实验完成：{meta['config_name']}"
        # CSV 导出改为开始弹框勾选决定（2026-08-14 需求：CSV 大不好存），
        # 不再按线别硬编码——线B 默认勾（维持原行为），线A 默认不勾。
        want_csv = bool(meta.get("export_csv"))
        if want_csv and h5_path:
            # 导出必须放工作线程：TE 工况 20 万行即使整列读也要 ~10s，放主线程
            # 会把 Qt 事件循环堵到 GNOME 弹"python 无响应"强杀框（2026-08-13
            # 真机 bug）。工作线程只碰文件不碰 Qt；完成后由主线程定时器收尾弹框。
            self.statusBar().showMessage("正在导出 CSV，完成后弹出实验汇总…")
            t = threading.Thread(target=self._export_csv_worker,
                                 args=(h5_path, meta, info), daemon=True)
            t.start()
            self._watch_export(t, info)
            return
        if want_csv and not h5_path:
            info["csv_path"] = None
            info["error"] = "未找到本次录制的 HDF5 路径，跳过 CSV 导出。"
        SummaryDialog(info, self).exec()

    def _export_csv_worker(self, h5_path: str, meta: dict, info: dict):
        """工作线程：h5 → A.1 CSV。结果写进 info（主线程等线程死透才读，无竞争）。
        这里绝对不能碰任何 Qt 对象。"""
        try:
            sys.path.insert(0, os.path.join(self.cfg.root, "tools"))
            import h5_to_csv
            import experiment_naming as en

            # 时间戳与 h5 文件名同源（后端 fileStamp 的 _YYYYMMDD_HHMMSS 后缀），
            # 保证同一次运行的 csv/h5 一眼能对上；解析不出就用当前时间兜底。
            h5_base = os.path.basename(h5_path)
            m = re.search(r"_(\d{8}_\d{6})\.h5$", h5_base)
            stamp = m.group(1) if m else time.strftime("%Y%m%d_%H%M%S")
            if meta.get("test_item"):
                # 线B：§4.2 节点命名模板 + 时间戳段
                csv_name = en.csv_filename(
                    meta["sample_id"], meta["life_hours"], meta["test_item"],
                    meta["load_percent_Tr"], meta["speed_rpm_target"], meta["rep"],
                    stamp=stamp)
            else:
                # 线A（持续运行）：没有 test_item/载荷/重复号，不套节点模板，
                # 直接与 h5 同名（名字里已有配置名/寿命区间/时间戳）
                csv_name = h5_base[:-3] + ".csv" if h5_base.endswith(".h5") \
                    else h5_base + ".csv"
            csv_path = os.path.join(meta["out_dir"], csv_name)
            # 双保险：万一还有锁（NFS、别的读者），重试 3 次
            for attempt in range(3):
                try:
                    h5_to_csv.export_a1(h5_path, csv_path)
                    break
                except (BlockingIOError, OSError) as e:
                    if attempt == 2 or getattr(e, "errno", None) not in (11, None):
                        raise
                    time.sleep(0.5)
            info["csv_path"] = csv_path
        except Exception as e:
            info["csv_path"] = None
            info["error"] = f"CSV 导出失败: {e}"

    def _watch_export(self, t: threading.Thread, info: dict):
        if t.is_alive():
            QTimer.singleShot(200, lambda: self._watch_export(t, info))
            return
        from widgets.experiment_dialog import SummaryDialog
        self.statusBar().clearMessage()
        SummaryDialog(info, self).exec()

    def _on_ack(self, cmd, ok, msg):
        # I3（spec §5）：record_start 可能被后端拒绝（磁盘不足等），一键面板要
        # 靠这个 ack 决定发不发 start_run，不能发了 record_start 就当已经开始。
        # on_record_ack 返回 True 表示这条 record_start 确实是一键面板发起、
        # 已经弹过一次"记录未能启动"——下面通用失败弹框要跳过它，否则弹两个框
        # （复审发现的破坏 #2）。手动 record_panel 触发的 record_start 不经过
        # 一键面板的等待态，on_record_ack 返回 False，通用弹框照常弹，不受影响。
        handled_by_experiment_panel = False
        if cmd == "record_start":
            handled_by_experiment_panel = self.experiment_panel.on_record_ack(ok, msg)
        if ok:
            return
        # 失败必须给人话原因（任务书第四十三节），而且要显眼
        self.log_panel.append("ERROR", f"{cmd} 失败: {msg}")
        self.statusBar().showMessage(f"{cmd} 失败: {msg}", 8000)
        if handled_by_experiment_panel:
            return
        if cmd in ("connect_bus", "reconnect", "servo_enable", "start_run",
                   "record_start", "reset_load_encoder", "homing"):
            QMessageBox.warning(self, f"{cmd} 失败", msg or "未提供原因")

    # ── 关闭 ────────────────────────────────────────────────────────────
    def closeEvent(self, ev):
        running = bool(self._last_status and self._last_status.get("running"))
        if running:
            r = QMessageBox.question(
                self, "仍在运行",
                "关节仍处于运行状态。\n\n"
                "关闭 GUI 会让 Backend 立即执行软停（速度/力矩按斜坡归零），"
                "但伺服会保持使能。\n\n确认关闭？")
            if r != QMessageBox.Yes:
                ev.ignore()
                return
        self.ipc.close()
        ev.accept()


def _scroll(w):
    from PySide6.QtWidgets import QScrollArea
    a = QScrollArea()
    a.setWidgetResizable(True)
    a.setWidget(w)
    a.setFrameShape(QScrollArea.NoFrame)
    return a
