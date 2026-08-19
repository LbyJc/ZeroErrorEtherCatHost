#!/usr/bin/env python3
"""EtherCAT Joint Control —— GUI 入口。

以普通用户身份运行，不需要任何特权（任务书第十一节）。
需要 root 的操作（启动主站、实时线程、/dev/EtherCAT0）全部在 Backend 里，
通过 Unix socket 请求，必要时由 pkexec 弹一次图形密码框，不保存密码。
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

# 允许直接 `python gui/main.py` 运行
sys.path.insert(0, str(Path(__file__).resolve().parent))

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QApplication, QMessageBox

from config import GuiConfig
from main_window import MainWindow


def find_backend(root: Path) -> Path | None:
    for c in (root / "build" / "ecjc-backend",
              Path("/opt/ethercat-joint-control/bin/ecjc-backend"),
              root / "bin" / "ecjc-backend"):
        if c.is_file() and os.access(c, os.X_OK):
            return c
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="EtherCAT Joint Control GUI")
    ap.add_argument("--mock", "--simulation", dest="mock", action="store_true",
                    help="无硬件仿真模式：自动拉起 --mock 后端，不接触真实 EtherCAT")
    ap.add_argument("--config", default=None, help="配置目录")
    ap.add_argument("--no-spawn", action="store_true",
                    help="不自动拉起 Backend，只连接已运行的实例")
    # 开发/文档用：Wayland 下外部截图工具抓不到窗口，让程序自己截
    ap.add_argument("--screenshot", metavar="PATH", default=None,
                    help=argparse.SUPPRESS)
    ap.add_argument("--screenshot-delay", type=float, default=6.0,
                    help=argparse.SUPPRESS)
    ap.add_argument("--autorun", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    app = QApplication(sys.argv)
    app.setApplicationName("EtherCAT Joint Control")
    # 本机 Wayland/GNOME 的 xdg-desktop-portal 文件选择器会偶发无法映射窗口
    # （mutter: "surface_state_changed: assertion 'wl_window->has_last_sent_
    # configuration' failed"），原生对话框永不显示，而 Qt 阻塞在嵌套事件循环里
    # 等它返回——主窗口被模态锁死、连关都关不掉（2026-08-13 真机，py-spy 抓到
    # 栈停在 QFileDialog.getExistingDirectory）。全局改用 Qt 自绘对话框，
    # 不依赖 portal 进程状态。
    QApplication.setAttribute(Qt.AA_DontUseNativeDialogs, True)
    # 自绘对话框的按钮/标签是 Qt 自己的文案，装上官方中文翻译（qtbase_zh_CN）
    from PySide6.QtCore import QLibraryInfo, QLocale, QTranslator
    qt_tr = QTranslator(app)
    if qt_tr.load(QLocale.system(), "qtbase", "_",
                  QLibraryInfo.path(QLibraryInfo.LibraryPath.TranslationsPath)):
        app.installTranslator(qt_tr)

    cfg = GuiConfig(Path(args.config) if args.config else None)

    proc = None
    if args.mock and not args.no_spawn:
        # Mock 模式下 Backend 不需要 root，GUI 直接拉起一个子进程，
        # 用户双击就能看到完整界面在跑，不用先开终端
        exe = find_backend(cfg.root)
        if exe is None:
            QMessageBox.critical(
                None, "找不到 Backend",
                f"未找到可执行的 ecjc-backend。\n\n"
                f"请先编译：\n"
                f"  cd {cfg.root}\n"
                f"  cmake -S . -B build && cmake --build build -j\n")
            return 1
        sock = "/tmp/ecjc-mock.sock"
        os.environ["ECJC_SOCKET"] = sock
        proc = subprocess.Popen(
            [str(exe), "--mock", "--config", str(cfg.config_dir), "--socket", sock],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.4)   # 等 socket 建好，避免第一次连接必然失败

    win = MainWindow(cfg, mock=args.mock)
    win.show()
    win.start()

    if args.autorun:
        # 仅用于截图/演示：自动走完 启动主站 → 使能 → CSV → 运行 的流程。
        # 正常使用绝不会走到这里（任务书第十二节要求这些步骤必须手动）。
        from PySide6.QtCore import QTimer as _T
        seq = [
            (1200, {"cmd": "connect_bus"}),
            (2800, {"cmd": "set_mode", "mode": "CSV"}),
            (3200, {"cmd": "servo_enable"}),
            (4200, {"cmd": "set_trajectory", "type": "sine", "offset": 60.0,
                    "amplitude": 40.0, "frequency_hz": 0.25, "duration_s": -1}),
            (4600, {"cmd": "start_run"}),
        ]
        for ms, msg in seq:
            _T.singleShot(ms, lambda m=msg: win.ipc.send(m))

    if args.screenshot:
        from PySide6.QtCore import QTimer as _T

        def _grab():
            win.grab().save(args.screenshot)
            print(f"截图已保存: {args.screenshot}")
            app.quit()

        _T.singleShot(int(args.screenshot_delay * 1000), _grab)

    try:
        rc = app.exec()
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
    return rc


if __name__ == "__main__":
    sys.exit(main())
