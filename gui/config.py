"""GUI 侧配置加载。与 Backend 读的是同一批 yaml，保证两边看到同一个世界。"""
from __future__ import annotations

import os
from pathlib import Path

import yaml


def project_root() -> Path:
    # 开发模式：gui/ 的上一级；安装后：/opt/ethercat-joint-control
    here = Path(__file__).resolve().parent
    if (here.parent / "config").is_dir():
        return here.parent
    return Path("/opt/ethercat-joint-control")


class GuiConfig:
    def __init__(self, config_dir: Path | None = None):
        self.root = project_root()
        self.config_dir = Path(config_dir) if config_dir else self._find_config_dir()
        self.app = self._load("app.yaml").get("app", {})
        self.logging = self._load("app.yaml").get("logging", {})
        self.gui = self._load("gui.yaml").get("gui", {})
        self.ethercat = self._load("ethercat.yaml").get("ethercat", {})
        self.slave = self._load("slave.yaml").get("slave", {})
        self.scaling = self._load("scaling.yaml").get("scaling", {})
        self.trajectory = self._load("trajectory.yaml").get("trajectory", {})
        self.controller = self._load("controller.yaml").get("controller", {})

    def _find_config_dir(self) -> Path:
        for c in (self.root / "config", Path("/etc/ethercat-joint-control")):
            if c.is_dir():
                return c
        return self.root / "config"

    def _load(self, name: str) -> dict:
        p = self.config_dir / name
        if not p.is_file():
            return {}
        with open(p, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    # ── 便捷访问 ────────────────────────────────────────────────────────
    @property
    def socket_path(self) -> str:
        # 以 root 跑的 Backend 用 /run/...，开发模式下用 /tmp/...
        # 两个都试，先连上哪个算哪个
        return self.app.get("socket_path", "/run/ethercat-joint-control/control.sock")

    @property
    def socket_path_dev(self) -> str:
        return self.app.get("socket_path_dev", "/tmp/ecjc-control.sock")

    def candidate_sockets(self) -> list[str]:
        env = os.environ.get("ECJC_SOCKET")
        out = [env] if env else []
        out += [self.socket_path, self.socket_path_dev]
        return [p for p in out if p]

    @property
    def helper(self) -> str:
        """需要 root 的动作全部收敛到这个脚本，由 pkexec 调用。"""
        return self.app.get("helper", "/opt/ethercat-joint-control/bin/ecjc-helper")

    @property
    def data_dir(self) -> Path:
        d = self.app.get("data_dir", "data")
        p = Path(d)
        return p if p.is_absolute() else self.root / p

    @property
    def log_dir(self) -> Path:
        d = self.app.get("log_dir", "logs")
        p = Path(d)
        return p if p.is_absolute() else self.root / p

    @property
    def plot_fps(self) -> int:
        return int(self.gui.get("plot_fps", 50))

    @property
    def telemetry_hz(self) -> int:
        return int(self.gui.get("telemetry_publish_hz", 100))

    @property
    def x_window_choices(self) -> list[int]:
        return list(self.gui.get("x_window_choices", [5, 10, 30, 60, 300]))

    @property
    def default_x_window(self) -> int:
        return int(self.gui.get("default_x_window_s", 10))

    @property
    def plots(self) -> list[dict]:
        return list(self.gui.get("plots", []))

    @property
    def encoder_verified(self) -> bool:
        return bool(self.scaling.get("encoder_resolution_verified", False))

    @property
    def supports_homing(self) -> bool:
        return bool(self.slave.get("supports_homing", False))
