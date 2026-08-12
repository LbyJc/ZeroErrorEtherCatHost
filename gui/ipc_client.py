"""与 Backend 通信的客户端。

⚠ SAMPLE_FORMAT 必须与 backend/include/ecjc/types.hpp 的 Sample 结构体逐字段对应。
   两边都有尺寸断言，并且连上后会用 ping/pong 再校验一次实际 sizeof——
   宁可启动时明确报错，也不要画出一屏乱码曲线让人以为是硬件坏了。
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass, field

import numpy as np
from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtNetwork import QLocalSocket

# 帧
FRAME_MAGIC = 0x434A4345  # 'ECJC'
FRAME_HEADER = struct.Struct("<IHHI")  # magic, type, version, length
FRAME_TELEMETRY = 1
FRAME_JSON = 2
PROTOCOL_VERSION = 2

# 遥测样本：与 C++ Sample（backend/include/ecjc/types.hpp）逐字段一一对应。
# 分组按 C++ 实际布局（8 字节 -> 4 字节 -> 2 字节 -> 1 字节），逐段核对过 offsetof：
#   q   : system_time_ns
#   14d : elapsed_time_s .. velocity_error_rpm
#   8i  : motor_position_raw, output_position_raw, twist_counts,
#         following_error_counts, torque_est_mNm, aux_position_raw,
#         position_counts_raw, motor_position_sdo
#   4I  : dc_link_voltage_mV, warning_code, working_counter, seq
#   4H  : controlword, statusword, error_code, temperature_drive_C
#   2h  : torque_actual_permille, torque_ratio
#   bBBB: operation_mode, cia402_state, ethercat_state, flags
SAMPLE_FORMAT = "<q14d8i4I4H2hbBBB"
SAMPLE = struct.Struct(SAMPLE_FORMAT)
SAMPLE_SIZE = SAMPLE.size

SAMPLE_FIELDS = [
    "system_time_ns", "elapsed_time_s",
    "motor_position_unwrapped_deg", "motor_position_deg", "motor_velocity_rpm",
    "output_position_unwrapped_deg", "output_position_deg", "output_velocity_rpm",
    "motor_current_A", "actual_torque_Nm",
    "target_position_deg", "target_velocity_rpm", "target_torque_Nm",
    "position_error_deg", "velocity_error_rpm",
    "motor_position_raw", "output_position_raw",
    "twist_counts", "following_error_counts", "torque_est_mNm",
    "aux_position_raw", "position_counts_raw", "motor_position_sdo",
    "dc_link_voltage_mV", "warning_code", "working_counter", "seq",
    "controlword", "statusword", "error_code", "temperature_drive_C",
    "torque_actual_permille", "torque_ratio",
    "operation_mode", "cia402_state", "ethercat_state", "flags",
]

# numpy 结构化 dtype：一次性把整批样本解析成数组，
# 比逐条 struct.unpack 快一个数量级，GUI 在 1kHz 全速回传下也不会卡。
SAMPLE_DTYPE = np.dtype([
    ("system_time_ns", "<i8"), ("elapsed_time_s", "<f8"),
    ("motor_position_unwrapped_deg", "<f8"), ("motor_position_deg", "<f8"),
    ("motor_velocity_rpm", "<f8"),
    ("output_position_unwrapped_deg", "<f8"), ("output_position_deg", "<f8"),
    ("output_velocity_rpm", "<f8"),
    ("motor_current_A", "<f8"), ("actual_torque_Nm", "<f8"),
    ("target_position_deg", "<f8"), ("target_velocity_rpm", "<f8"),
    ("target_torque_Nm", "<f8"),
    ("position_error_deg", "<f8"), ("velocity_error_rpm", "<f8"),
    ("motor_position_raw", "<i4"), ("output_position_raw", "<i4"),
    ("twist_counts", "<i4"), ("following_error_counts", "<i4"),
    ("torque_est_mNm", "<i4"), ("aux_position_raw", "<i4"),
    ("position_counts_raw", "<i4"), ("motor_position_sdo", "<i4"),
    ("dc_link_voltage_mV", "<u4"), ("warning_code", "<u4"),
    ("working_counter", "<u4"), ("seq", "<u4"),
    ("controlword", "<u2"), ("statusword", "<u2"),
    ("error_code", "<u2"), ("temperature_drive_C", "<u2"),
    ("torque_actual_permille", "<i2"), ("torque_ratio", "<i2"),
    ("operation_mode", "i1"), ("cia402_state", "u1"),
    ("ethercat_state", "u1"), ("flags", "u1"),
])

assert SAMPLE_DTYPE.itemsize == SAMPLE_SIZE == 184, (
    f"线格式不一致: dtype={SAMPLE_DTYPE.itemsize} struct={SAMPLE_SIZE}，"
    "请同步检查 types.hpp 的 Sample 定义"
)


@dataclass
class Status:
    """Backend 推来的状态。字段名与 statusJson() 一一对应。"""
    raw: dict = field(default_factory=dict)

    def get(self, k, default=None):
        return self.raw.get(k, default)

    def __getattr__(self, k):
        try:
            return self.__dict__["raw"][k]
        except KeyError:
            raise AttributeError(k)


class IpcClient(QObject):
    connected = Signal()
    disconnected = Signal()
    connect_failed = Signal(str)

    telemetry = Signal(object)       # numpy 结构化数组
    status_changed = Signal(object)  # Status
    log_message = Signal(str, str)   # level, msg
    startup_step = Signal(str, bool, str)
    params_changed = Signal(object)  # dict: items/controllers
    recording_changed = Signal(object)
    ack = Signal(str, bool, str)     # cmd, ok, msg
    handshake_ok = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._sock = QLocalSocket(self)
        self._buf = bytearray()
        self._path = ""
        self._sock.connected.connect(self._on_connected)
        self._sock.disconnected.connect(self.disconnected)
        self._sock.readyRead.connect(self._on_ready_read)
        self._sock.errorOccurred.connect(self._on_error)

        # 自动重连：Backend 重启后 GUI 不需要用户手动干预
        self._retry = QTimer(self)
        self._retry.setInterval(2000)
        self._retry.timeout.connect(self._try_reconnect)
        self._auto_reconnect = False

        # 线格式是否可信。协议版本不匹配、或 pong 里的 sample_size 与本地
        # SAMPLE_SIZE 不一致时置 False——此时遥测数据按旧/新布局互相错位
        # 解读，画出来的曲线比没有数据更危险，必须整体拒收。
        self._wire_ok = True

    # ── 连接 ────────────────────────────────────────────────────────────
    def connect_to(self, path: str, auto_reconnect: bool = True):
        self._path = path
        self._auto_reconnect = auto_reconnect
        self._do_connect()
        if auto_reconnect:
            self._retry.start()

    def _do_connect(self):
        if self._sock.state() == QLocalSocket.UnconnectedState:
            self._buf.clear()
            self._sock.connectToServer(self._path)

    def _try_reconnect(self):
        if self._sock.state() == QLocalSocket.UnconnectedState:
            self._do_connect()

    def close(self):
        self._auto_reconnect = False
        self._retry.stop()
        self._sock.disconnectFromServer()

    def is_connected(self) -> bool:
        return self._sock.state() == QLocalSocket.ConnectedState

    def _on_connected(self):
        self._wire_ok = True             # 新连接，重新做一次线格式校验再信任
        self.connected.emit()
        self.send({"cmd": "ping"})       # 先握手校验线格式
        self.send({"cmd": "get_status"})
        self.send({"cmd": "get_params"})

    def _on_error(self, err):
        if self._sock.state() != QLocalSocket.ConnectedState:
            self.connect_failed.emit(self._sock.errorString())

    # ── 发送 ────────────────────────────────────────────────────────────
    def send(self, obj: dict):
        if not self.is_connected():
            self.log_message.emit("ERROR", f"未连接到 Backend，命令 {obj.get('cmd')} 未发出")
            return False
        data = (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")
        self._sock.write(data)
        self._sock.flush()
        return True

    # ── 接收 ────────────────────────────────────────────────────────────
    def _on_ready_read(self):
        self._buf.extend(self._sock.readAll().data())
        while True:
            if len(self._buf) < FRAME_HEADER.size:
                return
            magic, ftype, version, length = FRAME_HEADER.unpack_from(self._buf, 0)
            if magic != FRAME_MAGIC:
                # 流已错位，无法安全恢复：丢弃缓冲并告警，
                # 而不是继续解析出一堆垃圾数据
                self.log_message.emit("ERROR", "IPC 帧同步丢失，已重置接收缓冲")
                self._buf.clear()
                return
            if version != PROTOCOL_VERSION:
                # 帧头里的协议版本从未被真正比较过（终审 finding I2）——版本不匹配
                # 意味着往后每一帧都可能是按错误的线格式在解析，不能继续收，
                # 也不能自动重连（重连只会用同一份不匹配的 Backend 再连一次）。
                self.log_message.emit(
                    "ERROR",
                    f"IPC 协议版本不匹配：Backend 是 v{version}，GUI 期望 v{PROTOCOL_VERSION}。"
                    "请重新编译使两边一致后再连接，已断开并停止自动重连。")
                self._wire_ok = False
                self._auto_reconnect = False
                self._retry.stop()
                self._sock.disconnectFromServer()
                self._buf.clear()
                return
            total = FRAME_HEADER.size + length
            if len(self._buf) < total:
                return
            payload = bytes(self._buf[FRAME_HEADER.size:total])
            del self._buf[:total]

            if ftype == FRAME_TELEMETRY:
                self._handle_telemetry(payload)
            elif ftype == FRAME_JSON:
                self._handle_json(payload)

    def _handle_telemetry(self, payload: bytes):
        if not payload:
            return
        if not self._wire_ok:
            # 线格式已知不可信（版本不匹配或 pong 里的 sample_size 对不上），
            # 拒收遥测——按错误布局解析出的曲线比没有曲线更容易骗人。
            return
        if len(payload) % SAMPLE_SIZE != 0:
            self.log_message.emit(
                "ERROR",
                f"遥测帧长度 {len(payload)} 不是样本尺寸 {SAMPLE_SIZE} 的整数倍，已丢弃")
            return
        arr = np.frombuffer(payload, dtype=SAMPLE_DTYPE)
        self.telemetry.emit(arr)

    def _handle_json(self, payload: bytes):
        try:
            obj = json.loads(payload.decode("utf-8"))
        except Exception as e:
            self.log_message.emit("ERROR", f"JSON 解析失败: {e}")
            return

        ev = obj.get("ev")
        if ev == "status":
            self.status_changed.emit(Status(obj))
        elif ev == "log":
            self.log_message.emit(obj.get("level", "INFO"), obj.get("msg", ""))
        elif ev == "startup":
            self.startup_step.emit(obj.get("step", ""), bool(obj.get("ok")), obj.get("msg", ""))
        elif ev == "params":
            self.params_changed.emit(obj)
        elif ev == "recording":
            self.recording_changed.emit(obj)
        elif ev == "ack":
            self.ack.emit(obj.get("cmd", ""), bool(obj.get("ok")), obj.get("msg", ""))
        elif ev == "pong":
            remote = int(obj.get("sample_size", -1))
            if remote != SAMPLE_SIZE:
                self._wire_ok = False
                self.log_message.emit(
                    "ERROR",
                    f"线格式不匹配：Backend 的 Sample 是 {remote} 字节，"
                    f"GUI 期望 {SAMPLE_SIZE} 字节。请重新编译使两边一致，"
                    "否则曲线数据不可信。已拒收后续遥测帧。")
            else:
                self.handshake_ok.emit(obj)
