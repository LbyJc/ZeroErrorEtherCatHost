#!/usr/bin/env python3
"""端到端集成测试：拉起 mock backend，走完整操作流程，校验遥测数据。

不依赖 Qt，直接用裸 socket，这样可以在无显示环境（CI / SSH）里跑。
验证的是任务书里那条主线：
  启动主站 → Servo Enable → 选 CSV → 设目标 → 开始运行 → 看到真实转速 → 停止 → 撤使能
"""
from __future__ import annotations

import json
import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "gui"))

import numpy as np  # noqa: E402
from ipc_client import (  # noqa: E402
    FRAME_HEADER, FRAME_JSON, FRAME_MAGIC, FRAME_TELEMETRY, SAMPLE_DTYPE, SAMPLE_SIZE,
)

SOCK = "/tmp/ecjc-integration.sock"
failures = []


def check(cond, msg):
    print(f"  {'[ OK ]' if cond else '[FAIL]'} {msg}")
    if not cond:
        failures.append(msg)


class Client:
    def __init__(self, path):
        self.s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        for _ in range(50):
            try:
                self.s.connect(path)
                break
            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(0.1)
        else:
            raise RuntimeError(f"无法连接 {path}")
        self.s.settimeout(0.2)
        self.buf = bytearray()
        self.samples = []
        self.status = {}
        self.events = []
        self.acks = []

    def send(self, obj):
        self.s.sendall((json.dumps(obj) + "\n").encode())

    def pump(self, seconds=0.5):
        end = time.time() + seconds
        while time.time() < end:
            try:
                d = self.s.recv(1 << 20)
                if not d:
                    break
                self.buf.extend(d)
            except socket.timeout:
                pass
            self._parse()

    def _parse(self):
        while len(self.buf) >= FRAME_HEADER.size:
            magic, ftype, ver, length = FRAME_HEADER.unpack_from(self.buf, 0)
            assert magic == FRAME_MAGIC, f"帧魔数错误 0x{magic:08X}"
            total = FRAME_HEADER.size + length
            if len(self.buf) < total:
                return
            payload = bytes(self.buf[FRAME_HEADER.size:total])
            del self.buf[:total]
            if ftype == FRAME_TELEMETRY:
                assert len(payload) % SAMPLE_SIZE == 0
                self.samples.append(np.frombuffer(payload, dtype=SAMPLE_DTYPE))
            elif ftype == FRAME_JSON:
                o = json.loads(payload.decode())
                self.events.append(o)
                if o.get("ev") == "status":
                    self.status = o
                elif o.get("ev") == "ack":
                    self.acks.append(o)

    def all_samples(self):
        return np.concatenate(self.samples) if self.samples else np.empty(0, SAMPLE_DTYPE)

    def last_ack(self, cmd):
        for a in reversed(self.acks):
            if a.get("cmd") == cmd:
                return a
        return None


def main():
    exe = ROOT / "build" / "ecjc-backend"
    if not exe.is_file():
        print(f"找不到 {exe}，请先编译")
        return 1
    if os.path.exists(SOCK):
        os.unlink(SOCK)

    proc = subprocess.Popen(
        [str(exe), "--mock", "--config", str(ROOT / "config"), "--socket", SOCK],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        c = Client(SOCK)
        c.pump(0.5)

        print("\n[1] 握手与线格式")
        c.send({"cmd": "ping"})
        c.pump(0.3)
        pong = next((e for e in c.events if e.get("ev") == "pong"), None)
        check(pong is not None, "收到 pong")
        check(pong and pong["sample_size"] == SAMPLE_SIZE,
              f"Sample 尺寸两边一致 = {SAMPLE_SIZE}")

        print("\n[2] 未启动主站时应拒绝使能")
        c.send({"cmd": "servo_enable"})
        c.pump(0.3)
        a = c.last_ack("servo_enable")
        check(a is not None and not a["ok"], "EtherCAT 未 OP 时拒绝使能")
        check(a and "OP" in a.get("msg", ""), f"拒绝原因是人话: {a.get('msg','')[:40]}")

        print("\n[3] 启动主站")
        c.send({"cmd": "connect_bus"})
        c.pump(1.5)
        steps = [e for e in c.events if e.get("ev") == "startup"]
        check(len(steps) >= 8, f"收到 {len(steps)} 条启动进度")
        check(all(s["ok"] for s in steps), "所有启动步骤成功")
        names = [s["step"] for s in steps]
        for need in ("PDO Configured", "SAFEOP", "OP"):
            check(need in names, f"进度包含 {need}")
        check(c.status.get("ethercat") == "OP", f"EtherCAT 状态 = {c.status.get('ethercat')}")

        print("\n[4] 选择 CSV 模式")
        c.send({"cmd": "set_mode", "mode": "CSV"})
        c.pump(0.5)
        check(c.status.get("mode") == "CSV", f"模式 = {c.status.get('mode')}")
        check(c.status.get("mode_matched"), "0x6061 与 0x6060 一致")

        print("\n[5] 未使能时应拒绝运行")
        c.send({"cmd": "start_run"})
        c.pump(0.3)
        a = c.last_ack("start_run")
        check(a is not None and not a["ok"], "未使能时拒绝开始运行")

        print("\n[6] Servo Enable（应走 06→07→0F）")
        c.send({"cmd": "servo_enable"})
        c.pump(1.0)
        check(c.status.get("servo") == "Operation Enabled",
              f"CiA402 状态 = {c.status.get('servo')}")
        check(c.status.get("controlword") == 0x000F,
              f"控制字 = 0x{c.status.get('controlword', 0):04X}")

        print("\n[7] 设目标 100 rpm 并运行")
        c.send({"cmd": "set_trajectory", "type": "constant", "value": 100.0})
        c.send({"cmd": "set_target", "value": 100.0})
        c.pump(0.3)
        c.samples.clear()
        c.send({"cmd": "start_run"})
        c.pump(2.0)
        a = c.last_ack("start_run")
        check(a is not None and a["ok"], "开始运行被接受")
        check(c.status.get("running"), "状态显示 running")

        s = c.all_samples()
        check(len(s) > 50, f"收到 {len(s)} 个遥测样本")
        if len(s):
            vel = s["motor_velocity_rpm"][-20:].mean()
            check(abs(vel - 100.0) < 5.0, f"电机侧转速稳定在 {vel:.2f} rpm（目标 100）")

            out_vel = s["output_velocity_rpm"][-20:].mean()
            ratio = vel / out_vel if out_vel else 0
            check(abs(ratio - 121.0) < 1.0, f"电机/输出转速比 = {ratio:.2f}（减速比 121）")

            moved = s["output_position_unwrapped_deg"][-1] - s["output_position_unwrapped_deg"][0]
            check(moved > 0.01, f"输出侧角度在增长: {moved:.4f} deg")

            check(np.all(np.diff(s["seq"].astype(np.int64)) > 0), "seq 单调递增，无乱序")
            check(s["cia402_state"][-1] == 4, "样本里的 CiA402 状态 = Operation Enabled")
            check(s["flags"][-1] & 0x01, "样本 flags 里 running 位置位")

        print("\n[8] 运行中禁止切模式")
        c.send({"cmd": "set_mode", "mode": "CST"})
        c.pump(0.3)
        a = c.last_ack("set_mode")
        check(a is not None and not a["ok"], "运行中拒绝切换模式")

        print("\n[9] 数据采集")
        c.send({"cmd": "record_start", "test_name": "integration", "description": "自动测试"})
        c.pump(1.5)
        a = c.last_ack("record_start")
        check(a is not None and a["ok"], f"开始采集: {a and a.get('msg')}")
        rec = next((e for e in reversed(c.events) if e.get("ev") == "recording"), None)
        check(rec is not None and rec.get("active"), "采集状态为 active")
        c.send({"cmd": "record_stop"})
        c.pump(1.0)
        h5 = sorted((ROOT / "data").glob("integration_*.h5"))
        check(len(h5) > 0, f"生成了 HDF5 文件: {h5[-1].name if h5 else '无'}")

        print("\n[10] 停止运行（软停，伺服保持使能）")
        c.send({"cmd": "stop_run"})
        c.pump(1.5)
        check(not c.status.get("running"), "已停止运行")
        check(c.status.get("servo") == "Operation Enabled",
              "停止运行后伺服仍保持使能（停止 ≠ Servo Disable）")
        s2 = c.all_samples()
        if len(s2):
            check(abs(s2["motor_velocity_rpm"][-1]) < 5.0,
                  f"转速已归零: {s2['motor_velocity_rpm'][-1]:.2f} rpm")

        print("\n[11] 撤使能")
        c.send({"cmd": "servo_disable"})
        c.pump(1.0)
        check(c.status.get("servo") != "Operation Enabled",
              f"已撤使能，当前 {c.status.get('servo')}")

        print("\n[12] Homing 应给出明确的不支持提示")
        c.send({"cmd": "homing"})
        c.pump(0.3)
        a = c.last_ack("homing")
        check(a is not None and not a["ok"], "Homing 被拒绝")
        check(a and "0x6502" in a.get("msg", ""),
              f"拒绝理由指明了硬件依据: {a.get('msg','')[:50]}")

        print("\n[13] 抖动统计存在")
        check(c.status.get("cycles", 0) > 100, f"已运行 {c.status.get('cycles')} 个周期")
        check("jitter_max_us" in c.status, f"最大抖动 {c.status.get('jitter_max_us')} µs")

        print("\n[14] 一键实验线B 全流程（mock 端到端）：record_start 实验字段 "
              "→ HDF5 attrs → A.1 CSV 导出")
        import glob
        import tempfile

        import h5py

        c.send({"cmd": "fault_reset"})
        c.pump(0.3)
        c.send({"cmd": "servo_enable"})
        c.pump(1.0)
        check(c.status.get("servo") == "Operation Enabled",
              f"重新使能成功，CiA402 状态 = {c.status.get('servo')}")
        c.send({"cmd": "set_mode", "mode": "CSV"})
        c.pump(0.3)
        c.send({"cmd": "set_trajectory", "type": "constant", "value": 30.0})
        c.send({"cmd": "set_target", "value": 30.0})
        c.pump(0.3)

        d = tempfile.mkdtemp(prefix="ecjc-lineB-")
        c.send({
            "cmd": "record_start",
            "out_dir": d,
            "sample_id": "A01",
            "baseline_stage": "life_node",
            "life_hours": 100,
            "test_item": "TE",
            "rep": 1,
            "load_percent_Tr": 0,
            "speed_rpm_target": 5,
            "test_name": "A01_lineB",
        })
        c.pump(1.0)
        a = c.last_ack("record_start")
        check(a is not None and a["ok"], f"线B record_start 被接受: {a and a.get('msg')}")

        c.send({"cmd": "start_run"})
        c.pump(2.0)
        a = c.last_ack("start_run")
        check(a is not None and a["ok"], "线B start_run 被接受")
        check(c.status.get("running"), "线B 状态显示 running")

        c.send({"cmd": "stop_run"})
        c.pump(0.3)
        c.send({"cmd": "record_stop"})
        c.pump(1.0)

        h5s = sorted(glob.glob(os.path.join(d, "*.h5")))
        check(len(h5s) == 1, f"h5 落盘到 out_dir: {h5s}")

        def _attr(attrs, key):
            v = attrs.get(key)
            return v.decode() if isinstance(v, bytes) else v

        if h5s:
            with h5py.File(h5s[0]) as f:
                attrs = f["experiment"].attrs
                check(_attr(attrs, "sample_id") == "A01", "sample_id 写入 HDF5 attrs")
                check(_attr(attrs, "test_item") == "TE", "test_item 写入 HDF5 attrs")
                check(float(attrs.get("life_hours")) == 100.0, "life_hours 写入 HDF5 attrs")

            sys.path.insert(0, str(ROOT / "tools"))
            import experiment_naming as en  # noqa: E402
            import h5_to_csv  # noqa: E402

            csv_name = en.csv_filename("A01", 100, "TE", 0, 5, 1)
            out_csv = os.path.join(d, csv_name)
            h5_to_csv.export_a1(h5s[0], out_csv)
            check(os.path.exists(out_csv), f"A.1 CSV 生成: {csv_name}")
            meta_path = os.path.splitext(out_csv)[0] + ".meta.yaml"
            check(os.path.exists(meta_path), "同名 .meta.yaml 生成")
            if os.path.exists(meta_path):
                import yaml
                with open(meta_path) as fp:
                    meta = yaml.safe_load(fp)
                check("empty_columns" in meta, ".meta.yaml 含 empty_columns")

            if os.path.exists(out_csv):
                import csv as _csv
                with open(out_csv) as fp:
                    rows = list(_csv.reader(fp))
                for col in en.EMPTY_COLUMNS:
                    check(col in rows[0], f"留空列 {col} 在表头")
                if "temperature_joint_C" in rows[0]:
                    i = rows[0].index("temperature_joint_C")
                    check(rows[1][i] == "", "留空列值为空字段（不是 NA 或 None）")

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        if os.path.exists(SOCK):
            os.unlink(SOCK)

    print("\n" + "=" * 60)
    if failures:
        print(f"{len(failures)} 项失败:")
        for f in failures:
            print("  -", f)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
