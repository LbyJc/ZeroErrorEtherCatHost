#!/usr/bin/env python3
"""输出侧编码器分辨率的物理验证。

软件里所有 rpm / 角度数字都建立在"输出侧 = 524288 counts/rev"这个**未经验证**的
假设上（它来自厂商 demo 的 ENCODER_RES）。两侧分辨率是绑定的：
若输出侧实为 2^18，电机侧同步变 2^16，所有 rpm 数值翻倍。

验证思路：让输出侧**恰好走半圈**（按假设值 = 262144 个 0x6064 计数）。

    假设成立(2^19) → 记号停在对面   约 180°
    假设是2^18     → 记号转满一圈   回到原位

不能走整圈：524288 counts 在 2^19 下是 1 圈、2^18 下是 2 圈，
记号两种情况都回原位，测了等于没测。

停车点按**计数**闭环，不靠掐时间——这样与速度增益、斜坡都无关。
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "gui"))

import numpy as np  # noqa: E402
from ipc_client import (  # noqa: E402
    FRAME_HEADER, FRAME_JSON, FRAME_MAGIC, FRAME_TELEMETRY, SAMPLE_DTYPE,
)

SOCK = "/run/ethercat-joint-control/control.sock"
ASSUMED_CPR = 524288          # 待验证的输出侧分辨率
TARGET_COUNTS = ASSUMED_CPR // 2   # 半圈
GEAR_RATIO = 121.0            # config/scaling.yaml gear_ratio


class C:
    def __init__(self, path):
        self.s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.s.connect(path)
        self.s.settimeout(0.1)
        self.buf = bytearray()
        self.last = None
        self.status = {}

    def send(self, **o):
        self.s.sendall((json.dumps(o, ensure_ascii=False) + "\n").encode())

    def pump(self, secs):
        end = time.time() + secs
        while time.time() < end:
            try:
                self.buf.extend(self.s.recv(1 << 20))
            except socket.timeout:
                pass
            while len(self.buf) >= 12:
                m, t, v, l = FRAME_HEADER.unpack_from(self.buf, 0)
                if m != FRAME_MAGIC:
                    self.buf.clear(); break
                if len(self.buf) < 12 + l:
                    break
                p = bytes(self.buf[12:12 + l]); del self.buf[:12 + l]
                if t == FRAME_TELEMETRY:
                    a = np.frombuffer(p, dtype=SAMPLE_DTYPE)
                    if len(a):
                        self.last = a[-1]
                elif t == FRAME_JSON:
                    o = json.loads(p.decode())
                    if o.get("ev") == "status":
                        self.status = o


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--socket", default=SOCK)
    ap.add_argument("--motor-rpm", type=float, default=200.0,
                    help="电机侧转速，默认 200 rpm（输出侧约 1.65 rpm，半圈约 18 秒）")
    args = ap.parse_args()
    # 2026-08-13 起 set_target 的语义是**输出侧 rpm**
    # （scaling.yaml target_velocity_is_motor_side=false），
    # 脚本参数保持电机侧习惯，在这里换算一次。
    target_out_rpm = args.motor_rpm / GEAR_RATIO

    c = C(args.socket)
    c.send(cmd="get_status"); c.pump(0.5)
    if c.status.get("ethercat") != "OP":
        print(f"EtherCAT 未进 OP（当前 {c.status.get('ethercat')}），请先启动主站")
        return 1

    print("准备：切 CSV、使能伺服…")
    c.send(cmd="set_mode", mode="CSV")
    c.send(cmd="servo_enable")
    c.pump(2.5)
    if c.status.get("servo") != "Operation Enabled":
        print(f"使能失败，当前 {c.status.get('servo')}")
        return 1

    c.send(cmd="set_trajectory", type="constant", value=target_out_rpm)
    c.send(cmd="set_target", value=target_out_rpm)
    c.pump(0.5)
    while c.last is None:
        c.pump(0.3)
    start_raw = int(c.last["output_position_raw"])
    print(f"起始 0x6064 = {start_raw:,}")
    print(f"目标位移   = {TARGET_COUNTS:,} counts（假设 2^19 下的半圈）")
    print(f"转速       = 电机侧 {args.motor_rpm} rpm\n")

    c.send(cmd="start_run")
    t0 = time.time()
    stopped = False
    # controller.yaml 里的 velocity_rate_rpm_per_s（现为输出侧 1.65 rpm/s），
    # 惯走预测用电机侧转速，换算回电机侧
    decel = 1.65 * GEAR_RATIO

    while time.time() - t0 < 180:
        c.pump(0.2)
        if c.last is None:
            continue
        d = int(c.last["output_position_raw"]) - start_raw
        rpm = float(c.last["motor_velocity_rpm"])
        # 预测减速过程还会走多远：v/2 × t_decel，t_decel = v / 减速度
        cps = float(c.last["output_velocity_rpm"]) / 60.0 * ASSUMED_CPR
        t_dec = abs(rpm) / decel if decel > 0 else 0
        coast = abs(cps) * t_dec / 2.0
        remain = TARGET_COUNTS - d
        print(f"\r  已走 {d:>8,} / {TARGET_COUNTS:,} counts  "
              f"({d/ASSUMED_CPR*360:6.1f}°假设)  电机 {rpm:6.1f} rpm  "
              f"预计惯走 {coast:6.0f}", end="", flush=True)
        if not stopped and remain <= coast:
            c.send(cmd="stop_run")
            stopped = True
            print("\n  → 已发停止指令，等待减速…")
        if stopped and abs(rpm) < 0.5:
            break

    c.pump(1.5)
    end_raw = int(c.last["output_position_raw"])
    d = end_raw - start_raw
    c.send(cmd="servo_disable")
    c.pump(1.0)

    print("\n" + "=" * 62)
    print(f"实际位移 Δ0x6064 = {d:,} counts")
    print(f"  与目标半圈相差 {d - TARGET_COUNTS:+,} counts "
          f"({(d-TARGET_COUNTS)/ASSUMED_CPR*360:+.2f}° 假设值)")
    print("=" * 62)
    print("\n请看输出法兰上的记号，它现在应该在：\n")
    print(f"  ┌ 若输出侧 = 524288 (2^19)  →  转过 {d/524288*360:7.2f}°  "
          f"≈ 半圈，记号在【对面】")
    print(f"  └ 若输出侧 = 262144 (2^18)  →  转过 {d/262144*360:7.2f}°  "
          f"≈ 整圈，记号【回到原位】")
    print("\n（若是回到原位，则电机侧同步变 2^16，所有 rpm 数值需翻倍，")
    print("  电机侧 100 rpm 对应的 0x60FF 从 7207 变为 3604）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
