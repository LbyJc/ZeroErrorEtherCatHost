#!/usr/bin/env python3
"""从遥测流直接测周期抖动。

比读后端的累计统计更可信：后端的 max 是自 RT 线程启动以来的累计值，
会混进启动瞬态；这里只统计一个指定窗口内的样本，
而且用的是 RT 循环每拍自己打的 system_time_ns，就是真实的周期间隔。
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--socket", default="/run/ethercat-joint-control/control.sock")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.connect(args.socket)
    s.settimeout(0.2)
    buf = bytearray()
    chunks = []
    status = {}

    end = time.time() + args.seconds
    while time.time() < end:
        try:
            buf.extend(s.recv(1 << 20))
        except socket.timeout:
            pass
        while len(buf) >= 12:
            m, t, v, l = FRAME_HEADER.unpack_from(buf, 0)
            if m != FRAME_MAGIC:
                buf.clear(); break
            if len(buf) < 12 + l:
                break
            p = bytes(buf[12:12 + l]); del buf[:12 + l]
            if t == FRAME_TELEMETRY:
                chunks.append(np.frombuffer(p, dtype=SAMPLE_DTYPE))
            elif t == FRAME_JSON:
                o = json.loads(p.decode())
                if o.get("ev") == "status":
                    status = o

    if not chunks:
        print("没有收到遥测")
        return 1
    a = np.concatenate(chunks)

    # 丢包检查：seq 必须连续，否则统计的"间隔"其实跨越了丢失的样本
    seq = a["seq"].astype(np.int64)
    gaps = np.diff(seq)
    lost = int((gaps[gaps > 1] - 1).sum()) if len(gaps) else 0

    t = a["system_time_ns"].astype(np.int64)
    dt = np.diff(t)[gaps == 1] / 1000.0        # 只取连续样本，单位 µs
    if len(dt) < 100:
        print("有效样本太少")
        return 1

    cyc = float(status.get("cycle_us", 1000))
    jit = dt - cyc

    print(f"周期设定      : {cyc:.0f} µs")
    print(f"样本数        : {len(a):,}（连续间隔 {len(dt):,}，IPC 丢样 {lost}）")
    print(f"窗口时长      : {(t[-1]-t[0])/1e9:.1f} s")
    print()
    print(f"实测周期均值  : {dt.mean():9.3f} µs")
    print(f"标准差        : {dt.std():9.3f} µs")
    print(f"最小 / 最大   : {dt.min():9.3f} / {dt.max():9.3f} µs")
    print()
    print("周期偏差分位数（|实测 - 设定|）:")
    ad = np.abs(jit)
    for q in (50, 90, 99, 99.9, 99.99):
        print(f"  p{q:<6}      : {np.percentile(ad, q):9.3f} µs")
    print(f"  最大        : {ad.max():9.3f} µs")
    print()
    for th in (0.5, 1.0, 2.0):
        n = int((ad > cyc * th).sum())
        print(f"  偏差 > {th*100:>3.0f}% 周期 : {n:>6} 次 "
              f"({n/len(ad)*100:.4f}%)")
    print()
    print(f"后端累计统计（含启动瞬态）: 最大 {status.get('jitter_max_us',0):.1f} µs, "
          f"错过 {status.get('deadline_miss')}/{status.get('cycles')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
