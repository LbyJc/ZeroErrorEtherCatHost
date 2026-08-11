#!/usr/bin/env python3
"""真机联调用的命令行客户端。

与 GUI 用的是同一条 IPC 协议、同一套解析代码，
所以这里跑通就等价于 GUI 那条路跑通。
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
    FRAME_HEADER, FRAME_JSON, FRAME_MAGIC, FRAME_TELEMETRY, SAMPLE_DTYPE, SAMPLE_SIZE,
)

DEFAULT_SOCK = "/run/ethercat-joint-control/control.sock"


class Client:
    def __init__(self, path):
        self.s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.s.connect(path)
        self.s.settimeout(0.2)
        self.buf = bytearray()
        self.samples = []
        self.status = {}
        self.events = []

    def send(self, **obj):
        self.s.sendall((json.dumps(obj, ensure_ascii=False) + "\n").encode())

    def pump(self, seconds=1.0, show=True):
        end = time.time() + seconds
        seen = len(self.events)
        while time.time() < end:
            try:
                d = self.s.recv(1 << 20)
                if not d:
                    break
                self.buf.extend(d)
            except socket.timeout:
                pass
            self._parse()
        if show:
            for e in self.events[seen:]:
                ev = e.get("ev")
                if ev == "startup":
                    print(f"  [{'✓' if e['ok'] else '✗'}] {e['step']}"
                          + (f"  — {e['msg']}" if e.get("msg") else ""))
                elif ev == "log":
                    print(f"  [{e['level']}] {e['msg']}")
                elif ev == "ack" and not e["ok"]:
                    print(f"  [ACK 失败] {e['cmd']}: {e['msg']}")
                elif ev == "pong":
                    print(f"  握手 OK: Sample={e['sample_size']}B "
                          f"协议v{e['protocol']} 后端v{e['version']}")

    def _parse(self):
        while len(self.buf) >= FRAME_HEADER.size:
            magic, ftype, ver, length = FRAME_HEADER.unpack_from(self.buf, 0)
            if magic != FRAME_MAGIC:
                self.buf.clear()
                return
            total = FRAME_HEADER.size + length
            if len(self.buf) < total:
                return
            payload = bytes(self.buf[FRAME_HEADER.size:total])
            del self.buf[:total]
            if ftype == FRAME_TELEMETRY:
                self.samples.append(np.frombuffer(payload, dtype=SAMPLE_DTYPE))
            elif ftype == FRAME_JSON:
                o = json.loads(payload.decode())
                self.events.append(o)
                if o.get("ev") == "status":
                    self.status = o

    def all_samples(self):
        return np.concatenate(self.samples) if self.samples else np.empty(0, SAMPLE_DTYPE)

    def show_status(self):
        s = self.status
        print(f"  EtherCAT={s.get('ethercat')}  Slave在线={s.get('slave_online')} "
              f"操作态={s.get('slave_operational')}  WC={s.get('wc')}/"
              f"{['ZERO','INCOMPLETE','COMPLETE'][s.get('wc_state',0)]}")
        print(f"  Servo={s.get('servo')}  SW=0x{s.get('statusword',0):04X} "
              f"CW=0x{s.get('controlword',0):04X}  Err=0x{s.get('error_code',0):04X}")
        print(f"  Mode={s.get('mode')} 0x6061={s.get('mode_display')} "
              f"匹配={s.get('mode_matched')}  周期={s.get('cycle_us')}us")
        print(f"  抖动 当前={s.get('jitter_us',0):.1f} 最大={s.get('jitter_max_us',0):.1f} "
              f"均值={s.get('jitter_mean_us',0):.1f} us  错过周期={s.get('deadline_miss')}"
              f"/{s.get('cycles')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--socket", default=DEFAULT_SOCK)
    ap.add_argument("action", nargs="+",
                    help="connect | status | enable | disable | fault_reset | "
                         "mode:CSV | target:100 | run | stop | wait:3 | sample")
    args = ap.parse_args()

    c = Client(args.socket)
    c.send(cmd="ping")
    c.pump(0.5)

    for act in args.action:
        print(f"\n▶ {act}")
        if act == "connect":
            c.send(cmd="connect_bus"); c.pump(15)
        elif act == "disconnect":
            c.send(cmd="disconnect_bus"); c.pump(5)
        elif act == "status":
            c.send(cmd="get_status"); c.pump(0.5, show=False); c.show_status()
        elif act == "enable":
            c.send(cmd="servo_enable"); c.pump(3)
        elif act == "disable":
            c.send(cmd="servo_disable"); c.pump(3)
        elif act == "fault_reset":
            c.send(cmd="fault_reset"); c.pump(2)
        elif act.startswith("mode:"):
            c.send(cmd="set_mode", mode=act.split(":")[1]); c.pump(1)
        elif act.startswith("target:"):
            v = float(act.split(":")[1])
            c.send(cmd="set_trajectory", type="constant", value=v)
            c.send(cmd="set_target", value=v)
            c.pump(1)
        elif act == "run":
            c.samples.clear()
            c.send(cmd="start_run"); c.pump(2)
        elif act == "stop":
            c.send(cmd="stop_run"); c.pump(3)
        elif act.startswith("wait:"):
            c.pump(float(act.split(":")[1]), show=True)
        elif act == "sample":
            s = c.all_samples()
            if len(s) == 0:
                print("  （无遥测样本）")
                continue
            n = min(len(s), 200)
            tail = s[-n:]
            print(f"  样本数={len(s)}  取末尾 {n} 个求均值：")
            print(f"    电机侧位置 {tail['motor_position_deg'][-1]:10.3f} deg  "
                  f"多圈 {tail['motor_position_unwrapped_deg'][-1]:12.2f} deg  "
                  f"raw={int(tail['motor_position_raw'][-1]):,}")
            print(f"    输出侧位置 {tail['output_position_deg'][-1]:10.3f} deg  "
                  f"多圈 {tail['output_position_unwrapped_deg'][-1]:12.2f} deg  "
                  f"raw={int(tail['output_position_raw'][-1]):,}")
            print(f"    电机转速   {tail['motor_velocity_rpm'].mean():10.3f} rpm")
            print(f"    输出转速   {tail['output_velocity_rpm'].mean():10.4f} rpm")
            ov = tail['output_velocity_rpm'].mean()
            if abs(ov) > 1e-6:
                print(f"    实测速比   {tail['motor_velocity_rpm'].mean()/ov:10.2f} : 1")
            print(f"    电流       {tail['motor_current_A'].mean():10.3f} A")
            print(f"    力矩       {tail['actual_torque_Nm'].mean():10.3f} Nm")
            print(f"    SW=0x{int(tail['statusword'][-1]):04X} "
                  f"CW=0x{int(tail['controlword'][-1]):04X} "
                  f"模式={int(tail['operation_mode'][-1])} "
                  f"WC={int(tail['working_counter'][-1])}")
        elif act == "ratio":
            # 独立的减速比测量：直接用两个编码器的**原始计数增量**，
            # 不经过 scaling 里的 gear_ratio，否则算出来必然等于配置值（同义反复）。
            s = c.all_samples()
            if len(s) < 100:
                print("  样本不足")
                continue
            d2240 = int(s["motor_position_raw"][-1]) - int(s["motor_position_raw"][0])
            d6064 = int(s["output_position_raw"][-1]) - int(s["output_position_raw"][0])
            dt = (int(s["system_time_ns"][-1]) - int(s["system_time_ns"][0])) / 1e9
            print(f"  窗口 {dt:.3f} s, {len(s)} 样本")
            print(f"    Δ0x2240 = {d2240:>12,} counts  → {d2240/dt:>12.1f} counts/s")
            print(f"    Δ0x6064 = {d6064:>12,} counts  → {d6064/dt:>12.1f} counts/s")
            if d6064:
                r = d2240 / d6064
                print(f"    Δ2240/Δ6064 = {r:.3f}   (121/4 = 30.25, 120/4 = 30.00)")
                print(f"    → 电机侧转速 {d2240/dt/131072*60:.2f} rpm  "
                      f"(按 2^17 counts/rev)")
                print(f"    → 输出侧转速 {d6064/dt/524288*60:.4f} rpm  "
                      f"(按 2^19 counts/rev)")
                print(f"    → 实测减速比 {(d2240/131072)/(d6064/524288):.2f} : 1")
        else:
            print(f"  未知动作: {act}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
