#!/usr/bin/env python3
"""慢客户端 / IPC 韧性行为验证脚本（终审 finding I4）。

背景：C1 修的是"踢出分支里 log() 在 evictClient() 之前调用"导致的无界递归
（log()→sendJson()→sendFrame() 广播→踢出中的 fd 还在快照里→再次超时→再次
满足踢出条件→再次 log()……每层约 200ms 直到 SIGSEGV）。这条递归可能一直
把"踢出路径本身走没走完"这件事掩盖掉了——光看单元测试看不出来，必须真的
起一个后端、真的喂出一个会被踢的慢客户端、真的等它被踢，才能确认：
  1. 后端进程不会因为踢出这个动作本身而崩溃（C1）；
  2. 踢出前后协议流不失步（沿用 Task 5 的撕裂场景断言）；
  3. 唯一客户端被踢出时不会立即 stopRun，而是进入重连宽限期（C3）；
  4. 宽限期内有新客户端连入会取消倒计时，run 不受影响；
  5. （--full）真的没人重连时，宽限期到期后 run 最终还是会被停止。

不接入 ctest：kSlowClientEvictAfterDrops=300 × 最多 200ms/次，触发一次真实
踢出本身就需要约 60 秒的墙钟时间（这是设计常量决定的下限，不是脚本慢），
默认运行（Phase A + Phase B）约 70~100 秒，--full 再加约 90~120 秒（Phase C
需要重新触发一次踢出 + 等满整个重连宽限期）。不适合塞进每次 build 的
自动化测试，但**修完 C1 之后必须手动跑一遍**，见任务书 finding I4。

用法：
    /home/tyy/miniconda3/envs/zeroError/bin/python tests/verify_slow_client.py
    /home/tyy/miniconda3/envs/zeroError/bin/python tests/verify_slow_client.py --full

要求先 `cmake --build build -j` 生成 build/ecjc-backend；全程 --mock，不碰
EtherCAT 硬件。
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "gui"))

from ipc_client import FRAME_HEADER, FRAME_JSON, FRAME_MAGIC  # noqa: E402

# 必须与 backend/include/ecjc/ipc_server.hpp 里的同名常量保持一致——
# 这里只是验证脚本用来估算等待预算，两边不同步的话脚本会等错时间，
# 不影响后端的真实行为。
K_SLOW_CLIENT_EVICT_AFTER_DROPS = 300   # 单条连接终生丢帧数达到这个值才被踢
K_RECONNECT_GRACE_SEC = 30              # 最后一个客户端因被踢而消失后的宽限期

EXE = ROOT / "build" / "ecjc-backend"

failures: list[str] = []


def check(cond: bool, msg: str) -> bool:
    print(f"  {'[ OK ]' if cond else '[FAIL]'} {msg}")
    if not cond:
        failures.append(msg)
    return cond


def scan_for_desync(data: bytes) -> tuple[bool, str]:
    """按长度前缀协议扫描一段原始字节，检查每个帧头处的 magic 是否始终有效。

    协议是纯长度前缀流：一旦有一帧发送到一半就断（Torn），后续所有字节的
    帧边界都会被错位解析，magic 校验必然在某处失败——这正是评审最关心的
    不变量：只要客户端曾经观察到一段"看起来像帧头但 magic 不对"的字节，
    就说明协议撕裂没有被服务端正确兜住（该整条断开、不该只丢帧）。

    允许的"正常"结束方式：
      - 数据在一个完整帧的边界处结束（frames 全部解析完，无残留）；
      - 数据在残留字节不足一个帧头，或声明长度超过剩余字节处结束——
        这代表流是在这里被截断的（我们主动停止了读取，或对端半关闭），
        不代表失步。
    """
    i, n, frames = 0, len(data), 0
    while i + FRAME_HEADER.size <= n:
        magic, _ftype, _ver, length = FRAME_HEADER.unpack_from(data, i)
        if magic != FRAME_MAGIC:
            return False, (f"帧头 magic 错误于偏移 {i}: 0x{magic:08X}"
                            f"（期望 0x{FRAME_MAGIC:08X}），协议流已失步")
        total = FRAME_HEADER.size + length
        if i + total > n:
            return True, (f"收到 {n} 字节，成功解出 {frames} 个完整帧，"
                           f"尾部残留 {n - i} 字节未消费（截断的尾巴，允许），连接仍然存活")
        i += total
        frames += 1
    return True, f"收到 {n} 字节，成功解出 {frames} 个完整帧，尾部残留 {n - i} 字节未消费"


def raw_connect(path: str, timeout: float = 0.2) -> socket.socket:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    for _ in range(50):
        try:
            s.connect(path)
            break
        except (FileNotFoundError, ConnectionRefusedError):
            time.sleep(0.1)
    else:
        raise RuntimeError(f"无法连接 {path}")
    s.settimeout(timeout)
    return s


class Observer:
    """行为良好、持续 pump() 的客户端——对照组，也用作"重连"的那一方。"""

    def __init__(self, path: str):
        self.s = raw_connect(path)
        self.buf = bytearray()
        self.events: list[dict] = []
        self.status: dict = {}

    def send(self, obj: dict) -> None:
        self.s.sendall((json.dumps(obj) + "\n").encode())

    def pump(self, seconds: float = 0.5) -> None:
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

    def _parse(self) -> None:
        while len(self.buf) >= FRAME_HEADER.size:
            magic, ftype, _ver, length = FRAME_HEADER.unpack_from(self.buf, 0)
            assert magic == FRAME_MAGIC, f"帧魔数错误 0x{magic:08X}"
            total = FRAME_HEADER.size + length
            if len(self.buf) < total:
                return
            payload = bytes(self.buf[FRAME_HEADER.size:total])
            del self.buf[:total]
            if ftype == FRAME_JSON:
                o = json.loads(payload.decode())
                self.events.append(o)
                if o.get("ev") == "status":
                    self.status = o

    def close(self) -> None:
        try:
            self.s.close()
        except OSError:
            pass


def spawn_backend(sock_path: str, log_path: str):
    if os.path.exists(sock_path):
        os.unlink(sock_path)
    log_f = open(log_path, "w")
    proc = subprocess.Popen(
        [str(EXE), "--mock", "--config", str(ROOT / "config"), "--socket", sock_path],
        stdout=log_f, stderr=subprocess.STDOUT)
    return proc, log_f


def read_log(log_path: str) -> str:
    try:
        return Path(log_path).read_text(errors="replace")
    except FileNotFoundError:
        return ""


def stop_backend(proc: subprocess.Popen, log_f, sock_path: str) -> None:
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
    log_f.close()
    if os.path.exists(sock_path):
        os.unlink(sock_path)


def drive_a_run(c: Observer) -> None:
    """走一遍最短路径把 mock 后端带到"正在运行"状态，供两个客户端观测。"""
    c.pump(0.3)
    c.send({"cmd": "connect_bus"}); c.pump(2.0)
    c.send({"cmd": "set_mode", "mode": "CSV"}); c.pump(0.3)
    c.send({"cmd": "servo_enable"}); c.pump(1.0)
    c.send({"cmd": "set_trajectory", "type": "constant", "value": 50.0})
    c.send({"cmd": "set_target", "value": 50.0})
    c.send({"cmd": "start_run"}); c.pump(1.0)


# ── Phase A：慢客户端不应挡住其他客户端连接 ──────────────────────────────
def phase_a() -> None:
    print("\n[Phase A] 慢客户端存在时，其它客户端仍应正常可用")
    sock, log = "/tmp/ecjc-verify-slow-A.sock", "/tmp/ecjc-verify-slow-A.log"
    proc, log_f = spawn_backend(sock, log)
    driver = slow = third = None
    try:
        driver = Observer(sock)
        drive_a_run(driver)
        check(driver.status.get("running") is True, "驱动客户端确认 run 已开始")

        # 只连不读的慢客户端：模拟冻结的 GUI / 休眠的笔记本。
        slow = raw_connect(sock)

        t0 = time.time()
        third = Observer(sock)
        third.send({"cmd": "get_status"})
        third.pump(2.0)
        elapsed = time.time() - t0
        check(any(e.get("ev") == "status" for e in third.events),
              f"第三个客户端在慢客户端存在期间仍能连上并拿到 get_status 响应（{elapsed:.2f}s）")

        for _ in range(20):
            driver.pump(0.5)
            if driver.status.get("slow_client_drops", 0) > 0:
                break
        check(driver.status.get("slow_client_drops", 0) > 0,
              f"status.slow_client_drops 已增长: {driver.status.get('slow_client_drops')}")

        log_text = read_log(log)
        check("客户端读取过慢" in log_text or "长期无法发送" in log_text,
              "后端日志出现慢客户端相关 WARNING")
        check(proc.poll() is None, "后端进程仍在正常运行（Phase A 未崩溃）")
    finally:
        for c in (driver, third):
            if c is not None:
                c.close()
        if slow is not None:
            try:
                slow.close()
            except OSError:
                pass
        stop_backend(proc, log_f, sock)


# ── Phase B：唯一客户端被踢 → 重连宽限期，不应立即 stopRun ─────────────────
def phase_b() -> bytes:
    print("\n[Phase B] 唯一客户端被踢出 → 应进入重连宽限期而不是立即停止运行（finding C3）")
    sock, log = "/tmp/ecjc-verify-slow-B.sock", "/tmp/ecjc-verify-slow-B.log"
    proc, log_f = spawn_backend(sock, log)
    g = None
    try:
        g = raw_connect(sock)

        def g_send(o):
            g.sendall((json.dumps(o) + "\n").encode())

        def g_drain(seconds):
            end = time.time() + seconds
            while time.time() < end:
                try:
                    g.recv(1 << 20)
                except socket.timeout:
                    pass

        g_drain(0.3)
        g_send({"cmd": "connect_bus"}); g_drain(2.0)
        g_send({"cmd": "set_mode", "mode": "CSV"}); g_drain(0.3)
        g_send({"cmd": "servo_enable"}); g_drain(1.0)
        g_send({"cmd": "set_trajectory", "type": "constant", "value": 50.0})
        g_send({"cmd": "set_target", "value": 50.0})
        g_send({"cmd": "start_run"}); g_drain(1.0)

        est_wait = K_SLOW_CLIENT_EVICT_AFTER_DROPS * 0.2
        print(f"  停止读取（模拟 GUI 冻结），轮询后端日志等待被踢出（预计约 {est_wait:.0f}s）...")
        t0 = time.time()
        evict_seen = False
        while time.time() - t0 < est_wait + 40:
            time.sleep(2.0)
            txt = read_log(log)
            if "判定为长期失联，主动断开" in txt or "协议流已撕裂" in txt:
                evict_seen = True
                break
        elapsed = time.time() - t0
        check(evict_seen, f"后端日志出现踢出该慢客户端的 WARNING，耗时 {elapsed:.1f}s")
        check(proc.poll() is None,
              "后端进程仍在正常运行（被踢出这个动作本身没有让它崩溃——验证 C1 递归修复）")

        # 把内核缓冲区里攒的字节一次性读出来：应该很快见到 EOF
        # （服务端早就 shutdown() 了这个 fd），且这些字节按协议扫描不该失步。
        g.settimeout(2.0)
        raw_history = bytearray()
        saw_eof = False
        try:
            while True:
                d = g.recv(1 << 20)
                if d == b"":
                    saw_eof = True
                    break
                raw_history += d
        except socket.timeout:
            pass
        check(saw_eof, "慢客户端最终收到 EOF（服务端已 shutdown 该 fd，evictClient 生效）")
        ok, detail = scan_for_desync(bytes(raw_history))
        check(ok, f"慢客户端积压的字节流协议扫描无失步：{detail}")

        # 此刻 g 是被踢出前**唯一**的客户端。按 C3 的设计，这不该立即
        # stopRun()，而是进入宽限期。用一个新客户端探测状态——这次连接本身
        # 就是"重连"，正对应 gui/ipc_client.py 2 秒自动重连、GUI 解冻后
        # 实验应当不受影响这条主线场景。
        reconnect = Observer(sock)
        reconnect.send({"cmd": "get_status"})
        reconnect.pump(1.0)
        check(reconnect.status.get("running") is True,
              "宽限期内重连后 run 仍在跑（没有被立即 stopRun）")
        reconnect.close()

        log_text = read_log(log)
        check("重连宽限期" in log_text, "后端日志出现「重连宽限期」提示")
        stop_idx = log_text.find("已停止运行")
        grace_idx = log_text.find("重连宽限期")
        check(stop_idx == -1 or (grace_idx != -1 and grace_idx < stop_idx),
              "没有在宽限期消息之前就已经 stopRun（顺序正确）")

        return bytes(raw_history)
    finally:
        if g is not None:
            try:
                g.close()
            except OSError:
                pass
        stop_backend(proc, log_f, sock)


# ── Phase C（--full）：没人重连，宽限期到期后 run 最终会被停止 ─────────────
def phase_c() -> None:
    print("\n[Phase C / --full] 宽限期内没有任何客户端重连，到期后 run 应被停止")
    sock, log = "/tmp/ecjc-verify-slow-C.sock", "/tmp/ecjc-verify-slow-C.log"
    proc, log_f = spawn_backend(sock, log)
    g = None
    try:
        g = raw_connect(sock)

        def g_send(o):
            g.sendall((json.dumps(o) + "\n").encode())

        def g_drain(seconds):
            end = time.time() + seconds
            while time.time() < end:
                try:
                    g.recv(1 << 20)
                except socket.timeout:
                    pass

        g_drain(0.3)
        g_send({"cmd": "connect_bus"}); g_drain(2.0)
        g_send({"cmd": "set_mode", "mode": "CSV"}); g_drain(0.3)
        g_send({"cmd": "servo_enable"}); g_drain(1.0)
        g_send({"cmd": "set_trajectory", "type": "constant", "value": 50.0})
        g_send({"cmd": "set_target", "value": 50.0})
        g_send({"cmd": "start_run"}); g_drain(1.0)

        est_wait = K_SLOW_CLIENT_EVICT_AFTER_DROPS * 0.2
        print(f"  停止读取，轮询等待被踢出（预计约 {est_wait:.0f}s）...")
        t0 = time.time()
        evict_seen = False
        while time.time() - t0 < est_wait + 40:
            time.sleep(2.0)
            if "判定为长期失联，主动断开" in read_log(log):
                evict_seen = True
                break
        check(evict_seen, f"后端日志出现踢出 WARNING，耗时 {time.time() - t0:.1f}s")
        g.close()
        g = None

        print(f"  不重连，等待整个重连宽限期到期（约 {K_RECONNECT_GRACE_SEC}s）...")
        t1 = time.time()
        grace_expired = False
        while time.time() - t1 < K_RECONNECT_GRACE_SEC + 20:
            time.sleep(2.0)
            txt = read_log(log)
            if "重连宽限期" in txt and "已过" in txt:
                grace_expired = True
                break
        check(grace_expired, f"宽限期到期日志出现，耗时 {time.time() - t1:.1f}s")
        check(proc.poll() is None, "后端进程仍在正常运行（--full 场景未崩溃）")

        checker = Observer(sock)
        checker.send({"cmd": "get_status"})
        checker.pump(1.0)
        check(checker.status.get("running") is False, "宽限期到期后 run 确实被停止了")
        checker.close()
    finally:
        if g is not None:
            try:
                g.close()
            except OSError:
                pass
        stop_backend(proc, log_f, sock)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--full", action="store_true",
                     help="额外跑 Phase C：真的等满整个重连宽限期，验证超时后确实会 stopRun"
                          "（比默认多花约 90~120 秒）")
    args = ap.parse_args()

    if not EXE.is_file():
        print(f"找不到 {EXE}，请先 cmake --build build -j")
        return 1

    phase_a()
    phase_b()
    if args.full:
        phase_c()

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
