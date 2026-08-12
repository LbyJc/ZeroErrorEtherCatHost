#!/usr/bin/env python3
"""PDO 重映射的 J1~J6 判定。只读，不改任何配置。

前提：后端已经在跑（`./build/ecjc-backend --config config &`），伺服不使能。
本脚本自己不下发任何 SDO 写、不改 pdo.yaml、不重启主站——只用
`ethercat upload` / `ethercat domains` / `ethercat slaves` / `dmesg` 读状态。

用法：
    python3 tools/verify_pdo_remap.py --expect-bytes 44   # 基线（0x1A00 仍注释）
    python3 tools/verify_pdo_remap.py --expect-bytes 48   # 只加了 0x2240
    python3 tools/verify_pdo_remap.py --expect-bytes 52   # 0x2240 + 0x2241 全加完

退出码：全部判据 PASS 为 0；任何一项 FAIL 为 1。

刻意不用 `ethercat pdos`：它取的是 idle 相位的 SII 扫描快照，从站进入 OP 之后
再查也可能仍然显示出厂映射——不能证明"当前生效的映射"是什么。改用
dmesg（映射写失败的唯一权威来源）+ domain 实际字节数（组帧协商的结果）。
"""
import argparse
import re
import subprocess
import sys

ECAT = "/usr/local/bin/ethercat"


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True).stdout


def j1_dmesg():
    """J1: 映射写失败在 IgH 里只是 WARN，不阻断状态机——唯一权威判据是 dmesg。"""
    out = run(["dmesg"])
    hits = [l for l in out.splitlines() if "Failed to configure mapping" in l]
    return (len(hits) == 0), hits


def j2_domain_bytes(expect):
    """J2: domain 实际字节数是否等于本次分级放开预期的大小。

    本机 IgH 工具（1.5.4，见 tool/CommandDomains.cpp）`ethercat domains` 的
    输出格式是（非 --verbose）：

        Domain0: LogBaseAddr 0x00000000, Size   6, WorkingCounter 0/1

    不是 "<N> byte"，是 "Size <N>,"（Size 后可能有 setw(3) 的空格填充）。
    """
    out = run(["pkexec", ECAT, "domains"])
    m = re.search(r"Size\s+(\d+)\s*,", out)
    if not m:
        return False, f"无法解析 domain 字节数:\n{out}"
    got = int(m.group(1))
    return got == expect, f"domain = {got} 字节，期望 {expect}"


def j6_sm_timing():
    """J6: SM3 长度变化后，1ms 周期还剩多少余量（参考值，不判 PASS/FAIL）。

    子索引已按 Task 13 的修正核对（不是 :04/:05）：
      0x1C33:05 Minimum Cycle Time   (u32)
      0x1C33:06 Calc and Copy Time   (u32)
      0x1C33:0B SM-Event Missed      (u16)
      0x1C33:0C Cycle Time Too Small (u16)
    对应 config/slave.yaml 里的 sm_in_min_cycle_ns / sm_in_calc_copy_ns /
    sm_in_event_missed / sm_in_cycle_too_small。
    """
    rows = {}
    for sub, name, typ in [
        ("5", "min_cycle_ns", "uint32"),
        ("6", "calc_copy_ns", "uint32"),
        ("0xB", "event_missed", "uint16"),
        ("0xC", "cycle_too_small", "uint16"),
    ]:
        out = run(["pkexec", ECAT, "upload", "-p0", "0x1C33", sub, "--type", typ])
        rows[name] = out.strip()
    return rows


def j_op_state():
    """从站是否全部处于 OP。逐行解析 `ethercat slaves` 的状态列（第 3 列，
    见 tool/CommandSlaves.cpp::listSlaves：pos  alias:relpos  state  flag  name），
    不用子串匹配——"PREOP"/"SAFEOP" 都以 "OP" 结尾，" OP " 子串匹配不可靠。
    """
    out = run(["pkexec", ECAT, "slaves"])
    lines = [l for l in out.splitlines() if l.strip()]
    if not lines:
        return False, "无从站输出", out
    states = []
    for l in lines:
        tokens = l.split()
        if len(tokens) < 3:
            continue
        states.append(tokens[2])
    ok = len(states) > 0 and all(s == "OP" for s in states)
    return ok, states, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--expect-bytes", type=int, required=True)
    a = ap.parse_args()

    results = []

    ok1, hits = j1_dmesg()
    print(f"J1 dmesg 无映射失败      : {'PASS' if ok1 else 'FAIL'}")
    for h in hits:
        print("   ", h)
    results.append(ok1)

    ok2, msg = j2_domain_bytes(a.expect_bytes)
    print(f"J2 domain 字节数         : {'PASS' if ok2 else 'FAIL'}  {msg}")
    results.append(ok2)

    print("J6 SM3 同步时序（参考值，不计入 PASS/FAIL）:", j6_sm_timing())

    ok_op, states, raw = j_op_state()
    print(f"从站状态: {states}")
    print(f"   进入 OP               : {'PASS' if ok_op else 'FAIL'}")
    results.append(ok_op)

    all_pass = all(results)
    print(f"\n总判定: {'PASS' if all_pass else 'FAIL'}")
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
