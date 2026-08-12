#!/usr/bin/env python3
"""PDO 重映射的 J1~J6 判定。只读，不改任何配置。

前提：后端已经在跑（`./build/ecjc-backend --config config &`），伺服不使能。
本脚本自己不下发任何 SDO 写、不改 pdo.yaml、不重启主站——只用
`ethercat upload` / `ethercat domains` / `ethercat slaves` / `dmesg` 读状态。

── J1（dmesg）用法 ──────────────────────────────────────────────────────
本机 `kernel.dmesg_restrict=1`：不带权限读 dmesg 只会读到空输出、判定恒 PASS
（这正是终审 C1 抓到的假阳性）。因此 J1 固定走 `pkexec /bin/dmesg -T`，并检查
`returncode`——读不到内核日志本身就没有判据，判 **FAIL** 并给出原因，而不是
把"没读到告警"悄悄等同于"没有告警"。

同一次开机内，更早的一次历史失败（例如上一轮验证时的映射失败）会一直留在
dmesg 环形缓冲区里，若不做基线区分会让 J1 永久 FAIL。为此加了基线机制：

    # 改 pdo.yaml 之前先记一次基线（同时记 dmesg 标记行与当前 domain 字节数）：
    python3 tools/verify_pdo_remap.py --record-baseline

    # 之后每一步用 --expect-delta 判定"比基线多了多少字节"，J1 只看基线标记
    # 行之后新增的 dmesg 内容：
    python3 tools/verify_pdo_remap.py --expect-delta 0    # 基线（0x1A00 仍注释）
    python3 tools/verify_pdo_remap.py --expect-delta 4    # 只加了 0x2240
    python3 tools/verify_pdo_remap.py --expect-delta 8    # 0x2240 + 0x2241 全加完

基线记在 /tmp/ecjc_verify_baseline.json（进程重启、机器重启后失效，需要重新
`--record-baseline`）。也可以用 `--dmesg-since <dmesg 里的某一整行文本>` 手动
指定标记行，跳过基线文件。都不给的话 J1 退化为全量扫描（沿用旧行为，等价于
"从不曾记过基线"）。

── J2（domain 字节数）换算 ──────────────────────────────────────────────
`ethercat domains` 报的是 **Rx + Tx 合计**，不是只报 TxPDO（这是终审 C2 抓到的
第二个错误）。本机当前基线（0x1A00 仍注释）实测应为 **60 字节**：
    RxPDO 0x1605 = 16 字节（4+4+2+2+2+1+1）
  + TxPDO 0x1A06+1A07+1A0D+1A1F+1A08+1A18+1A19 = 44 字节
  = 60 字节
加 0x2240（4 字节）→ 64；再加 0x2241（4 字节）→ 68。

`--expect-delta N` 就是"当前 Size − 基线 Size == N"，不需要背这套换算。
仍保留 `--expect-bytes N` 作绝对值判据（不依赖基线文件），但用它之前必须自己
按上面的算法核算 Rx+Tx 总量——不要直接抄旧文档里的 Tx-only 数字（44/48/52），
那是本次修复之前的错误换算。

退出码：全部判据 PASS 为 0；任何一项 FAIL 为 1；`--record-baseline` 模式下
成功记录返回 0、读 dmesg/domain 失败返回 1。

刻意不用 `ethercat pdos`：它取的是 idle 相位的 SII 扫描快照，从站进入 OP 之后
再查也可能仍然显示出厂映射——不能证明"当前生效的映射"是什么。改用
dmesg（映射写失败的唯一权威来源）+ domain 实际字节数（组帧协商的结果）。
"""
import argparse
import json
import re
import subprocess
import sys

ECAT = "/usr/local/bin/ethercat"
BASELINE_FILE = "/tmp/ecjc_verify_baseline.json"


def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True).stdout


def dmesg_priv():
    """带权限读内核日志。本机 kernel.dmesg_restrict=1，不带 pkexec 只读到空输出。"""
    return subprocess.run(["pkexec", "/bin/dmesg", "-T"], capture_output=True, text=True)


def j1_dmesg(dmesg_since):
    """J1: 映射写失败在 IgH 里只是 WARN，不阻断状态机——唯一权威判据是 dmesg。

    returncode != 0（权限被拒、pkexec 认证取消等）时直接 FAIL：读不到内核日志
    没有判据，"没有证据"不能当"证据表明没有"用。
    """
    r = dmesg_priv()
    if r.returncode != 0:
        return False, [
            f"dmesg 读取失败（returncode={r.returncode}）: {r.stderr.strip()}"
            "——无法判断有无映射失败，视为 FAIL（读不到内核日志 = 无判据 ≠ 通过）"
        ]

    lines = [l for l in r.stdout.splitlines()]
    if dmesg_since:
        try:
            idx = lines.index(dmesg_since)
            lines = lines[idx + 1:]
        except ValueError:
            return False, [
                "基线标记行在当前 dmesg 中未找到（环形缓冲区可能已滚出，或系统/"
                "内核日志被重置过）——请重新执行 --record-baseline 后再判定"
            ]

    hits = [l for l in lines if "Failed to configure mapping" in l]
    return (len(hits) == 0), hits


def get_domain_size():
    """读 `ethercat domains` 的实际字节数（Rx + Tx 合计，见文件头换算说明）。"""
    r = subprocess.run(["pkexec", ECAT, "domains"], capture_output=True, text=True)
    if r.returncode != 0:
        return None, f"`ethercat domains` 失败（returncode={r.returncode}）: {r.stderr.strip()}"
    m = re.search(r"Size\s+(\d+)\s*,", r.stdout)
    if not m:
        return None, f"无法解析 domain 字节数:\n{r.stdout}"
    return int(m.group(1)), None


def load_baseline():
    try:
        with open(BASELINE_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def record_baseline():
    r = dmesg_priv()
    if r.returncode != 0:
        print(f"记录基线失败：dmesg 读取失败（returncode={r.returncode}）: {r.stderr.strip()}")
        sys.exit(1)
    lines = [l for l in r.stdout.splitlines() if l.strip()]
    last_line = lines[-1] if lines else ""

    size, size_err = get_domain_size()
    if size is None:
        print(f"记录基线失败：{size_err}")
        sys.exit(1)

    baseline = {"domain_size": size, "dmesg_last_line": last_line}
    with open(BASELINE_FILE, "w") as f:
        json.dump(baseline, f)
    print(f"基线已记录 -> {BASELINE_FILE}")
    print(f"  domain_size      = {size} 字节")
    print(f"  dmesg 标记行     = {last_line!r}")


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
    ap.add_argument("--record-baseline", action="store_true",
                     help="记录当前 dmesg 标记行 + domain 字节数到 " + BASELINE_FILE +
                          "，不做 J1~J6 判定")
    ap.add_argument("--expect-bytes", type=int, default=None,
                     help="J2 绝对值判据：domain Size 必须等于此值（须自行核算 Rx+Tx，"
                          "见文件头注释；本机当前基线实测应为 60）")
    ap.add_argument("--expect-delta", type=int, default=None,
                     help="J2 增量判据：domain Size − 基线 Size 必须等于此值"
                          "（需要先跑过 --record-baseline）")
    ap.add_argument("--dmesg-since", type=str, default=None,
                     help="手动指定 dmesg 基线标记行（整行文本），覆盖基线文件里记的那行；"
                          "J1 只统计这一行之后的新增内容")
    a = ap.parse_args()

    if a.record_baseline:
        record_baseline()
        return

    if a.expect_bytes is None and a.expect_delta is None:
        ap.error("需要 --expect-bytes 或 --expect-delta 之一（推荐先跑 --record-baseline "
                  "再用 --expect-delta）")
    if a.expect_bytes is not None and a.expect_delta is not None:
        ap.error("--expect-bytes 与 --expect-delta 二选一，不要同时给")

    baseline = load_baseline()
    dmesg_since = a.dmesg_since or (baseline["dmesg_last_line"] if baseline else None)

    results = []

    ok1, hits = j1_dmesg(dmesg_since)
    print(f"J1 dmesg 无映射失败      : {'PASS' if ok1 else 'FAIL'}"
          + ("" if dmesg_since else "（未提供基线，全量扫描——同一次开机内的历史失败"
             "会让本次判定不可靠，建议先跑 --record-baseline）"))
    for h in hits:
        print("   ", h)
    results.append(ok1)

    got, size_err = get_domain_size()
    if got is None:
        print(f"J2 domain 字节数         : FAIL  {size_err}")
        results.append(False)
    elif a.expect_delta is not None:
        if not baseline:
            print("J2 domain 字节数         : FAIL  未找到基线文件，"
                  f"先跑 `--record-baseline`（{BASELINE_FILE} 不存在或已损坏）")
            results.append(False)
        else:
            expect = baseline["domain_size"] + a.expect_delta
            ok2 = got == expect
            print(f"J2 domain 字节数         : {'PASS' if ok2 else 'FAIL'}  "
                  f"domain = {got} 字节，基线 {baseline['domain_size']} + "
                  f"delta {a.expect_delta} = 期望 {expect}")
            results.append(ok2)
    else:
        ok2 = got == a.expect_bytes
        print(f"J2 domain 字节数         : {'PASS' if ok2 else 'FAIL'}  "
              f"domain = {got} 字节，期望 {a.expect_bytes}（绝对值判据，"
              "须自行核算 Rx+Tx 总量，见文件头注释）")
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
