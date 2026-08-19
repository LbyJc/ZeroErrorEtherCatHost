#!/usr/bin/env python3
"""重复定位精度 RP 离线分析（analyze_node_tests.analyze_rp）的合成数据测试。

用真实生成的 RP 工况文件（experiments/工况/重复定位精度_空载0Tr.csv）插值出
target_position_deg，再叠加已知的方向偏置与噪声合成 h5：
  - ↑ 逼近（正向）到位偏置 +0.0010°，↓ 逼近 -0.0010° → 双向差 B 应≈7.2 角秒
  - 高斯噪声 σ=0.0002° → R（±3σ of 段尾2s均值）应远小于 1 角秒
  - 每方向开头 2 次预跑的到位段注入 +0.5° 野值：若预跑没被剔除，R 会爆到千角秒
不碰真实主站、不依赖 Qt。
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import h5py
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "experiments"))

failures = []


def check(cond, msg):
    print(f"  {'[ OK ]' if cond else '[FAIL]'} {msg}")
    if not cond:
        failures.append(msg)


def load_profile():
    p = ROOT / "experiments" / "工况" / "重复定位精度_空载0Tr.csv"
    rows = [ln.split(",") for ln in p.read_text(encoding="utf-8").splitlines()
            if ln and not ln.startswith("#") and not ln.startswith("time")]
    t = np.array([float(r[0]) for r in rows])
    pos = np.array([float(r[1]) for r in rows])
    return t, pos


def synth_h5(out_path: Path):
    """合成 RP 遥测。返回注入的真值 (B_deg, sigma_deg)。"""
    bp_t, bp_pos = load_profile()
    fs = 100.0
    t = np.arange(0.0, bp_t[-1], 1.0 / fs)
    base = 123.0                                # 模拟起点平移后的绝对角
    target = np.interp(t, bp_t, bp_pos) + base

    rng = np.random.default_rng(42)
    sigma = 0.0002
    bias_up, bias_dn = +0.0010, -0.0010
    out = target + rng.normal(0.0, sigma, t.size)
    half = bp_t[-1] / 2.0                       # 前半 = ↑ 块，后半 = ↓ 块
    out[t < half] += bias_up
    out[t >= half] += bias_dn

    # 预跑野值：每方向开头 2 次"到位停留"（target==base 的 ≥3s 平台）加 0.5°
    on_point = np.isclose(target, base)
    edges = np.flatnonzero(np.diff(on_point.astype(int)))
    starts = edges[::2] + 1 if not on_point[0] else np.concatenate(([0], edges[1::2] + 1))
    ends = edges[1::2] if not on_point[0] else edges[::2]
    dwells = [(a, b) for a, b in zip(starts, ends) if t[b] - t[a] >= 3.0]
    n_half = sum(1 for a, b in dwells if t[a] < half)
    for i, (a, b) in enumerate(dwells):
        rank = i if t[a] < half else i - n_half
        if rank < 2:
            out[a:b + 1] += 0.5

    motor = out * 121.0                          # 电机侧与输出侧一致（÷121 后）

    with h5py.File(out_path, "w") as f:
        g = f.create_group("experiment")
        g.attrs["test_item"] = "RP"
        g.create_dataset("system_time_ns", data=(t * 1e9 + 1.7e18).astype(np.int64))
        g.create_dataset("target_position_deg", data=target)
        g.create_dataset("output_position_unwrapped_deg", data=out)
        g.create_dataset("motor_position_unwrapped_deg", data=motor)
    return (bias_up - bias_dn) * 3600.0, sigma


def main():
    import analyze_node_tests as an

    print("\n[1] analyze_rp 指标正确性（合成数据，真值已知）")
    tmp = Path(tempfile.mkdtemp(prefix="ecjc_rp_test_"))
    h5 = tmp / "sample_A01__test_RP.h5"
    b_true_arcsec, _ = synth_h5(h5)

    res = an.analyze_rp(h5)
    check(res is not None, "analyze_rp 返回结果")
    check(res["n_up"] == 30 and res["n_down"] == 30,
          f"每方向计入 30 次循环（↑{res['n_up']} ↓{res['n_down']}），预跑已剔除")
    o = res["output"]
    check(abs(o["B_arcsec"] - b_true_arcsec) < 0.5,
          f"双向差 B ≈ 注入真值 {b_true_arcsec:.1f} 角秒（实际 {o['B_arcsec']:.2f}）")
    check(o["R_up_arcsec"] < 1.0 and o["R_down_arcsec"] < 1.0,
          f"R↑/R↓ < 1 角秒（{o['R_up_arcsec']:.3f}/{o['R_down_arcsec']:.3f}）"
          "——预跑 0.5° 野值若被计入会爆到千角秒")
    check(o["noise_arcsec"] < 2.0,
          f"停留窗内噪声地板 {o['noise_arcsec']:.3f} 角秒（σ=0.0002° → ~0.7）")
    m = res["motor"]
    check(abs(m["B_arcsec"] - o["B_arcsec"]) < 0.1,
          "电机侧 ÷121 与输出侧互证一致（合成数据两侧同源）")

    print("\n[2] 指标文件落盘")
    metrics = h5.with_name(h5.stem + "_rp_metrics.csv")
    summary = h5.with_name(h5.stem + "_rp_summary.csv")
    check(metrics.is_file(), f"逐次停留指标 {metrics.name}")
    check(summary.is_file(), f"汇总指标 {summary.name}")
    lines = metrics.read_text(encoding="utf-8").splitlines()
    check(len(lines) == 1 + 64, f"逐次表 = 表头 + 64 次停留（含预跑，标 counted，"
                                f"实际 {len(lines) - 1}）")

    print("\n[3] analyze() 按 test_item=RP 自动分流")
    r = an.analyze(h5, {}, {})
    check(isinstance(r, dict) and "n_up" in r,
          "RP 文件走 RP 分析，不再按恒速平台切段")

    print("\n" + "=" * 50)
    if failures:
        print(f"{len(failures)} 项失败"); return 1
    print("全部通过"); return 0


if __name__ == "__main__":
    sys.exit(main())
