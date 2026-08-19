#!/usr/bin/env python3
"""节点周期测试离线分析：§4.3 电流/摩擦指标、§4.4 传动误差 TE、§4.5 回差、
重复定位精度 RP。

用法：
    python analyze_node_tests.py <采集的.h5文件> [更多.h5 ...]

对每个文件输出 <文件名>_node_metrics.csv（每个恒速采集段一行，§4.3/§4.4 指标）。
h5 里 test_item 属性为 RP 的文件自动分流到 RP 分析（单个文件传即可），
输出 *_rp_metrics.csv（逐次停留）与 *_rp_summary.csv（R↑/R↓/双向差 B，角秒）。

回差（2026-08-13 起工况按"转速×方向"拆分，一个文件只有一个方向）：
把**同载荷档的正转和反转两个 h5 一起**传进来，脚本把所有文件的正/反转段
汇总配对，输出 return_error_<速度>rpm.csv 到第一个文件所在目录（角秒）。
注意两个文件必须同一次上电会话采集（多圈计数断电丢失会破坏角度基准）。

方法与约定：
- 切段：按指令速度 target_velocity_rpm 的恒速平台（工况文件保证每段 90 s
  恒速、段间停 10 s），每段取**最后 60 s** 为采集窗（前 30 s 是稳定期）。
- TE(t) = theta_out(t) − theta_in(t)/i，i=121（计划正文写 120，修订建议 §1.1
  已按手册"输出转一圈电机转 X+1 圈"与实测确认为 121，物理上精确）。
  两侧编码器零位互不对齐，TE 带一个恒定偏置：TE_pp 不受影响，
  TE_rms 按"去均值后的 RMS"（即 std）报告，回差差分时偏置精确对消。
- 回差（§4.5，修订建议 §2.2 方法）：把正转段与反转段的 TE 分别线性插值到
  公共输出角栅格（步长 0.1°，禁止最近邻），return_error(θ) = TE_f(θ) − TE_r(θ)，
  正转 = 从负角向正角（速度正号）。**单位角秒**（用户 2026-08-13 指定）。
- 有效性核查（计划无效数据判据第 2 条）：采集窗内输出速度均值偏离目标
  超过 ±5% 的段在表里标 invalid。
"""
from __future__ import annotations

import sys
from pathlib import Path

import h5py
import numpy as np

GEAR = 121.0          # 减速比，物理精确值（铭牌 120 是标称）

# ── 重复定位精度 RP（2026-08-18 第四轮设计）────────────────────────────
# 工况：就地起测，回程 ±10°，逼近 15°/s，到位停 4s，每方向 2 预跑 + 30 循环，
# 正反两方向同一文件。h5 attr test_item == "RP" 时 analyze() 自动分流到这里。
RP_DWELL_MIN_S = 3.0   # 测试点停留 4s、回程点只停 1s——用 3s 门槛区分
RP_MEAS_TAIL_S = 2.0   # 每次停留取段尾 2s 均值（前 2s 当整定期扔掉）
RP_WARMUP = 2          # 每方向开头预跑次数，不计入统计
RP_MIN_MOVE_DEG = 1.0  # 停留段与前一平台差 ≥1° 才算一次真实逼近——
                       # 滤掉开跑前/跑完后 CSP 空闲保持形成的假平台
COLLECT_MAX_S = 60.0  # 采集窗上限（段尾）；实际 = min(60, 0.75×平台时长)
                      # → 90s 平台取尾 60s（计划口径），20s 平台（2026-08-13
                      #   摩擦测试缩短后）取尾 15s，头部自动当稳定期扔掉
MIN_HOLD_S = 15.0     # 平台最短时长（20s 恒速段要能进来；斜坡碎段被它滤掉）
GRID_DEG = 0.1        # 回差插值栅格（修订建议 0.05~0.1°）
ARCMIN = 60.0
ARCSEC = 3600.0


def segments(t, v_cmd):
    """按指令速度切恒速平台，返回 [(i0, i1, v_target)]（含端点索引）。"""
    out = []
    v = np.round(v_cmd, 3)
    edges = np.flatnonzero(np.diff(v) != 0)
    starts = np.concatenate(([0], edges + 1))
    ends = np.concatenate((edges, [len(v) - 1]))
    for a, b in zip(starts, ends):
        if v[a] == 0:
            continue
        if t[b] - t[a] >= MIN_HOLD_S:
            out.append((a, b, float(v[a])))
    return out


def _plateaus(t, x):
    """相邻相等值的平台段，返回 [(i0, i1, value)]（含端点索引）。"""
    v = np.round(x, 4)
    edges = np.flatnonzero(np.diff(v) != 0)
    starts = np.concatenate(([0], edges + 1))
    ends = np.concatenate((edges, [len(v) - 1]))
    return [(int(a), int(b), float(v[a])) for a, b in zip(starts, ends)]


def analyze_rp(path: Path):
    """重复定位精度：按 target_position_deg 平台切出测试点停留段，
    输出 R↑/R↓（±3σ）、双向差 B、停留窗噪声地板（角秒）。
    输出端为主，电机端 ÷121 双编码器互证——若驱动位置环闭在输出端，
    输出端散布只是伺服精度，机械重复性看电机端散布。"""
    with h5py.File(path, "r") as f:
        g = f["experiment"]
        t = np.asarray(g["system_time_ns"], dtype=np.float64) / 1e9
        t -= t[0]
        tgt = np.asarray(g["target_position_deg"])
        th_out = np.asarray(g["output_position_unwrapped_deg"])
        th_in = np.asarray(g["motor_position_unwrapped_deg"])

    # 移动斜坡上每个采样都是单点"平台"，先滤成显著平台（≥0.5s：
    # 回程点 1s 停留和测试点 4s 停留都在，斜坡碎点都不在），
    # 再在显著平台序列里找每次到位停留的前驱（回程平台）判方向。
    plats = [p for p in _plateaus(t, tgt) if t[p[1]] - t[p[0]] >= 0.5]
    rows = []
    per_dir = {"up": [], "down": []}
    for k, (a, b, val) in enumerate(plats):
        if k == 0 or t[b] - t[a] < RP_DWELL_MIN_S:
            continue
        prev_val = plats[k - 1][2]
        if abs(val - prev_val) < RP_MIN_MOVE_DEG:
            continue                      # 空闲保持假平台，不是逼近到位
        direction = "up" if prev_val < val else "down"
        w = (t >= t[b] - RP_MEAS_TAIL_S) & (np.arange(len(t)) >= a) \
            & (np.arange(len(t)) <= b)
        if int(w.sum()) < 10:
            continue
        per_dir[direction].append(len(rows))
        rows.append({
            "direction": "↑" if direction == "up" else "↓",
            "rep": len(per_dir[direction]),          # 含预跑的序号，counted 另标
            "counted": len(per_dir[direction]) > RP_WARMUP,
            "t_start_s": float(t[b] - RP_MEAS_TAIL_S), "t_end_s": float(t[b]),
            "out_mean_deg": float(th_out[w].mean()),
            "out_std_arcsec": float(th_out[w].std() * ARCSEC),
            "motor_mean_deg": float((th_in[w] / GEAR).mean()),
            "motor_std_arcsec": float((th_in[w] / GEAR).std() * ARCSEC),
        })
    if not per_dir["up"] or not per_dir["down"]:
        print(f"  !! {path.name}: 没切出正反两方向的到位停留段"
              f"（↑{len(per_dir['up'])} ↓{len(per_dir['down'])}），不是 RP 工况数据？")
        return None

    import csv
    mp = path.with_name(path.stem + "_rp_metrics.csv")
    with open(mp, "w", newline="", encoding="utf-8") as fo:
        wtr = csv.DictWriter(fo, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    print(f"  逐次停留 → {mp.name}（↑{len(per_dir['up'])} ↓{len(per_dir['down'])} 次，"
          f"每方向前 {RP_WARMUP} 次预跑不计入）")

    res = {"n_up": sum(1 for i in per_dir["up"] if rows[i]["counted"]),
           "n_down": sum(1 for i in per_dir["down"] if rows[i]["counted"])}
    sum_rows = []
    for side, mean_k, std_k in (("output", "out_mean_deg", "out_std_arcsec"),
                                ("motor_div121", "motor_mean_deg", "motor_std_arcsec")):
        m = {}
        for d in ("up", "down"):
            means = np.array([rows[i][mean_k] for i in per_dir[d] if rows[i]["counted"]])
            m[f"R_{d}_arcsec"] = float(3.0 * means.std(ddof=1) * ARCSEC) \
                if means.size >= 2 else float("nan")
            m[f"mean_{d}_deg"] = float(means.mean())
        m["B_arcsec"] = abs(m["mean_up_deg"] - m["mean_down_deg"]) * ARCSEC
        m["noise_arcsec"] = float(np.mean([r[std_k] for r in rows if r["counted"]]))
        res[side if side == "output" else "motor"] = m
        sum_rows.append({"side": side, "n_up": res["n_up"], "n_down": res["n_down"],
                         **{k: m[k] for k in ("R_up_arcsec", "R_down_arcsec",
                                              "B_arcsec", "noise_arcsec")}})
    sp = path.with_name(path.stem + "_rp_summary.csv")
    with open(sp, "w", newline="", encoding="utf-8") as fo:
        wtr = csv.DictWriter(fo, fieldnames=list(sum_rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(sum_rows)
    o = res["output"]
    print(f"  RP（输出端）：R↑={o['R_up_arcsec']:.2f}  R↓={o['R_down_arcsec']:.2f}  "
          f"B={o['B_arcsec']:.2f}  噪声地板={o['noise_arcsec']:.2f} 角秒 → {sp.name}")
    mm = res["motor"]
    print(f"  RP（电机端÷121 互证）：R↑={mm['R_up_arcsec']:.2f}  "
          f"R↓={mm['R_down_arcsec']:.2f}  B={mm['B_arcsec']:.2f} 角秒")
    return res


def analyze(path: Path, fwd_pool: dict, rev_pool: dict):
    with h5py.File(path, "r") as f:
        ti = f["experiment"].attrs.get("test_item", "")
    ti = ti.decode() if isinstance(ti, bytes) else str(ti)
    if ti == "RP":
        return analyze_rp(path)          # RP 按位置平台切段，不走恒速分析

    with h5py.File(path, "r") as f:
        g = f["experiment"]
        t = np.asarray(g["system_time_ns"], dtype=np.float64) / 1e9
        t -= t[0]
        v_cmd = np.asarray(g["target_velocity_rpm"])
        v_out = np.asarray(g["output_velocity_rpm"])
        cur = np.asarray(g["motor_current_A"])
        th_out = np.asarray(g["output_position_unwrapped_deg"])
        th_in = np.asarray(g["motor_position_unwrapped_deg"])

    te_deg = th_out - th_in / GEAR          # 带恒定偏置，差分时对消
    segs = segments(t, v_cmd)
    if not segs:
        print(f"  !! {path.name}: 没找到恒速平台（target_velocity_rpm 全为 0？）")
        return

    rows = []
    fwd, rev = fwd_pool, rev_pool            # v_abs -> [(θ数组, TE数组)]，跨文件累积
    rep_count = {}
    for a, b, vt in segs:
        # 采集窗 = 段尾 min(60, 0.75×平台时长) 秒
        collect = min(COLLECT_MAX_S, 0.75 * (t[b] - t[a]))
        w = (t >= t[b] - collect) & (t <= t[b]) & (np.arange(len(t)) >= a)
        n = int(w.sum())
        if n < 100:
            continue
        key = abs(vt)
        rep_count[(vt > 0, key)] = rep_count.get((vt > 0, key), 0) + 1
        rep = rep_count[(vt > 0, key)]
        vm = float(v_out[w].mean())
        valid = abs(vm - vt) <= abs(vt) * 0.05
        te_w = te_deg[w]
        rows.append({
            "segment": f"{'+' if vt > 0 else '-'}{key:g}rpm_rep{rep}",
            "v_target_rpm": vt,
            "v_mean_rpm": vm,
            "v_std_rpm": float(v_out[w].std()),
            "valid_speed_5pct": valid,
            "t_start_s": float(t[b] - collect), "t_end_s": float(t[b]),
            "current_mean_A": float(cur[w].mean()),
            "current_rms_A": float(np.sqrt((cur[w] ** 2).mean())),
            "current_pp_A": float(cur[w].max() - cur[w].min()),
            "TE_pp_arcmin": float((te_w.max() - te_w.min()) * ARCMIN),
            "TE_rms_arcmin": float(te_w.std() * ARCMIN),   # 去均值 RMS，见文件头
        })
        (fwd if vt > 0 else rev).setdefault(key, []).append((th_out[w], te_w))

    # §4.3 正反向电流差（同速各次重复的均值相减）
    dir_diff = {}
    for key in sorted({abs(s[2]) for s in segs}):
        icw = [r["current_mean_A"] for r in rows if r["v_target_rpm"] == key]
        iccw = [r["current_mean_A"] for r in rows if r["v_target_rpm"] == -key]
        if icw and iccw:
            dir_diff[key] = float(np.mean(icw) - np.mean(iccw))

    import csv
    mp = path.with_name(path.stem + "_node_metrics.csv")
    with open(mp, "w", newline="", encoding="utf-8") as fo:
        wtr = csv.DictWriter(fo, fieldnames=list(rows[0].keys()))
        wtr.writeheader()
        wtr.writerows(rows)
    print(f"  段指标 → {mp.name}（{len(rows)} 段）")
    for k, d in dir_diff.items():
        print(f"  本文件内正反向电流差 @{k:g}rpm = {d:+.4f} A")
    return rows


def return_error(fwd: dict, rev: dict, out_dir: Path):
    """§4.5 回差：所有文件累积的正/反转 TE 插到公共输出角栅格后相减，角秒。"""
    import csv
    for key in sorted(set(fwd) & set(rev)):
        thf = np.concatenate([x for x, _ in fwd[key]])
        tef = np.concatenate([y for _, y in fwd[key]])
        thr = np.concatenate([x for x, _ in rev[key]])
        ter = np.concatenate([y for _, y in rev[key]])
        of, orr = np.argsort(thf), np.argsort(thr)
        thf, tef, thr, ter = thf[of], tef[of], thr[orr], ter[orr]
        lo = max(thf[0], thr[0])
        hi = min(thf[-1], thr[-1])
        if hi - lo < 5 * GRID_DEG:
            print(f"!! {key:g} rpm 正反转输出角无重叠区间，算不了回差"
                  f"（正转 {thf[0]:.0f}~{thf[-1]:.0f}°，反转 {thr[0]:.0f}~{thr[-1]:.0f}°）")
            continue
        grid = np.arange(lo, hi, GRID_DEG)
        re_arcsec = (np.interp(grid, thf, tef) - np.interp(grid, thr, ter)) * ARCSEC
        rp = out_dir / f"return_error_{key:g}rpm.csv"
        with open(rp, "w", newline="", encoding="utf-8") as fo:
            wtr = csv.DictWriter(fo, fieldnames=["theta_out_deg", "return_error_arcsec"])
            wtr.writeheader()
            for gth, gre in zip(grid, re_arcsec):
                wtr.writerow({"theta_out_deg": float(gth),
                              "return_error_arcsec": float(gre)})
        print(f"回差 @{key:g}rpm（{lo:.1f}°~{hi:.1f}°，{len(grid)} 点）: "
              f"max={re_arcsec.max():.1f}  min={re_arcsec.min():.1f}  "
              f"mean={re_arcsec.mean():.1f} 角秒 → {rp.name}")
    if not (set(fwd) & set(rev)):
        print("（未同时提供正转与反转数据，跳过回差。回差要把同载荷档的"
              "正转+反转两个 h5 一起传进来。）")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    fwd, rev = {}, {}
    all_rows = []
    for p in sys.argv[1:]:
        print(f"\n== {p}")
        rows = analyze(Path(p), fwd, rev)
        if isinstance(rows, list):       # RP 文件返回 dict，不进恒速段行池
            all_rows.extend(rows)
    print()
    # §4.3 正反向电流差：工况按方向拆了文件，跨全部传入文件汇总
    for key in sorted({abs(r["v_target_rpm"]) for r in all_rows}):
        icw = [r["current_mean_A"] for r in all_rows if r["v_target_rpm"] == key]
        iccw = [r["current_mean_A"] for r in all_rows if r["v_target_rpm"] == -key]
        if icw and iccw:
            print(f"正反向电流差 @{key:g}rpm = "
                  f"{float(np.mean(icw) - np.mean(iccw)):+.4f} A（跨文件）")
    return_error(fwd, rev, Path(sys.argv[1]).resolve().parent)
    return 0


if __name__ == "__main__":
    sys.exit(main())
