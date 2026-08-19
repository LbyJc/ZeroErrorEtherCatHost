#!/usr/bin/env python3
"""生成节点周期测试的轨迹工况文件 + 一键实验配置（实验计划 V4.0 §4.3 / §4.4）。

2026-08-13 用户现场决定：
  - 摩擦测试恒速段 90s → 20s；重复次数 3 → 1；
  - 每个"转速 × 方向"单独一个工况文件和一个配置（正反转分开、不同转速分开），
    测试员一次只跑一个点。数据文件名与配置名同步。
  - 回差因此要把同载荷档的正转、反转两个数据文件一起交给分析脚本配对。

2026-08-14 用户决定：合并只差恒速时长的重复配置。
  周期测试1(电流摩擦, 20s) 与周期测试2(TE, 90s) 在"5rpm × 空载/轻载 × 正反转"
  四个点上轨迹形状完全相同、只差恒速时长——合并成一次 90s 运行同时供两种分析
  （analyze_node_tests.py 的电流采集窗自适应 min(60, 0.75×平台时长)，90s 足够）。
  合并后保留周期测试2 的配置（yaml 加 merged_from 字段，列表名显示"（含 …）"），
  删除对应的周期测试1 配置；摩擦 10rpm（TE 不测）和 TE 中载（摩擦不测）不受影响。
  运行次数 14 → 10，总时长反而更短。

2026-08-18 第四轮：新增重复定位精度 RP 工况（41~43，CSP 位置模式）。
参数经用户认可：就地起测（后端对 CSP 的 CSV 轨迹做起点平移，文件里是相对
波形）、回程 ±10°、逼近 15°/s（与 CSP 恒值斜坡速率一致，每次逼近条件相同）、
到位停 4s（分析取段尾 2s 均值）、每方向 2 次预跑 + 30 循环（GB/T 12642
n=30 → ±3σ），正反两方向放同一文件（双向差 B 必须同会话）。

速度类轨迹文件是四列格式 time,target_position,target_velocity,target_torque：
后端 loadCsv() 对两列文件写死按 CSP 位置解析（trajectory.cpp），CSV 速度
模式必须用四列格式，速度走第三列（单位 = 输出端 rpm，正转 = 正号）。
RP 工况用两列 time,target（CSP 位置，单位 = 输出侧相对角度 deg）。

本脚本同时生成 presets/*.yaml（除 10_持续运行_寿命摆臂.yaml 手写保留），
工况参数只在这里改，跑一遍脚本两边就同步。
"""
from pathlib import Path

RAMP_RPM_PER_S = 1.25    # < controller.yaml velocity_rate_rpm_per_s(1.65)，指令即实际
HOLD_T1_S = 20.0         # 摩擦测试恒速段（现场由计划 90s 缩短）
HOLD_T2_S = 90.0         # TE 测试保持计划值（要覆盖输出端≥1圈：90s@5rpm=7.5圈）
TAIL_S = 2.0             # 结尾零速小段，跑完自动软停

HERE = Path(__file__).resolve().parent
OUT = HERE / "工况"
PRESETS = HERE / "presets"
OUT.mkdir(exist_ok=True)
PRESETS.mkdir(exist_ok=True)


def write_profile(name, rpm, hold_s, comment):
    """单点工况：斜坡 → 恒速 hold_s → 斜坡 → 零速 2s。"""
    ramp = abs(rpm) / RAMP_RPM_PER_S
    rows = [(0.0, 0.0), (ramp, rpm), (ramp + hold_s, rpm),
            (2 * ramp + hold_s, 0.0), (2 * ramp + hold_s + TAIL_S, 0.0)]
    p = OUT / name
    with open(p, "w", encoding="utf-8") as f:
        f.write(f"# {comment}\n")
        f.write("# 速度单位 = 输出端 rpm，正转 = 正号（负角向正角方向）。\n")
        f.write(f"# 总时长 {rows[-1][0]:.0f} s，跑完自动软停。\n")
        f.write("time,target_position,target_velocity,target_torque\n")
        for t, v in rows:
            f.write(f"{t:.2f},0,{v:g},0\n")
    return rows[-1][0]


def write_preset(num, name, csv_name, test_item, load_pct, speed, desc,
                 merged_from=None, mode="CSV"):
    p = PRESETS / f"{num}_{name}.yaml"
    with open(p, "w", encoding="utf-8") as f:
        f.write(f"# 由 generate_profiles.py 生成，改参数请改脚本后重新生成\n")
        f.write(f"name: {name}\n")
        f.write("kind: node\n")
        f.write(f"mode: {mode}\n")
        if merged_from:
            f.write("merged_from:\n")
            for m in merged_from:
                f.write(f"  - {m}\n")
        f.write("description: |\n")
        for line in desc.strip().splitlines():
            f.write(f"  {line}\n")
        f.write("trajectory:\n")
        f.write("  type: csv\n")
        f.write(f"  csv_path: ../工况/{csv_name}\n")
        f.write("record:\n")
        f.write(f"  test_item: {test_item}\n")
        f.write(f"  load_percent_Tr: {load_pct}\n")
        f.write(f"  speed_rpm_target: {speed}\n")


# ── 重复定位精度 RP（2026-08-18 第四轮，用户认可的参数）──
RP_RETREAT_DEG = 10.0      # 回程距离（测试点 ± 两个方向各用一侧）
RP_APPROACH_DEG_S = 15.0   # 逼近速度 = controller csp_position_rate_deg_per_s
RP_DWELL_S = 4.0           # 到位停留（分析取段尾 2s 均值）
RP_RETREAT_DWELL_S = 1.0   # 回程点短停，让每次逼近初始条件一致
RP_CYCLES = 30             # 每方向计入循环数（GB/T 12642 n=30 → ±3σ）
RP_WARMUP = 2              # 每方向开头预跑次数，分析不计入


def write_rp_profile(name, comment):
    """RP 工况：两列 CSP 位置相对波形，首点 0（后端起点平移到当前实测位置）。
    先正向逼近块（回程到 -10° 再回 0，↑），后反向逼近块（+10° 回 0，↓）。"""
    rows = [(0.0, 0.0)]
    t = 0.0
    move = RP_RETREAT_DEG / RP_APPROACH_DEG_S
    for retreat in (-RP_RETREAT_DEG, +RP_RETREAT_DEG):
        for _ in range(RP_WARMUP + RP_CYCLES):
            t += move;              rows.append((t, retreat))
            t += RP_RETREAT_DWELL_S; rows.append((t, retreat))
            t += move;              rows.append((t, 0.0))
            t += RP_DWELL_S;        rows.append((t, 0.0))
    p = OUT / name
    with open(p, "w", encoding="utf-8") as f:
        f.write(f"# {comment}\n")
        f.write("# 两列 CSP 位置工况，单位 = 输出侧相对角度 deg（后端把首点平移到当前实测位置，就地起测）。\n")
        f.write(f"# 回程 ±{RP_RETREAT_DEG:g}°，逼近 {RP_APPROACH_DEG_S:g}°/s，"
                f"到位停 {RP_DWELL_S:g}s，每方向 {RP_WARMUP} 预跑 + {RP_CYCLES} 循环。\n")
        f.write(f"# 总时长 {t:.0f} s，跑完自动软停。\n")
        f.write("time,target\n")
        for tt, pos in rows:
            f.write(f"{tt:.4f},{pos:g}\n")
    return t


AUTO_NOTE = "跑完自动停止采集、数据落盘并弹出完成提示，无需手动点结束。"
LOADS_T1 = [("空载0Tr", 0, "加载装置已【机械脱开】（不是归零）"),
            ("轻载10Tr", 10, "外部加载装置设 3.1 Nm（10%Tr）并记录实际值")]
LOADS_T2 = LOADS_T1 + [("中载30Tr", 30, "外部加载装置设 9.3 Nm（30%Tr）并记录实际值")]
DIRS = [("正转", +1), ("反转", -1)]

num = 21
# §4.3 周期测试1：只剩 10 rpm（5 rpm 已并入周期测试2，见文件头 2026-08-14）。
# num 照旧走满 载荷×转速×方向 的格子，保持 23/24/27/28 编号与合并前一致。
for tag, pct, load_note in LOADS_T1:
    for speed in (5, 10):
        for dname, sign in DIRS:
            if speed == 5:
                num += 1        # 占位：该点由周期测试2 的合并配置覆盖
                continue
            name = f"周期测试1_电流摩擦_{tag}_{speed}rpm{dname}"
            csv_name = name + ".csv"
            dur = write_profile(csv_name, sign * speed, HOLD_T1_S,
                                f"§4.3 电流摩擦 | {tag} | {speed}rpm {dname} | 恒速 {HOLD_T1_S:.0f}s")
            write_preset(num, name, csv_name, "current", pct, speed,
                         f"电流与摩擦基线（§4.3），{tag}：{speed} rpm {dname}，"
                         f"恒速 {HOLD_T1_S:.0f} s，全程约 {dur:.0f} 秒。\n"
                         f"5 rpm 的电流摩擦数据来自同载荷档的周期测试2 合并工况，无需单独跑。\n"
                         f"开始前确认：① {load_note}；② 已 Servo Enable。\n{AUTO_NOTE}")
            print(f"{num}  {name}  ({dur:.0f}s)")
            num += 1

num = 31
# §4.4 周期测试2：每个 载荷 × 方向 一个文件/配置（5 rpm）。
# 空载/轻载档同时覆盖周期测试1 的 5rpm 电流摩擦点（merged_from）。
T1_TAGS = {tag for tag, _, _ in LOADS_T1}
for tag, pct, load_note in LOADS_T2:
    for dname, sign in DIRS:
        name = f"周期测试2_传动误差_{tag}_5rpm{dname}"
        csv_name = name + ".csv"
        dur = write_profile(csv_name, sign * 5, HOLD_T2_S,
                            f"§4.4 传动误差TE | {tag} | 5rpm {dname} | 恒速 {HOLD_T2_S:.0f}s"
                            + (" | 兼供§4.3电流摩擦分析" if tag in T1_TAGS else ""))
        merged = ([f"周期测试1_电流摩擦_{tag}_5rpm{dname}"] if tag in T1_TAGS else None)
        merged_note = ("本工况同时供电流摩擦（§4.3）分析使用——同一份数据两用，"
                       "不必再单跑 5 rpm 摩擦点。\n" if merged else "")
        write_preset(num, name, csv_name, "TE", pct, 5,
                     f"传动误差 TE 测试（§4.4），{tag}：5 rpm {dname}，"
                     f"恒速 {HOLD_T2_S:.0f} s（输出端 7.5 圈），全程约 {dur:.0f} 秒。\n"
                     f"{merged_note}"
                     f"开始前确认：① {load_note}；② 已 Servo Enable。\n{AUTO_NOTE}\n"
                     f"回差：把同载荷档【正转+反转】两个数据文件一起交给分析脚本。",
                     merged_from=merged)
        print(f"{num}  {name}  ({dur:.0f}s)" + ("  [含电流摩擦]" if merged else ""))
        num += 1

num = 41
# 重复定位精度 RP：每载荷档一个配置（正反两方向在同一文件里，双向差要同会话）。
for tag, pct, load_note in LOADS_T2:
    name = f"重复定位精度_{tag}"
    csv_name = name + ".csv"
    dur = write_rp_profile(csv_name,
                           f"重复定位精度 RP | {tag} | 回程 ±{RP_RETREAT_DEG:g}° | "
                           f"每方向 {RP_WARMUP} 预跑 + {RP_CYCLES} 循环")
    write_preset(num, name, csv_name, "RP", pct,
                 RP_APPROACH_DEG_S / 6.0,       # 逼近速度换算 rpm（15°/s = 2.5）
                 f"重复定位精度 RP 测试，{tag}：就地起测——开始前先把关节停到本节点固定"
                 f"参考位（建议摆臂竖直下垂），每次测试都用同一参考位。\n"
                 f"回程 ±{RP_RETREAT_DEG:g}°，逼近 {RP_APPROACH_DEG_S:g}°/s，"
                 f"到位停 {RP_DWELL_S:g} s；正反两方向各 {RP_WARMUP} 次预跑（不计入）+ "
                 f"{RP_CYCLES} 次循环，同一文件采完，全程约 {dur/60:.1f} 分钟。\n"
                 f"开始前确认：① {load_note}；② 已 Servo Enable；③ 关节已在参考位。\n"
                 f"{AUTO_NOTE}\n"
                 f"分析：单个 h5 交给 analyze_node_tests.py，输出 R↑/R↓/双向差 B（角秒）。",
                 mode="CSP")
    print(f"{num}  {name}  ({dur:.0f}s)  [CSP]")
    num += 1
