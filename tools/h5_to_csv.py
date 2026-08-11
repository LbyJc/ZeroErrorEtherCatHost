#!/usr/bin/env python3
"""把实验 HDF5 导出成 CSV。

HDF5 是长期高频采集的主存储（任务书第三十二节明确要求不能只靠 CSV），
CSV 只作为导出格式。支持按时间区间和字段子集导出，
避免一个 10 小时的实验导出成几个 GB 的文本文件。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import h5py
import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description="EtherCAT Joint Control 数据导出")
    ap.add_argument("h5file")
    ap.add_argument("-o", "--output", help="输出 CSV（默认与输入同名）")
    ap.add_argument("--fields", help="逗号分隔的字段子集，默认全部")
    ap.add_argument("--start", type=float, default=None, help="起始 elapsed_time_s")
    ap.add_argument("--end", type=float, default=None, help="结束 elapsed_time_s")
    ap.add_argument("--decimate", type=int, default=1,
                    help="抽稀倍数，例如 10 表示每 10 个样本取 1 个")
    ap.add_argument("--list", action="store_true", help="只列出字段与 metadata")
    args = ap.parse_args()

    p = Path(args.h5file)
    if not p.is_file():
        print(f"文件不存在: {p}", file=sys.stderr)
        return 1

    with h5py.File(p, "r") as h:
        g = h["/experiment"]

        if args.list:
            print("── metadata ──")
            for k, v in g.attrs.items():
                print(f"  {k:34s} = {v.decode() if isinstance(v, bytes) else v}")
            print("\n── 字段 ──")
            for k in g.keys():
                d = g[k]
                print(f"  {k:34s} {d.dtype}  n={d.shape[0]}")
            return 0

        names = list(g.keys())
        if args.fields:
            want = [f.strip() for f in args.fields.split(",")]
            unknown = [f for f in want if f not in names]
            if unknown:
                print(f"未知字段: {', '.join(unknown)}\n可用字段: {', '.join(names)}",
                      file=sys.stderr)
                return 1
            names = want
        # elapsed_time_s 总是放在第一列，便于直接画图
        if "elapsed_time_s" in g and "elapsed_time_s" not in names:
            names.insert(0, "elapsed_time_s")
        elif "elapsed_time_s" in names:
            names.remove("elapsed_time_s")
            names.insert(0, "elapsed_time_s")

        t = g["elapsed_time_s"][:]
        mask = np.ones(len(t), dtype=bool)
        if args.start is not None:
            mask &= t >= args.start
        if args.end is not None:
            mask &= t <= args.end
        idx = np.nonzero(mask)[0]
        if args.decimate > 1:
            idx = idx[::args.decimate]
        if len(idx) == 0:
            print("选定区间内没有数据", file=sys.stderr)
            return 1

        cols = [g[n][:][idx] for n in names]

        out = Path(args.output) if args.output else p.with_suffix(".csv")
        with open(out, "w", encoding="utf-8") as f:
            # 把 metadata 作为注释写进头部，导出的 CSV 也能自证来源
            for k, v in g.attrs.items():
                v = v.decode() if isinstance(v, bytes) else v
                f.write(f"# {k}: {v}\n")
            f.write(",".join(names) + "\n")
            for row in zip(*cols):
                f.write(",".join(
                    f"{x:.9g}" if isinstance(x, (float, np.floating)) else str(x)
                    for x in row) + "\n")

        print(f"已导出 {len(idx):,} 行 × {len(names)} 列 → {out}")
        print(f"大小: {out.stat().st_size/1e6:.2f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
