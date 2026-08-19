import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gui"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import h5py
import numpy as np
import yaml

import h5_to_csv as htc
import experiment_naming as en


def _fixed_str_attr(grp, name, value: str):
    """模拟后端 data_logger.cpp 的字符串 attr 写法（H5Tcopy(H5T_C_S1) 定长字符串）：
    h5py 读回是 bytes/numpy.bytes_，不是 str。终审 C2 (export_a1 忘 decode，
    CSV/.meta.yaml 里变成 "b'A01'") 必须在这种 fixture 上才能复现——用
    `grp.attrs[name] = "A01"` 造的 fixture 是可变长 UTF-8 字符串，读回就是 str，
    根本测不到这个 bug（这正是终审指出的"fixture 与真实后端行为分叉"）。
    """
    grp.attrs.create(name, np.bytes_(value.encode("ascii")))


def _make_h5(path):
    """真实后端形状的最小 HDF5：dataset 名/单位取自
    backend/include/ecjc/data_logger.hpp 的 ECJC_SAMPLE_COLUMNS（deg / rpm，
    不是 A.1 要的 rad / rad_s），字符串 attr 用定长 H5T_C_S1 模拟。

    motor_position_deg / output_position_deg 故意跟对应的 _unwrapped_deg 在最后
    一个样本上不同（模拟过零回绕），用来验证 export_a1 优先取 unwrapped 那一路。
    """
    with h5py.File(path, "w") as f:
        g = f.create_group("experiment")
        n = 5
        g.create_dataset("elapsed_time_s", data=np.arange(n, dtype=float))
        g.create_dataset("motor_position_deg",
                          data=np.array([0, 90, 180, 270, 0], dtype=float))
        g.create_dataset("motor_position_unwrapped_deg",
                          data=np.array([0, 90, 180, 270, 360], dtype=float))
        g.create_dataset("output_position_deg",
                          data=np.array([0, 1, 2, 3, 0], dtype=float))
        g.create_dataset("output_position_unwrapped_deg",
                          data=np.array([0, 1, 2, 3, 4], dtype=float))
        g.create_dataset("motor_velocity_rpm", data=np.full(n, 30.0))
        g.create_dataset("output_velocity_rpm", data=np.full(n, 0.25))
        g.create_dataset("motor_current_A", data=np.zeros(n))
        _fixed_str_attr(g, "sample_id", "A01")
        _fixed_str_attr(g, "baseline_stage", "life_node")
        _fixed_str_attr(g, "test_item", "TE")
        _fixed_str_attr(g, "operator", "tyy")
        g.attrs["gear_ratio"] = 121.0


def test_export_a1_writes_empty_columns_as_blank(tmp_path):
    h5 = tmp_path / "x.h5"; _make_h5(str(h5))
    out = tmp_path / "out.csv"
    htc.export_a1(str(h5), str(out))
    rows = list(csv.reader(open(out)))
    header = rows[0]
    # 三个留空列必须在表头
    for c in en.EMPTY_COLUMNS:
        assert c in header
    # 留空列的值是空字段(不是 "NA")
    i = header.index("temperature_joint_C")
    assert rows[1][i] == ""


def test_export_a1_writes_meta_yaml(tmp_path):
    h5 = tmp_path / "x.h5"; _make_h5(str(h5))
    out = tmp_path / "out.csv"
    htc.export_a1(str(h5), str(out))
    meta = tmp_path / "out.meta.yaml"
    assert meta.exists()
    d = yaml.safe_load(open(meta))
    assert "empty_columns" in d
    assert d.get("sample_id") == "A01"


def test_export_a1_kinematics_columns_are_populated_in_rad(tmp_path):
    """终审 C1：timestamp_s / theta_*_rad / omega_*_rad_s 必须有数据，且单位是
    rad / rad_s（不是原始 deg / rpm 数值原样抄过去）。"""
    h5 = tmp_path / "x.h5"; _make_h5(str(h5))
    out = tmp_path / "out.csv"
    htc.export_a1(str(h5), str(out))
    rows = list(csv.reader(open(out)))
    header = rows[0]
    for c in ("timestamp_s", "theta_in_rad", "theta_out_rad",
              "omega_in_rad_s", "omega_out_rad_s"):
        assert c in header
    data = rows[1:]
    assert len(data) == 5

    def col(name):
        i = header.index(name)
        return [float(r[i]) for r in data]

    ts = col("timestamp_s")
    assert ts == [0.0, 1.0, 2.0, 3.0, 4.0]

    theta_in = col("theta_in_rad")
    theta_out = col("theta_out_rad")
    omega_in = col("omega_in_rad_s")
    omega_out = col("omega_out_rad_s")

    # 非空、数量级是 rad（几个 rad），不是 deg 数值本身（360）原样抄过去
    assert all(v != "" for v in theta_in)
    assert max(theta_in) < 7.0  # 2π ≈ 6.283；若没转换会看到 360 这种量级

    # 优先取 unwrapped：最后一个样本 wrapped=0deg / unwrapped=360deg，
    # 若代码错误地选了 wrapped 会读到 0，而不是 2π
    assert abs(theta_in[-1] - math.radians(360)) < 1e-9
    assert abs(theta_out[-1] - math.radians(4)) < 1e-9

    # 30 rpm -> pi rad/s；0.25 rpm -> ~0.02618 rad/s
    assert all(abs(v - math.pi) < 1e-6 for v in omega_in)
    assert all(abs(v - (0.25 * 2 * math.pi / 60)) < 1e-6 for v in omega_out)


def test_export_a1_scales_to_real_run_length(tmp_path):
    """真机 bug（2026-08-13）：export_a1 曾逐元素读 HDF5（g[ds][i] 双重循环），
    每行 ~5ms——2kHz 采样下摩擦工况 8 万行要 ~7 分钟，全堵在 GUI 主线程，
    GNOME 弹"python 无响应"。必须整列读（向量化）。5000 行给 5s 上限：
    向量化 <1s 稳过；逐元素 ~26s 必挂。"""
    n = 5000
    h5 = tmp_path / "big.h5"
    with h5py.File(str(h5), "w") as f:
        g = f.create_group("experiment")
        g.create_dataset("elapsed_time_s", data=np.arange(n) / 2000.0)
        for name in ("motor_position_unwrapped_deg", "output_position_unwrapped_deg",
                     "motor_velocity_rpm", "output_velocity_rpm", "motor_current_A",
                     "target_position_deg", "torque_Nm", "bus_voltage_V"):
            g.create_dataset(name, data=np.random.rand(n))
        _fixed_str_attr(g, "sample_id", "A01")
    out = tmp_path / "big.csv"
    t0 = time.perf_counter()
    htc.export_a1(str(h5), str(out))
    dt = time.perf_counter() - t0
    assert dt < 5.0, f"export_a1 花了 {dt:.1f}s / {n} 行——又退化成逐元素读了?"
    rows = list(csv.reader(open(out)))
    assert len(rows) == n + 1


def test_export_a1_tolerates_ragged_columns(tmp_path):
    """真机 bug（2026-08-13）：DataLogger::stop() 与 writer 线程的批次写入有
    竞态，closeFile 可能打断逐列 extend——前几列比后几列多出最后一个批次
    （实测 A01 传动误差文件：elapsed 105163 行 vs 其余 105158）。导出必须
    按所有列的最小长度截齐，不能 IndexError 崩掉。"""
    h5 = tmp_path / "ragged.h5"
    with h5py.File(str(h5), "w") as f:
        g = f.create_group("experiment")
        g.create_dataset("elapsed_time_s", data=np.arange(10, dtype=float))
        g.create_dataset("motor_velocity_rpm", data=np.full(7, 30.0))
        g.create_dataset("motor_current_A", data=np.zeros(7))
        _fixed_str_attr(g, "sample_id", "A01")
    out = tmp_path / "ragged.csv"
    htc.export_a1(str(h5), str(out))
    rows = list(csv.reader(open(out)))
    assert len(rows) == 7 + 1  # 截到最短列，含表头


def test_export_a1_string_attrs_are_decoded_not_bytes_repr(tmp_path):
    """终审 C2：定长字符串 attr 读回是 bytes，export_a1 必须 decode，
    CSV 与 .meta.yaml 里都不能出现 "b'A01'" 这种 bytes repr。"""
    h5 = tmp_path / "x.h5"; _make_h5(str(h5))
    out = tmp_path / "out.csv"
    htc.export_a1(str(h5), str(out))

    rows = list(csv.reader(open(out)))
    header = rows[0]
    i = header.index("sample_id")
    assert rows[1][i] == "A01"
    assert "b'" not in rows[1][i]

    j = header.index("test_item")
    assert rows[1][j] == "TE"

    meta = yaml.safe_load(open(tmp_path / "out.meta.yaml"))
    assert meta.get("sample_id") == "A01"
    assert meta.get("test_item") == "TE"
    assert "b'" not in str(meta.get("sample_id"))
