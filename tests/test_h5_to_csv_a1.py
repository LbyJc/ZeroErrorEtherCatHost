import sys, os, csv
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gui"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import h5py, numpy as np, yaml, pytest
import h5_to_csv as htc
import experiment_naming as en


def _make_h5(path):
    with h5py.File(path, "w") as f:
        g = f.create_group("experiment")
        n = 5
        g.create_dataset("elapsed_time_s", data=np.arange(n, dtype=float))
        g.create_dataset("theta_out_unwrapped_deg", data=np.arange(n, dtype=float))
        g.create_dataset("motor_current_A", data=np.zeros(n))
        g.attrs["sample_id"] = "A01"
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
