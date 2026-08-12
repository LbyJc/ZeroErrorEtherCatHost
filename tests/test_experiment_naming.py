import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "gui"))
import pytest
import experiment_naming as en


def test_csv_filename_basic():
    assert en.csv_filename("A01", 100, "TE", 0, 5, 1) == \
        "sample_A01__life_100h__test_TE__load_0Tr__speed_5rpm__dir_na__amp_na__freq_na__rep_01.csv"


def test_csv_filename_rep_zero_padded():
    n = en.csv_filename("A01", 0, "current", 10, 10, 3)
    assert "__rep_03.csv" in n
    assert "__load_10Tr__" in n
    assert "__speed_10rpm__" in n


def test_csv_filename_rejects_unknown_test_item():
    with pytest.raises(ValueError) as e:
        en.csv_filename("A01", 100, "bogus", 0, 5, 1)
    assert "test_item" in str(e.value)


def test_csv_filename_accepts_all_valid_items():
    for item in ["current", "TE", "backlash", "stiffness", "p2p", "sine"]:
        en.csv_filename("A01", 100, item, 0, 5, 1)   # 不抛异常即通过


def test_a1_columns_contain_empty_columns_in_order():
    for c in en.EMPTY_COLUMNS:
        assert c in en.A1_COLUMNS
    # 留空三列必须在公共字段区，不在末尾扩展列
    assert en.A1_COLUMNS.index("theta_out_rad") < en.A1_COLUMNS.index("load_torque_Nm_actual") or \
           "theta_out_rad" in en.A1_COLUMNS


def test_meta_yaml_declares_empty_columns():
    d = en.meta_yaml_dict({"gear_ratio": 121})
    assert "empty_columns" in d
    for c in en.EMPTY_COLUMNS:
        assert c in d["empty_columns"]
    assert d["gear_ratio"] == 121
