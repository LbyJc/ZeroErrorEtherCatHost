"""实验数据命名与 A.1 字段布局（纯函数，无 Qt 依赖）。

依据《整机关节不可拆在线寿命实验方案》附录 A.1（CSV 公共字段）与 §4.2（命名规范），
以及《2026-08-11 实验计划修订建议 v2》的扩展命名模板（加 dir/amp/freq）。
"""
from __future__ import annotations

VALID_TEST_ITEMS = {"current", "TE", "backlash", "stiffness", "p2p", "sine", "RP"}

# 总线测不了、CSV 里写空字段（不写 NA）的三列。语义见 meta_yaml_dict。
EMPTY_COLUMNS = ["load_torque_Nm_actual", "temperature_motor_C", "temperature_joint_C"]

# A.1 公共字段有序列表（含三个留空列）。顺序即 CSV 列顺序的前半段；扩展列排在其后。
A1_COLUMNS = [
    "sample_id", "life_hours", "baseline_stage", "test_item",
    "load_percent_Tr", "load_torque_Nm_target", "load_torque_Nm_actual",
    "speed_rpm_target", "timestamp_s",
    "theta_in_rad", "theta_out_rad", "omega_in_rad_s", "omega_out_rad_s",
    "motor_current_A", "temperature_motor_C", "temperature_joint_C",
    "theta_out_ref_deg", "theta_in_ref_phase_deg", "theta_in_phase_deg",
    "phase_error_in_deg", "phase_tol_in_deg", "zero_approach_direction",
    "mounting_phase_mark", "operator", "notes",
]


def _norm_num(x):
    """去掉整数值浮点数的 .0（GUI 的 QDoubleSpinBox.value() 直传进来是 float，
    100.0 会拼成文件名 life_100.0h，下游 life_(\\d+)h 的正则解析不上——终审 I4）。
    非整数值原样返回，不做四舍五入。"""
    try:
        f = float(x)
    except (TypeError, ValueError):
        return x
    return int(f) if f == int(f) else f


def csv_filename(sample_id, life_hours, test_item, load_pct, speed_rpm, rep,
                 direction="na", amp_deg="na", freq_hz="na", stamp=None) -> str:
    """按 §4.2 扩展模板拼 CSV 文件名。test_item 必须属受控词表。
    stamp（如 20260814_103000，与 h5 的 fileStamp 同源）非空时附加
    __t_<stamp> 段——区分同工况多次运行，也防重名互相覆盖（2026-08-14 需求）。"""
    if test_item not in VALID_TEST_ITEMS:
        raise ValueError(
            f"test_item '{test_item}' 不在受控词表 {sorted(VALID_TEST_ITEMS)}")
    life_hours = _norm_num(life_hours)
    load_pct = _norm_num(load_pct)
    speed_rpm = _norm_num(speed_rpm)
    tail = f"__t_{stamp}" if stamp else ""
    return (f"sample_{sample_id}__life_{life_hours}h__test_{test_item}"
            f"__load_{load_pct}Tr__speed_{speed_rpm}rpm"
            f"__dir_{direction}__amp_{amp_deg}__freq_{freq_hz}"
            f"__rep_{int(rep):02d}{tail}.csv")


def meta_yaml_dict(calib: dict) -> dict:
    """生成 .meta.yaml 内容：标定声明 + 三个留空列的原因说明。"""
    d = dict(calib)
    d["empty_columns"] = {
        "load_torque_Nm_actual":
            "外部转矩传感器，未采集；0x3B69 是关节自估传递转矩，语义不同，未填此列",
        "temperature_motor_C": "总线无绕组温度，未采集",
        "temperature_joint_C": "外部传感器，未采集",
    }
    return d
