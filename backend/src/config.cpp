#include "ecjc/config.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <filesystem>
#include <fstream>

namespace ecjc {
namespace {

namespace fs = std::filesystem;

// 支持 0x 前缀的整数
uint32_t asHex(const YAML::Node& n, uint32_t def) {
    if (!n) return def;
    const std::string s = n.as<std::string>();
    if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
        return static_cast<uint32_t>(std::stoul(s, nullptr, 16));
    return static_cast<uint32_t>(std::stoul(s, nullptr, 10));
}

template <typename T>
T get(const YAML::Node& n, const char* key, T def) {
    if (!n || !n[key]) return def;
    try { return n[key].as<T>(); } catch (...) { return def; }
}

// 终审 finding I4①：diagnostic_sdos / async_sdo 的 type 字段此前不做任何
// 合法性校验——写错一个 typo（比如 "u64"）会静默落到 blockingSdoReadTyped /
// pollAsyncSdo 的"未识别类型回落到 s32"分支，读出来的数值看着正常，实际上
// 位宽全错，没有任何报错线索。
const char* kValidSdoTypes = "u8, i8, u16, i16, u32, i32";

bool isValidSdoType(const std::string& t) {
    return t == "u8" || t == "i8" || t == "u16" || t == "i16" ||
           t == "u32" || t == "i32";
}

std::string hex04(uint32_t v) {
    char b[8];
    snprintf(b, sizeof b, "0x%04X", v);
    return b;
}

std::string hex02(uint32_t v) {
    char b[8];
    snprintf(b, sizeof b, "%02X", v);
    return b;
}

bool loadFile(const fs::path& p, YAML::Node* out, std::string* err) {
    if (!fs::exists(p)) {
        *err = "配置文件不存在: " + p.string();
        return false;
    }
    try { *out = YAML::LoadFile(p.string()); }
    catch (const std::exception& e) {
        *err = "配置文件 " + p.string() + " 解析失败: " + e.what();
        return false;
    }
    return true;
}

void parsePdoList(const YAML::Node& list, std::vector<PdoCfg>* out) {
    if (!list) return;
    for (const auto& p : list) {
        PdoCfg c;
        c.index = asHex(p["index"], 0);
        c.watchdog = get<bool>(p, "watchdog", false);
        for (const auto& e : p["entries"]) {
            PdoEntryCfg ec;
            ec.index = asHex(e["index"], 0);
            ec.sub   = static_cast<uint8_t>(asHex(e["sub"], 0));
            ec.bits  = static_cast<uint8_t>(get<int>(e, "bits", 0));
            ec.type  = get<std::string>(e, "type", "");
            ec.name  = get<std::string>(e, "name", "");
            c.entries.push_back(ec);
        }
        out->push_back(c);
    }
}

}  // namespace

bool loadConfig(const std::string& dir, FullConfig* o, std::string* err) {
    const fs::path d(dir);
    if (!fs::is_directory(d)) {
        *err = "配置目录不存在: " + dir;
        return false;
    }
    o->config_dir = dir;

    YAML::Node n;

    // ── app.yaml ──
    if (!loadFile(d / "app.yaml", &n, err)) return false;
    if (auto a = n["app"]) {
        o->app.name              = get<std::string>(a, "name", o->app.name);
        o->app.version           = get<std::string>(a, "version", o->app.version);
        o->app.auto_start_master = get<bool>(a, "auto_start_master", false);
        o->app.socket_path       = get<std::string>(a, "socket_path", o->app.socket_path);
        o->app.socket_path_dev   = get<std::string>(a, "socket_path_dev", o->app.socket_path_dev);
        o->app.data_dir          = get<std::string>(a, "data_dir", o->app.data_dir);
        o->app.log_dir           = get<std::string>(a, "log_dir", o->app.log_dir);
    }
    if (auto l = n["logging"]) o->app.log_level = get<std::string>(l, "level", "INFO");

    // ── ethercat.yaml ──
    if (!loadFile(d / "ethercat.yaml", &n, err)) return false;
    if (auto e = n["ethercat"]) {
        o->ethercat.master_index  = get<unsigned>(e, "master_index", 0);
        o->ethercat.interface     = get<std::string>(e, "interface", "enp3s0");
        o->ethercat.cycle_us      = get<unsigned>(e, "cycle_us", 1000);
        o->ethercat.dc_enabled    = get<bool>(e, "dc_enabled", true);
        o->ethercat.dc_assign_activate = asHex(e["dc_assign_activate"], 0x0300);
        o->ethercat.dc_sync0_shift_ns  = get<int>(e, "dc_sync0_shift_ns", 0);
        o->ethercat.op_timeout_ms      = get<unsigned>(e, "op_timeout_ms", 10000);
        o->ethercat.wc_error_tolerance = get<unsigned>(e, "wc_error_tolerance", 5);
    }
    if (auto r = n["realtime"]) {
        o->realtime.sched_priority    = get<int>(r, "sched_priority", 80);
        o->realtime.stack_prefault_kb = get<int>(r, "stack_prefault_kb", 8192);
        o->realtime.lock_memory       = get<bool>(r, "lock_memory", true);
        o->realtime.log_ring_capacity = get<size_t>(r, "log_ring_capacity", 16384);
        o->realtime.gui_ring_capacity = get<size_t>(r, "gui_ring_capacity", 4096);
        if (r["cpu_affinity"])
            for (const auto& c : r["cpu_affinity"]) o->realtime.cpu_affinity.push_back(c.as<int>());
    }
    if (o->ethercat.cycle_us == 0) {
        *err = "ethercat.yaml: cycle_us 不能为 0";
        return false;
    }

    // ── slave.yaml ──
    if (!loadFile(d / "slave.yaml", &n, err)) return false;
    if (auto s = n["slave"]) {
        o->slave.alias        = static_cast<uint16_t>(get<int>(s, "alias", 0));
        o->slave.position     = static_cast<uint16_t>(get<int>(s, "position", 0));
        o->slave.vendor_id    = asHex(s["vendor_id"], 0);
        o->slave.product_code = asHex(s["product_code"], 0);
        o->slave.name         = get<std::string>(s, "name", "");
        o->slave.supported_modes_raw = asHex(s["supported_modes_raw"], 0);
        o->slave.supports_homing     = get<bool>(s, "supports_homing", false);
        o->slave.min_cycle_us        = get<unsigned>(s, "min_cycle_us", 0);
    }

    // 周期与从站能力的一致性校验。
    // 不校验的话，cycle_us 设得比从站支持的还快，表现是"从站卡在 PREOP"，
    // 排查起来完全看不出跟周期有关（实测 250 µs 就是这样）。
    if (o->slave.min_cycle_us > 0) {
        const unsigned m = o->slave.min_cycle_us;
        if (o->ethercat.cycle_us < m) {
            *err = "ethercat.yaml: cycle_us = " + std::to_string(o->ethercat.cycle_us) +
                   " µs 快于从站支持的最小通信周期 " + std::to_string(m) +
                   " µs（见 slave.yaml 的 min_cycle_us）。"
                   "设得过快时从站会在 PREOP 阶段直接拒绝配置，连 SAFEOP 都进不去。";
            return false;
        }
        if (o->ethercat.cycle_us % m != 0) {
            *err = "ethercat.yaml: cycle_us = " + std::to_string(o->ethercat.cycle_us) +
                   " µs 不是从站最小周期 " + std::to_string(m) +
                   " µs 的整数倍。厂商要求主站周期必须是该值的倍数"
                   "（1x, 2x, 3x…），否则 DC 同步无法建立。";
            return false;
        }
    }
    if (n["diagnostic_sdos"]) {
        for (const auto& a : n["diagnostic_sdos"]) {
            DiagnosticSdoCfg c;
            c.index = static_cast<uint16_t>(asHex(a["index"], 0));
            c.sub   = static_cast<uint8_t>(asHex(a["sub"], 0));
            c.type  = get<std::string>(a, "type", "u32");
            c.name  = get<std::string>(a, "name", "");
            if (c.index == 0) {
                *err = "slave.yaml: diagnostic_sdos 条目缺少 index";
                return false;
            }
            if (!isValidSdoType(c.type)) {
                *err = "slave.yaml: diagnostic_sdos 条目 0x" + hex04(c.index) + ":" +
                       hex02(c.sub) + " (" + c.name + ") 的 type 非法: \"" + c.type +
                       "\"（合法值: " + kValidSdoTypes + "）";
                return false;
            }
            o->slave.diagnostic_sdos.push_back(c);
        }
    }
    if (n["startup_sdo"]) {
        for (const auto& a : n["startup_sdo"]) {
            StartupSdoCfg c;
            c.index = static_cast<uint16_t>(asHex(a["index"], 0));
            c.sub   = static_cast<uint8_t>(asHex(a["sub"], 0));
            c.type  = get<std::string>(a, "type", "u32");
            c.value = get<int64_t>(a, "value", 0);
            c.name  = get<std::string>(a, "name", "");
            c.comment = get<std::string>(a, "comment", "");
            if (c.index == 0) {
                *err = "slave.yaml: startup_sdo 条目缺少 index";
                return false;
            }
            o->startup_sdos.push_back(c);
        }
    }
    if (o->slave.vendor_id == 0 || o->slave.product_code == 0) {
        *err = "slave.yaml: vendor_id / product_code 必须填写实际值（用 `ethercat slaves -v` 读取）";
        return false;
    }

    // ── pdo.yaml ──
    if (!loadFile(d / "pdo.yaml", &n, err)) return false;
    if (auto p = n["pdo"]) {
        parsePdoList(p["rx"], &o->rx_pdos);
        parsePdoList(p["tx"], &o->tx_pdos);
    }
    if (o->rx_pdos.empty() || o->tx_pdos.empty()) {
        *err = "pdo.yaml: rx / tx PDO 列表不能为空";
        return false;
    }
    if (n["async_sdo"]) {
        for (const auto& a : n["async_sdo"]) {
            AsyncSdoCfg c;
            c.index = asHex(a["index"], 0);
            c.sub   = static_cast<uint8_t>(asHex(a["sub"], 0));
            c.type  = get<std::string>(a, "type", "i32");
            c.name  = get<std::string>(a, "name", "");
            c.poll_divisor = get<int>(a, "poll_divisor", 1);
            if (c.poll_divisor < 1) c.poll_divisor = 1;
            if (!isValidSdoType(c.type)) {
                *err = "pdo.yaml: async_sdo 条目 0x" + hex04(c.index) + ":" +
                       hex02(c.sub) + " (" + c.name + ") 的 type 非法: \"" + c.type +
                       "\"（合法值: " + kValidSdoTypes + "）";
                return false;
            }
            o->async_sdos.push_back(c);
        }
    }

    // ── scaling.yaml ──
    if (!loadFile(d / "scaling.yaml", &n, err)) return false;
    if (auto s = n["scaling"]) {
        auto& sc = o->scaling;
        sc.motor_counts_per_rev  = get<double>(s, "motor_encoder_counts_per_rev", 131072.0);
        sc.output_counts_per_rev = get<double>(s, "output_encoder_counts_per_rev", 524288.0);
        sc.resolution_verified   = get<bool>(s, "encoder_resolution_verified", false);
        sc.gear_ratio            = get<double>(s, "gear_ratio", 121.0);
        sc.velocity_gain_correction = get<double>(s, "velocity_gain_correction", 1.0);
        sc.rated_current_mA = get<double>(s, "rated_current_mA", 6300.0);
        sc.rated_torque_mNm = get<double>(s, "rated_torque_mNm", 31000.0);
        sc.current_scale    = get<double>(s, "current_scale", 0.001);
        sc.torque_scale     = get<double>(s, "torque_scale", 0.001);
        sc.position_direction = get<int>(s, "position_direction", 1);
        sc.velocity_direction = get<int>(s, "velocity_direction", 1);
        sc.current_direction  = get<int>(s, "current_direction", 1);
        // 默认值必须与 ScalingConfig 的 struct 默认值一致（true / 0 / 0），
        // 否则缺 key 的旧配置目录会在这里悄悄漂移出与代码默认不一致的行为
        // （参见 P0 finding I1 的教训：stop_ramp.csp_hold_position 曾经这样）。
        sc.target_velocity_is_motor_side =
            get<bool>(s, "target_velocity_is_motor_side", true);
        sc.motor_position_modulus =
            static_cast<int64_t>(get<double>(s, "motor_position_modulus", 0.0));
        sc.output_position_modulus =
            static_cast<int64_t>(get<double>(s, "output_position_modulus", 0.0));
        if (auto lim = s["limits"]) {
            sc.motor_velocity_rpm_max  = get<double>(lim, "motor_velocity_rpm_max", 3000.0);
            sc.output_velocity_rpm_max = get<double>(lim, "output_velocity_rpm_max", 25.0);
            sc.torque_Nm_max           = get<double>(lim, "torque_Nm_max", 20.0);
            sc.current_A_max           = get<double>(lim, "current_A_max", 6.3);
            sc.csp_target_jump_deg_max = get<double>(lim, "csp_target_jump_deg_max", 5.0);
        }
        if (sc.motor_counts_per_rev <= 0 || sc.output_counts_per_rev <= 0 || sc.gear_ratio <= 0) {
            *err = "scaling.yaml: 编码器分辨率与减速比必须为正数";
            return false;
        }
    }

    // ── gui.yaml ──
    if (!loadFile(d / "gui.yaml", &n, err)) return false;
    if (auto g = n["gui"]) {
        o->gui.telemetry_publish_hz  = get<int>(g, "telemetry_publish_hz", 100);
        o->gui.decimate_keep_extremes = get<bool>(g, "decimate_keep_extremes", true);
        if (o->gui.telemetry_publish_hz < 1) o->gui.telemetry_publish_hz = 1;
    }

    // ── trajectory.yaml ──
    if (!loadFile(d / "trajectory.yaml", &n, err)) return false;
    if (auto t = n["trajectory"]) {
        std::string ty = get<std::string>(t, "default_type", "constant");
        if (!parseTrajType(ty, &o->trajectory.type)) o->trajectory.type = TrajType::Constant;
        if (auto c = t["constant"]) o->trajectory.constant_value = get<double>(c, "csv_rpm", 100.0);
        if (auto s = t["sine"]) {
            o->trajectory.offset       = get<double>(s, "offset", 0.0);
            o->trajectory.amplitude    = get<double>(s, "amplitude", 10.0);
            o->trajectory.frequency_hz = get<double>(s, "frequency_hz", 0.5);
            o->trajectory.phase_deg    = get<double>(s, "phase_deg", 0.0);
            o->trajectory.duration_s   = get<double>(s, "duration_s", -1.0);
        }
        if (auto r = t["ramp"]) {
            o->trajectory.ramp_initial    = get<double>(r, "initial", 0.0);
            o->trajectory.ramp_final      = get<double>(r, "final", 100.0);
            o->trajectory.ramp_duration_s = get<double>(r, "duration_s", 5.0);
        }
        if (auto z = t["trapezoidal"]) {
            o->trajectory.trapz_target_deg = get<double>(z, "target_position_deg", 90.0);
            o->trajectory.trapz_vmax_rpm   = get<double>(z, "max_velocity_rpm", 50.0);
            o->trajectory.trapz_acc        = get<double>(z, "acceleration", 200.0);
            o->trajectory.trapz_dec        = get<double>(z, "deceleration", 200.0);
        }
        if (auto s = t["stop_ramp"]) {
            o->stop_ramp.csv_decel_rpm_per_s = get<double>(s, "csv_decel_rpm_per_s", 200.0);
            o->stop_ramp.cst_decel_Nm_per_s  = get<double>(s, "cst_decel_Nm_per_s", 5.0);
            // 默认值必须和 StopRampCfg 的 struct 默认值（false）、config/*.yaml
            // 里的显式值保持一致——这三处曾经只改了两处，缺 key 的旧配置目录
            // （没写 csp_hold_position 这一行）会在这里悄悄复活"保持 Run 起始
            // 位置"的危险默认行为（终审 finding I1）。
            o->stop_ramp.csp_hold_position   = get<bool>(s, "csp_hold_position", false);
            o->stop_ramp.disable_timeout_cycles =
                static_cast<uint64_t>(get<double>(s, "disable_timeout_cycles", 15000.0));
        }
    }

    // ── controller.yaml ──
    if (!loadFile(d / "controller.yaml", &n, err)) return false;
    if (auto c = n["controller"]) {
        o->controller.default_id = get<std::string>(c, "default", "passthrough");
        if (auto l = c["output_limits"]) {
            o->controller.torque_Nm_limit     = get<double>(l, "torque_Nm", 20.0);
            o->controller.torque_rate_Nm_per_s = get<double>(l, "torque_rate_Nm_per_s", 100.0);
            o->controller.velocity_rate_rpm_per_s =
                get<double>(l, "velocity_rate_rpm_per_s", 1.65);
            o->controller.csp_position_rate_deg_per_s =
                get<double>(l, "csp_position_rate_deg_per_s", 15.0);
        }
    }

    return true;
}

}  // namespace ecjc
