// config.cpp 此前完全没有测试，正因如此三个 scaling 键
// （target_velocity_is_motor_side / motor_position_modulus /
// output_position_modulus）与 slave.yaml 的 diagnostic_sdos
// 漏解析至今无人察觉。本文件在**临时目录**里造最小配置，
// 不依赖也不触碰工程自带的 config/。
#include "test_framework.hpp"
#include "ecjc/config.hpp"

#include <cstdio>
#include <filesystem>
#include <string>

using namespace ecjc;

namespace {

// 每个用例用独立的临时子目录（含用例名），避免用例之间/并发运行时互相踩踏。
std::filesystem::path makeConfigDir(const std::string& case_tag,
                                     const std::string& scaling_extra) {
    auto dir = std::filesystem::temp_directory_path() / ("ecjc_cfg_test_" + case_tag);
    std::filesystem::create_directories(dir);
    auto write = [&](const char* fn, const std::string& body) {
        FILE* f = fopen((dir / fn).c_str(), "w");
        fwrite(body.data(), 1, body.size(), f);
        fclose(f);
    };
    write("app.yaml", "app:\n  socket_path: \"/tmp/x.sock\"\n");
    write("ethercat.yaml", "ethercat:\n  cycle_us: 1000\n");
    write("slave.yaml",
          "slave:\n  vendor_id: 0x5a65726f\n  product_code: 0x00029252\n"
          "  min_cycle_us: 500\n"
          "diagnostic_sdos:\n"
          "  - {index: 0x6093, sub: 0x01, type: u32, name: \"position_factor_num\"}\n"
          "  - {index: 0x6075, sub: 0x00, type: u32, name: \"rated_current_mA\"}\n");
    write("pdo.yaml",
          "pdo:\n  rx:\n    - index: 0x1605\n      entries:\n"
          "        - {index: 0x6040, sub: 0x00, bits: 16, type: u16, name: controlword}\n"
          "  tx:\n    - index: 0x1A06\n      entries:\n"
          "        - {index: 0x6041, sub: 0x00, bits: 16, type: u16, name: statusword}\n");
    write("scaling.yaml", "scaling:\n" + scaling_extra);
    write("controller.yaml", "controller:\n  default: passthrough\n");
    write("trajectory.yaml", "trajectory:\n  default_type: constant\n");
    write("gui.yaml", "gui:\n  plot_hz: 50\n");
    return dir;
}

}  // namespace

TEST(config_parses_target_velocity_is_motor_side) {
    auto dir = makeConfigDir("tvms", "  target_velocity_is_motor_side: false\n");
    FullConfig cfg; std::string err;
    CHECK(loadConfig(dir.string(), &cfg, &err));
    CHECK(cfg.scaling.target_velocity_is_motor_side == false);
}

TEST(config_parses_position_modulus) {
    auto dir = makeConfigDir("modulus",
                              "  motor_position_modulus: 131072\n"
                              "  output_position_modulus: 524288\n");
    FullConfig cfg; std::string err;
    CHECK(loadConfig(dir.string(), &cfg, &err));
    CHECK_EQ(cfg.scaling.motor_position_modulus, 131072);
    CHECK_EQ(cfg.scaling.output_position_modulus, 524288);
}

TEST(config_parses_diagnostic_sdos) {
    auto dir = makeConfigDir("diag", "  gear_ratio: 121.0\n");
    FullConfig cfg; std::string err;
    CHECK(loadConfig(dir.string(), &cfg, &err));
    CHECK_EQ((int)cfg.slave.diagnostic_sdos.size(), 2);
    CHECK_EQ((int)cfg.slave.diagnostic_sdos[0].index, 0x6093);
    CHECK_EQ((int)cfg.slave.diagnostic_sdos[0].sub, 0x01);
    CHECK(cfg.slave.diagnostic_sdos[1].name == "rated_current_mA");
}

// 拒绝路径：减速比为 0 必须被拦下并给出人能看懂的错误
TEST(config_rejects_zero_gear_ratio) {
    auto dir = makeConfigDir("zerogear", "  gear_ratio: 0\n");
    FullConfig cfg; std::string err;
    CHECK(!loadConfig(dir.string(), &cfg, &err));
    CHECK(err.find("减速比") != std::string::npos);
}
