#include "test_framework.hpp"
#include "ecjc/data_logger.hpp"
#include <filesystem>

using namespace ecjc;

// RecordingMeta 必须携带实验字段，且 out_dir 非空时覆盖默认目录
TEST(recording_meta_holds_experiment_fields) {
    RecordingMeta m;
    m.sample_id = "A01";
    m.baseline_stage = "life_node";
    m.life_hours = 100.0;
    m.test_item = "TE";
    m.rep = 1;
    m.load_percent_Tr = 0.0;
    m.speed_rpm_target = 5.0;
    CHECK(m.sample_id == "A01");
    CHECK(m.test_item == "TE");
    CHECK_EQ((int)m.rep, 1);
}

TEST(recording_cfg_out_dir_overrides_default) {
    // out_dir 空 → 用 app.data_dir；非空 → 用 out_dir
    CHECK(recordingTargetDir("", "/var/lib/x") == std::string("/var/lib/x"));
    CHECK(recordingTargetDir("/tmp/exp", "/var/lib/x") == std::string("/tmp/exp"));
}
