#include "test_framework.hpp"
#include "ecjc/data_logger.hpp"
#include "ecjc/config.hpp"
#include "ecjc/ring_buffer.hpp"
#include "ecjc/types.hpp"

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

using namespace ecjc;

// column_count_matches_writer_count（sampleColumnCount() == sampleWriterCount()）
// 曾经在这里，但审查发现它是直通调用的假恒等：sampleWriterCount() 就是
// `return sampleColumnCount();`，两边按定义相等，永远不可能失败——防不住
// writeBatch() 里 X-macro 展开点被改动后漏写/多写某一列。
//
// 真正防这个的是 writeBatch()（data_logger.cpp）里新增的运行期校验：
// 宏链展开写完之后检查 k == impl_->cols.size()，不等则以清晰错误拒绝整批写入。
// 下面这条测试把 DataLogger 端到端跑一遍，实际穿过 writeBatch 的这条校验路径
// （而不是像旧测试那样只调用两个查询函数），是一条真能失败的测试：
// 如果未来有人让某个 tag 漏展开/重复展开，start() 之后的写入会失败，
// samples 计数对不上 kN，这条用例就会红。
TEST(writer_persists_all_samples_through_real_datalogger) {
    auto dir = std::filesystem::temp_directory_path() / "ecjc_logger_columns_test";
    std::filesystem::create_directories(dir);

    FullConfig cfg;
    cfg.app.data_dir = dir.string();

    SpscRing<Sample> ring;
    ring.init(256);

    DataLogger logger(cfg, &ring);
    logger.setMinFreeGb(0.0);  // 沙盒 /tmp 空间可能很小，不因磁盘阈值误判失败

    RecordingMeta meta;
    meta.test_name = "unittest";
    std::string err;
    CHECK(logger.start(meta, &err));

    constexpr size_t kN = 50;
    for (size_t i = 0; i < kN; ++i) {
        Sample s{};
        s.system_time_ns = static_cast<int64_t>(i);
        s.seq = static_cast<uint32_t>(i);
        s.flags = 0x01;
        ring.push(s);
    }

    // 等 Logger 线程把 ring 排空并写盘（轮询，带超时，避免测试挂死）
    for (int i = 0; i < 300 && logger.status().samples < kN && logger.status().active; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

    logger.stop();

    CHECK(logger.status().active == false);
    CHECK_EQ((int)logger.status().samples, (int)kN);
}

// 列名不得重复（重复会让 H5Dcreate2 失败或后一列覆盖前一列）
TEST(column_names_are_unique) {
    auto names = sampleColumnNames();
    for (size_t i = 0; i < names.size(); ++i)
        for (size_t j = i + 1; j < names.size(); ++j)
            CHECK(names[i] != names[j]);
}

// 关键列必须在场——回归保护，防止有人重构时把它们弄丢
TEST(required_columns_present) {
    auto names = sampleColumnNames();
    auto has = [&](const char* n) {
        for (auto& s : names) if (s == n) return true;
        return false;
    };
    CHECK(has("system_time_ns"));
    CHECK(has("motor_position_raw"));
    CHECK(has("output_position_raw"));
    CHECK(has("seq"));        // 现在缺，事后无法从文件检测丢包
    CHECK(has("flags"));      // 现在缺
}
