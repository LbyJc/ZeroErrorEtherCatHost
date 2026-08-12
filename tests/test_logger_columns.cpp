#include "test_framework.hpp"
#include "ecjc/data_logger.hpp"

using namespace ecjc;

// 列定义与写入序列必须来自同一份声明，数量恒等
TEST(column_count_matches_writer_count) {
    CHECK_EQ((int)sampleColumnCount(), (int)sampleWriterCount());
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
