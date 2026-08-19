#include "test_framework.hpp"
#include "ecjc/config.hpp"
#include "ecjc/moving_time_store.hpp"

#include <cstdio>
#include <string>
#include <unistd.h>

using namespace ecjc;

// ── 关节「在转」判定：isJointMoving ─────────────────────────────────────
// 2026-08-18 需求：本次/累计运行时长只统计关节实际转动的时间。
// 阈值 0.5 rpm（输出侧）由用户拍板：高于静止残余读数（当天软停后实测
// 0.2~0.25 rpm），低于最慢实验工况（CSV 2 rpm 低速采数）。

TEST(joint_moving_above_threshold) {
    CHECK(isJointMoving(0.6));
    CHECK(isJointMoving(2.0));     // 最慢工况必须计入
    CHECK(isJointMoving(25.0));
}

TEST(joint_moving_is_direction_agnostic) {
    CHECK(isJointMoving(-0.6));
    CHECK(isJointMoving(-25.0));   // 反转同样算在转
}

TEST(joint_not_moving_at_or_below_threshold) {
    CHECK(!isJointMoving(0.0));
    CHECK(!isJointMoving(0.25));   // 停机后的静止残余读数不许计时
    CHECK(!isJointMoving(-0.25));
    CHECK(!isJointMoving(0.5));    // 边界值本身不算（严格大于）
}

// ── 累计值持久化：MovingTimeStore ───────────────────────────────────────
// 跨后端重启续算。写盘必须原子（tmp+rename），断电只丢最后一个写盘周期。

static std::string tempPath() {
    char buf[] = "/tmp/ecjc_moving_time_XXXXXX";
    const int fd = ::mkstemp(buf);
    CHECK(fd >= 0);
    ::close(fd);
    ::unlink(buf);                 // 只要路径；文件由被测代码创建
    return std::string(buf);
}

TEST(store_roundtrip_persists_value) {
    const std::string path = tempPath();
    MovingTimeStore store(path);
    std::string err;
    CHECK(store.save(123456789012345LL, &err));
    CHECK_EQ(store.load(), 123456789012345LL);
    ::unlink(path.c_str());
}

TEST(store_missing_file_loads_zero) {
    MovingTimeStore store("/tmp/ecjc_moving_time_definitely_missing");
    CHECK_EQ(store.load(), 0LL);
}

TEST(store_corrupt_file_loads_zero) {
    const std::string path = tempPath();
    FILE* f = ::fopen(path.c_str(), "w");
    CHECK(f != nullptr);
    ::fputs("not a number", f);
    ::fclose(f);
    MovingTimeStore store(path);
    CHECK_EQ(store.load(), 0LL);
    ::unlink(path.c_str());
}

TEST(store_overwrite_keeps_latest) {
    const std::string path = tempPath();
    MovingTimeStore store(path);
    std::string err;
    CHECK(store.save(1000, &err));
    CHECK(store.save(2000, &err));
    CHECK_EQ(store.load(), 2000LL);
    ::unlink(path.c_str());
}

TEST(store_creates_missing_parent_directory) {
    // 部署首启时 log_dir 可能还不存在，save 要能自己把父目录建出来
    const std::string dir = tempPath();          // 独占路径当目录用
    const std::string path = dir + "/sub/moving_time_total_ns";
    MovingTimeStore store(path);
    std::string err;
    CHECK(store.save(42, &err));
    CHECK_EQ(store.load(), 42LL);
    ::unlink(path.c_str());
    ::rmdir((dir + "/sub").c_str());
    ::rmdir(dir.c_str());
}
