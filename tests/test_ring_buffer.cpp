#include "test_framework.hpp"
#include "ecjc/ring_buffer.hpp"
#include "ecjc/types.hpp"

#include <atomic>
#include <thread>

using namespace ecjc;

TEST(容量向上取整到2的幂) {
    CHECK_EQ(roundUpPow2(1), 2u);
    CHECK_EQ(roundUpPow2(3), 4u);
    CHECK_EQ(roundUpPow2(1000), 1024u);
    CHECK_EQ(roundUpPow2(1024), 1024u);
}

TEST(基本进出) {
    SpscRing<int> r;
    r.init(8);
    CHECK_EQ(r.size(), 0u);
    for (int i = 0; i < 5; ++i) CHECK(r.push(i));
    CHECK_EQ(r.size(), 5u);

    int v = 0;
    for (int i = 0; i < 5; ++i) { CHECK(r.pop(&v)); CHECK_EQ(v, i); }
    CHECK_EQ(r.size(), 0u);
    CHECK(!r.pop(&v));
}

TEST(满了就丢并计数_绝不阻塞) {
    // 这是 RT 安全的关键：宁可丢遥测，也不能让实时线程等在这里
    SpscRing<int> r;
    r.init(4);
    for (int i = 0; i < 4; ++i) CHECK(r.push(i));
    CHECK(!r.push(99));            // 满了，返回 false 而不是卡住
    CHECK_EQ(r.dropped(), 1u);
    CHECK(!r.push(100));
    CHECK_EQ(r.dropped(), 2u);
    CHECK_EQ(r.size(), 4u);        // 已有数据不受影响
}

TEST(批量取) {
    SpscRing<int> r;
    r.init(16);
    for (int i = 0; i < 10; ++i) r.push(i);
    int buf[16] = {0};
    CHECK_EQ(r.popBatch(buf, 16), 10u);
    for (int i = 0; i < 10; ++i) CHECK_EQ(buf[i], i);
    CHECK_EQ(r.popBatch(buf, 16), 0u);
}

TEST(环回复用不越界) {
    SpscRing<int> r;
    r.init(4);
    int v;
    for (int round = 0; round < 100; ++round) {
        CHECK(r.push(round));
        CHECK(r.pop(&v));
        CHECK_EQ(v, round);
    }
}

TEST(单生产者单消费者并发不丢不乱序) {
    SpscRing<int> r;
    r.init(1024);
    constexpr int N = 200000;
    std::atomic<bool> go{false};

    std::thread prod([&] {
        while (!go.load()) {}
        for (int i = 0; i < N; ) if (r.push(i)) ++i;   // 满了就重试，测的是正确性
    });

    int expect = 0;
    go.store(true);
    int v;
    while (expect < N) if (r.pop(&v)) { CHECK_EQ(v, expect); ++expect; }
    prod.join();
    CHECK_EQ(expect, N);
}

TEST(Sample结构体尺寸锁定) {
    // GUI 的 struct 格式串按 200 字节写死（协议 v3，见 test_wire_format.cpp），这里守住它
    CHECK_EQ(sizeof(Sample), 200u);
    CHECK_EQ(sizeof(FrameHeader), 12u);
}
