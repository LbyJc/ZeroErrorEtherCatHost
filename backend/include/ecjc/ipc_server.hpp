// ipc_server.hpp —— Unix domain socket 服务端。
//
// 一条流上跑两种帧（见 ARCHITECTURE.md §4）：
//   type=1 遥测：N × Sample，固定二进制，高频
//   type=2 JSON：命令 / 状态 / 日志 / 参数表，低频、自描述
//
// 选 Unix socket 而不是共享内存的核心理由：**有连接生命周期**。
// GUI 崩了 / 关了，服务端立刻从 read() 拿到 0 或 EPOLLHUP，
// 于是可以主动进入安全状态，而不是对着一块陈旧的共享内存无知无觉。
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ecjc/config.hpp"
#include "ecjc/data_logger.hpp"
#include "ecjc/realtime_task.hpp"

namespace ecjc {

class IpcServer {
public:
    IpcServer(const FullConfig& cfg, RealtimeTask* rt, DataLogger* logger);
    ~IpcServer();

    bool start(const std::string& socket_path, std::string* err);
    void stop();

    /// 启动过程的分步进度：给 GUI 发 {"ev":"startup",...}
    void reportStep(const std::string& step, bool ok, const std::string& msg);
    void log(const std::string& level, const std::string& msg);

    /// 由 main 注入：GUI 点【启动主站】/【停止主站】时执行的动作
    using BusAction = std::function<bool(std::string* err)>;
    void setBusActions(BusAction connect, BusAction disconnect, BusAction reconnect) {
        do_connect_ = std::move(connect);
        do_disconnect_ = std::move(disconnect);
        do_reconnect_ = std::move(reconnect);
    }
    void setResetEncoderAction(BusAction a) { do_reset_encoder_ = std::move(a); }

    bool hasClient() const;

private:
    void threadMain();
    void serveClient(int fd);          ///< 每个客户端一个线程
    void telemetryLoop();
    void graceLoop();                  ///< 重连宽限期计时线程，见 clients_ 附近的注释（finding C3）
    void handleLine(const std::string& json);
    void sendJson(const std::string& json);
    void sendFrame(uint16_t type, const void* payload, size_t len);
    void sendFrameTo(int fd, uint16_t type, const void* payload, size_t len);
    void dropClient(int fd);
    void evictClient(int fd);          ///< 摘出 clients_ 并 shutdown()；close() 仍由拥有者线程经 dropClient() 做
    void forgetClient(int fd);         ///< 清掉 per_client_drops_/evicted_fds_ 里这个 fd 的记账
    void beginReconnectGrace();        ///< 最后一个客户端被"踢出"（非主动断开）时启动/续期宽限倒计时
    void cancelReconnectGrace();       ///< 有新客户端连入时取消倒计时
    std::string statusJson();
    std::string paramsJson();
    std::string recordingJson();

    FullConfig cfg_;
    RealtimeTask* rt_;
    DataLogger* logger_;

    int listen_fd_ = -1;
    std::string socket_path_;

    // 多客户端：GUI 与调试脚本可以同时连。
    // 原来是"accept 之后就阻塞在该连接的读循环里"，第二个客户端会卡在 backlog
    // 里永远拿不到数据——调试时必须先关掉 GUI，很难受。
    static constexpr size_t kMaxClients = 8;
    mutable std::mutex clients_mu_;
    std::vector<int> clients_;
    std::vector<std::thread> client_threads_;
    // fd 是被 evictClient() 主动踢掉的（协议撕裂 / 长期发不出去数据），
    // 还是自己正常断开（进程退出、主动 close）的——dropClient() 靠这张表
    // 区分这两种情况，只有前者才走"重连宽限期"而不是立刻 stopRun（见下面
    // kReconnectGraceSec 的注释，finding C3）。跟 clients_ 共用 clients_mu_，
    // evictClient() 里"摘出 clients_"和"标记为已踢出"是原子的一步。
    std::unordered_set<int> evicted_fds_;

    std::thread accept_th_, telemetry_th_, grace_th_;
    std::atomic<bool> quit_{false};
    std::mutex send_mu_;

    // ── 重连宽限期（finding C3） ─────────────────────────────────────────
    // 背景：慢客户端被 evictClient() 踢掉后，dropClient() 原来的逻辑是
    // "最后一个客户端消失就 stopRun()"——对着一个真的走了的操作者这是对的，
    // 但对着一个只是被踢掉、GUI 本身还开着、2 秒后就会自动重连
    // （gui/ipc_client.py 的 _retry 定时器）的连接，这个判定过于激进：
    // 450h 无人值守实验里，GUI 冻结/笔记本休眠攒够 kSlowClientEvictAfterDrops
    // 次丢帧（一次 60s 冻结就够）会被踢，若立刻 stopRun，一次冻结就中止整个
    // 实验；改之前的行为是"冻结只降级遥测，实验继续"，不该在加了超时丢帧之后
    // 变成"冻结会中止实验"。
    //
    // 方案（终审推荐，备选是给 evictClient 的 fd 打标记让 dropClient 跳过
    // stopRun 判定——那样等价于完全恢复"踢出不影响实验"的旧语义，但会让
    // "真的没人了"这件事永远发现不了；宽限期是二者的折中：给重连留够时间，
    // 但超时之后仍然会停，不会永久悬挂）：
    // 最后一个客户端是因为被踢（不是主动断开）而消失时，不立即 stopRun()，
    // 而是启动这个宽限倒计时；期间只要有任意客户端连入（不管是不是原来那个
    // GUI）就取消倒计时；到期仍然没有客户端才真正 stopRun()。
    static constexpr int kReconnectGraceSec = 30;
    std::mutex grace_mu_;
    std::condition_variable grace_cv_;
    bool grace_pending_ = false;
    uint64_t grace_epoch_ = 0;   // 每次开始新宽限期/取消都递增，graceLoop() 靠它判断"还是不是同一次"

    // 慢客户端丢帧：::send 设了 SO_SNDTIMEO=200ms，超时即丢这一帧而不是拖死
    // telemetry/accept 线程（见 sendFrameTo）。但如果超时发生在**帧发到一半**
    // （帧头或 payload 已经部分写出去之后），这一帧不能只是丢弃——接收侧
    // （gui/ipc_client.py、tests/hw_driver.py）是纯长度前缀流，没有重同步能力，
    // 半帧会导致后续所有帧被错位解析、协议流永久失步。这种情况必须直接断开
    // 该连接（evictClient，见 sendFrameTo 里的 Torn 分支），逼客户端重连拿一条
    // 干净的流。只有恰好卡在帧边界（一个字节都没发出去）的超时才能安全丢帧。
    //
    // slow_client_drops_ 是跨全部连接的累计总数，用于诊断"GUI 曾经冻结过"，
    // 也透出到 statusJson()——它不按连接区分，多个慢客户端同时出现时，
    // 要看单条日志里带的 fd 才能定位是谁。
    //
    // per_client_drops_ 按 fd 记这条连接自己的丢帧数（一直卡在帧边界、没有触发
    // 上面 Torn 分支的情况）：
    //   - 从 0 变成 1（这条连接第一次丢帧）时，如果丢的是 JSON 帧
    //     （应答/状态/日志，比遥测丢帧严重——客户端会以为命令没有响应），
    //     立即记一条日志，且只记这一次，不按 100 取模，避免刷屏；
    //   - 累计达到 kSlowClientEvictAfterDrops（这条连接终生丢帧数，不是"连续"，
    //     粗粒度但零热路径开销）时判定它长期发不出去数据，主动 evictClient()
    //     踢掉——否则一个总是恰好卡在帧边界、因此永远碰不到 Torn 分支的客户端
    //     会占着连接名额、让每次广播都白等 200ms，直到天荒地老。
    // fd 会被后续 accept() 复用，断开时（dropClient/forgetClient）必须清掉对应
    // 条目，否则新连接会"继承"旧连接的丢帧史。
    static constexpr uint32_t kSlowClientEvictAfterDrops = 300;  // ×最多200ms ≈ 累计 60s
    std::atomic<uint64_t> slow_client_drops_{0};
    std::mutex slow_client_mu_;
    std::unordered_map<int, uint32_t> per_client_drops_;

    BusAction do_connect_, do_disconnect_, do_reconnect_, do_reset_encoder_;
    std::vector<Sample> tele_buf_;
};

}  // namespace ecjc
