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
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
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
    void handleLine(const std::string& json);
    void sendJson(const std::string& json);
    void sendFrame(uint16_t type, const void* payload, size_t len);
    void sendFrameTo(int fd, uint16_t type, const void* payload, size_t len);
    void dropClient(int fd);
    void evictClient(int fd);          ///< 摘出 clients_ 并 shutdown()；close() 仍由拥有者线程经 dropClient() 做
    void forgetClient(int fd);         ///< 清掉 per_client_drops_ 里这个 fd 的记账
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

    std::thread accept_th_, telemetry_th_;
    std::atomic<bool> quit_{false};
    std::mutex send_mu_;

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
