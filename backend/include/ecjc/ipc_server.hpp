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
    void handleLine(const std::string& json);
    void sendJson(const std::string& json);
    void sendFrame(uint16_t type, const void* payload, size_t len);
    void sendFrameTo(int fd, uint16_t type, const void* payload, size_t len);
    void dropClient(int fd);
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

    // 慢客户端丢帧：::send 设了 SO_SNDTIMEO，超时即丢这一帧而不是拖死
    // telemetry/accept 线程（见 sendFrameTo）。总数用于诊断"GUI 曾经冻结过"，
    // 也透出到 statusJson()。json_drop_warned_ 记录哪些 fd 已经因为丢失
    // JSON 帧（应答/状态/日志，比遥测丢帧严重）被记过一次日志——每个连接只记一次，
    // 避免慢客户端把日志刷屏；连接断开时在 dropClient() 里清掉对应条目。
    std::atomic<uint64_t> slow_client_drops_{0};
    std::mutex warn_mu_;
    std::unordered_set<int> json_drop_warned_;

    BusAction do_connect_, do_disconnect_, do_reconnect_, do_reset_encoder_;
    std::vector<Sample> tele_buf_;
};

}  // namespace ecjc
