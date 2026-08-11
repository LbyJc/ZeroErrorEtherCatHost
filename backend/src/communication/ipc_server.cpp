#include "ecjc/ipc_server.hpp"

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <grp.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <map>
#include <sstream>
#include <thread>

namespace ecjc {
namespace fs = std::filesystem;
namespace {

// ── 极简 JSON ────────────────────────────────────────────────────────────
// 只用于低频命令与状态，不值得为此引入一个 JSON 库依赖。
// 解析器只支持扁平对象（这也正是协议的全部需要），遇到嵌套会安全地忽略。
std::string esc(const std::string& s) {
    std::string o;
    o.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"':  o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n";  break;
            case '\r': o += "\\r";  break;
            case '\t': o += "\\t";  break;
            default:   o += c;      break;
        }
    }
    return o;
}

std::string num(double v) {
    if (!std::isfinite(v)) return "null";
    char b[40];
    snprintf(b, sizeof b, "%.6g", v);
    return b;
}

class Json {
public:
    explicit Json(const std::string& s) { parse(s); }
    bool has(const std::string& k) const { return kv_.count(k) > 0; }
    std::string str(const std::string& k, const std::string& def = "") const {
        auto it = kv_.find(k); return it == kv_.end() ? def : it->second;
    }
    double dbl(const std::string& k, double def = 0) const {
        auto it = kv_.find(k);
        if (it == kv_.end()) return def;
        try { return std::stod(it->second); } catch (...) { return def; }
    }
    bool boolean(const std::string& k, bool def = false) const {
        auto it = kv_.find(k);
        if (it == kv_.end()) return def;
        return it->second == "true" || it->second == "1";
    }

private:
    void parse(const std::string& s) {
        size_t i = 0;
        auto skip = [&] { while (i < s.size() && isspace((unsigned char)s[i])) ++i; };
        skip();
        if (i >= s.size() || s[i] != '{') return;
        ++i;
        int depth = 0;
        while (i < s.size()) {
            skip();
            if (i < s.size() && s[i] == '}') break;
            if (i >= s.size() || s[i] != '"') { ++i; continue; }
            std::string key = readStr(s, i);
            skip();
            if (i >= s.size() || s[i] != ':') continue;
            ++i; skip();
            std::string val;
            if (i < s.size() && s[i] == '"') val = readStr(s, i);
            else if (i < s.size() && (s[i] == '{' || s[i] == '[')) {
                // 嵌套：整段跳过，不参与扁平表
                const char open = s[i], close = (open == '{') ? '}' : ']';
                depth = 0;
                while (i < s.size()) {
                    if (s[i] == open) ++depth;
                    else if (s[i] == close) { if (--depth == 0) { ++i; break; } }
                    ++i;
                }
            } else {
                const size_t st = i;
                while (i < s.size() && s[i] != ',' && s[i] != '}') ++i;
                val = s.substr(st, i - st);
                while (!val.empty() && isspace((unsigned char)val.back())) val.pop_back();
            }
            kv_[key] = val;
            skip();
            if (i < s.size() && s[i] == ',') ++i;
        }
    }
    /// 把一个 Unicode 码点编码成 UTF-8 追加到 out
    static void appendUtf8(std::string& out, uint32_t cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    static bool hex4(const std::string& s, size_t pos, uint32_t* out) {
        if (pos + 4 > s.size()) return false;
        uint32_t v = 0;
        for (int k = 0; k < 4; ++k) {
            const char c = s[pos + k];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= uint32_t(c - '0');
            else if (c >= 'a' && c <= 'f') v |= uint32_t(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= uint32_t(c - 'A' + 10);
            else return false;
        }
        *out = v;
        return true;
    }

    static std::string readStr(const std::string& s, size_t& i) {
        ++i;  // 跳过起始引号
        std::string o;
        while (i < s.size() && s[i] != '"') {
            if (s[i] == '\\' && i + 1 < s.size()) {
                ++i;
                switch (s[i]) {
                    case 'n': o += '\n'; break;
                    case 't': o += '\t'; break;
                    case 'r': o += '\r'; break;
                    case 'b': o += '\b'; break;
                    case 'f': o += '\f'; break;
                    case 'u': {
                        // \uXXXX 转义。不处理它的话，任何 ensure_ascii=True 的
                        // 标准 JSON 客户端发来的中文都会变成 "u81eau52a8" 这种垃圾，
                        // 而 description 这类字段是要永久写进实验 metadata 的。
                        uint32_t cp = 0;
                        if (!hex4(s, i + 1, &cp)) { o += 'u'; break; }
                        i += 4;
                        // UTF-16 代理对：高位 D800-DBFF 后面必须跟低位 DC00-DFFF
                        if (cp >= 0xD800 && cp <= 0xDBFF &&
                            i + 6 < s.size() && s[i + 1] == '\\' && s[i + 2] == 'u') {
                            uint32_t lo = 0;
                            if (hex4(s, i + 3, &lo) && lo >= 0xDC00 && lo <= 0xDFFF) {
                                cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                                i += 6;
                            }
                        }
                        appendUtf8(o, cp);
                        break;
                    }
                    default:  o += s[i]; break;
                }
            } else o += s[i];
            ++i;
        }
        if (i < s.size()) ++i;
        return o;
    }
    std::map<std::string, std::string> kv_;
};

}  // namespace

IpcServer::IpcServer(const FullConfig& cfg, RealtimeTask* rt, DataLogger* logger)
    : cfg_(cfg), rt_(rt), logger_(logger) {
    tele_buf_.resize(4096);
}

IpcServer::~IpcServer() { stop(); }

bool IpcServer::start(const std::string& path, std::string* err) {
    socket_path_ = path;
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    ::unlink(path.c_str());

    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) { *err = std::string("创建 socket 失败: ") + strerror(errno); return false; }

    struct sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (path.size() >= sizeof(addr.sun_path)) {
        *err = "socket 路径过长: " + path;
        return false;
    }
    std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

    if (::bind(listen_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        *err = "绑定 socket " + path + " 失败: " + strerror(errno) +
               "（该路径是否已被占用？目录是否存在？）";
        ::close(listen_fd_); listen_fd_ = -1;
        return false;
    }
    // 权限：属组 ethercat、0660。GUI 以普通用户身份连接，不需要任何特权。
    ::chmod(path.c_str(), 0660);
    if (struct group* g = ::getgrnam("ethercat"))
        ::chown(path.c_str(), 0, g->gr_gid);
    else
        ::chmod(path.c_str(), 0666);   // 没建组时退化，开发环境用

    if (::listen(listen_fd_, 4) < 0) {
        *err = std::string("listen 失败: ") + strerror(errno);
        return false;
    }

    quit_.store(false);
    accept_th_ = std::thread([this] { threadMain(); });
    telemetry_th_ = std::thread([this] { telemetryLoop(); });
    return true;
}

void IpcServer::stop() {
    quit_.store(true);
    if (listen_fd_ >= 0) { ::shutdown(listen_fd_, SHUT_RDWR); ::close(listen_fd_); listen_fd_ = -1; }

    // shutdown 而不是 close：让各客户端线程的阻塞 read() 立刻返回 0 退出，
    // fd 本身由 dropClient() 负责关，避免二次 close。
    {
        std::lock_guard<std::mutex> lk(clients_mu_);
        for (int fd : clients_) ::shutdown(fd, SHUT_RDWR);
    }
    std::vector<std::thread> ths;
    {
        std::lock_guard<std::mutex> lk(clients_mu_);
        ths.swap(client_threads_);
    }
    for (auto& t : ths) if (t.joinable()) t.join();

    if (accept_th_.joinable()) accept_th_.join();
    if (telemetry_th_.joinable()) telemetry_th_.join();
    if (!socket_path_.empty()) ::unlink(socket_path_.c_str());
}

bool IpcServer::hasClient() const {
    std::lock_guard<std::mutex> lk(clients_mu_);
    return !clients_.empty();
}

void IpcServer::threadMain() {
    while (!quit_.load()) {
        const int fd = ::accept(listen_fd_, nullptr, nullptr);
        if (fd < 0) { if (quit_.load()) break; usleep(100000); continue; }

        {
            std::unique_lock<std::mutex> lk(clients_mu_);
            if (clients_.size() >= kMaxClients) {
                lk.unlock();
                // 明确拒绝而不是静默挂起：挂起的连接最难排查
                const std::string msg =
                    "{\"ev\":\"log\",\"level\":\"ERROR\","
                    "\"msg\":\"客户端数量已达上限，拒绝连接\"}";
                sendFrameTo(fd, static_cast<uint16_t>(FrameType::Json),
                            msg.data(), msg.size());
                ::close(fd);
                continue;
            }
            clients_.push_back(fd);
        }

        log("INFO", "客户端已连接");
        // 新客户端先单独收一份当前状态，不用等下一个广播周期
        const std::string st = statusJson(), pa = paramsJson();
        sendFrameTo(fd, static_cast<uint16_t>(FrameType::Json), st.data(), st.size());
        sendFrameTo(fd, static_cast<uint16_t>(FrameType::Json), pa.data(), pa.size());

        // 每个客户端一个线程。清理在 stop() 里统一 join。
        {
            std::lock_guard<std::mutex> lk(clients_mu_);
            client_threads_.emplace_back([this, fd] { serveClient(fd); });
        }
    }
}

void IpcServer::serveClient(int fd) {
    std::string buf;
    char tmp[4096];
    while (!quit_.load()) {
        const ssize_t n = ::read(fd, tmp, sizeof tmp);
        if (n <= 0) break;               // 0 = 正常关闭；<0 = 出错
        buf.append(tmp, static_cast<size_t>(n));
        size_t pos;
        while ((pos = buf.find('\n')) != std::string::npos) {
            const std::string line = buf.substr(0, pos);
            buf.erase(0, pos + 1);
            if (!line.empty()) handleLine(line);
        }
    }
    dropClient(fd);
}

void IpcServer::dropClient(int fd) {
    bool last = false;
    {
        std::lock_guard<std::mutex> lk(clients_mu_);
        for (size_t i = 0; i < clients_.size(); ++i) {
            if (clients_[i] == fd) { clients_.erase(clients_.begin() + i); break; }
        }
        last = clients_.empty();
    }
    ::close(fd);

    // 只有**最后一个**客户端断开才算"失去操作者"。
    // 多客户端下若任一断开就停机，调试脚本退出会误停实验。
    if (last && !quit_.load()) {
        rt_->stopRun();
        log("WARNING", "最后一个客户端已断开，已停止运行（伺服保持使能状态）");
    }
}

void IpcServer::sendFrameTo(int fd, uint16_t type, const void* payload, size_t len) {
    if (fd < 0) return;
    FrameHeader h{kFrameMagic, type, kProtocolVersion, static_cast<uint32_t>(len)};

    // send_mu_ 保证一帧不会被另一个线程的帧插进来切碎
    std::lock_guard<std::mutex> lk(send_mu_);
    // MSG_NOSIGNAL：客户端突然消失时不要给我们一个 SIGPIPE 把进程干掉
    if (::send(fd, &h, sizeof h, MSG_NOSIGNAL) != static_cast<ssize_t>(sizeof h)) return;
    size_t off = 0;
    const char* p = static_cast<const char*>(payload);
    while (off < len) {
        const ssize_t n = ::send(fd, p + off, len - off, MSG_NOSIGNAL);
        if (n <= 0) return;
        off += static_cast<size_t>(n);
    }
}

/// 广播给所有客户端
void IpcServer::sendFrame(uint16_t type, const void* payload, size_t len) {
    std::vector<int> fds;
    {
        std::lock_guard<std::mutex> lk(clients_mu_);
        fds = clients_;
    }
    for (int fd : fds) sendFrameTo(fd, type, payload, len);
}

void IpcServer::sendJson(const std::string& j) {
    sendFrame(static_cast<uint16_t>(FrameType::Json), j.data(), j.size());
}

void IpcServer::reportStep(const std::string& step, bool ok, const std::string& msg) {
    sendJson("{\"ev\":\"startup\",\"step\":\"" + esc(step) + "\",\"ok\":" +
             (ok ? "true" : "false") + ",\"msg\":\"" + esc(msg) + "\"}");
}

void IpcServer::log(const std::string& level, const std::string& msg) {
    const auto now = std::chrono::system_clock::now();
    const int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
    sendJson("{\"ev\":\"log\",\"level\":\"" + level + "\",\"ts\":" +
             std::to_string(ns) + ",\"msg\":\"" + esc(msg) + "\"}");
    fprintf(stderr, "[%s] %s\n", level.c_str(), msg.c_str());
}

std::string IpcServer::statusJson() {
    const StatusSnapshot s = rt_->snapshot();
    std::ostringstream o;
    o << "{\"ev\":\"status\""
      << ",\"app_state\":\"" << toString(s.app_state) << "\""
      << ",\"ethercat\":\""  << toString(s.bus.slave_state) << "\""
      << ",\"master\":\""    << (s.bus.slave_online ? "Running" : "Stopped") << "\""
      << ",\"slave_online\":" << (s.bus.slave_online ? "true" : "false")
      << ",\"slave_operational\":" << (s.bus.slave_operational ? "true" : "false")
      << ",\"slave_count\":" << s.bus.slave_count
      << ",\"slave_name\":\"" << esc(cfg_.slave.name) << "\""
      << ",\"link_up\":"     << (s.bus.link_up ? "true" : "false")
      << ",\"interface\":\"" << esc(cfg_.ethercat.interface) << "\""
      << ",\"wc\":"          << s.bus.working_counter
      << ",\"wc_state\":"    << s.bus.wc_state
      << ",\"dc_ok\":"       << (s.bus.dc_ok ? "true" : "false")
      << ",\"servo\":\""     << toString(s.cia_state) << "\""
      << ",\"controlword\":" << s.controlword
      << ",\"statusword\":"  << s.statusword
      << ",\"error_code\":"  << s.error_code
      // 状态字 bit7(Warning) 置位时，具体原因在这里。
      // 它一直在 PDO 里，之前忘了透出来，结果真机上看到 Warning 却查不到是什么。
      << ",\"warning_code\":" << s.warning_code
      << ",\"warning\":"     << ((s.statusword & (1u << 7)) ? "true" : "false")
      << ",\"mode\":\""      << toString(s.mode_selected) << "\""
      << ",\"mode_display\":" << static_cast<int>(s.mode_display)
      << ",\"mode_matched\":" << (s.mode_matched ? "true" : "false")
      << ",\"running\":"     << (s.running ? "true" : "false")
      << ",\"run_time_s\":"  << num(s.run_time_s)
      << ",\"cycle_us\":"    << cfg_.ethercat.cycle_us
      << ",\"jitter_us\":"      << num(s.stats.jitter_ns / 1000.0)
      << ",\"jitter_max_us\":"  << num(s.stats.jitter_max_ns / 1000.0)
      << ",\"jitter_mean_us\":" << num(s.stats.jitter_mean_ns / 1000.0)
      << ",\"deadline_miss\":"  << s.stats.deadline_miss
      << ",\"cycles\":"         << s.stats.cycles
      << ",\"wc_errors\":"      << s.stats.wc_errors
      << ",\"dropped_log\":"    << s.stats.dropped_log
      << ",\"dropped_gui\":"    << s.stats.dropped_gui
      << ",\"encoder_verified\":" << (cfg_.scaling.resolution_verified ? "true" : "false")
      << ",\"gear_ratio\":"     << num(cfg_.scaling.gear_ratio)
      << ",\"supports_homing\":" << (cfg_.slave.supports_homing ? "true" : "false")
      << ",\"last_error\":\""   << esc(s.last_error) << "\""
      << "}";
    return o.str();
}

std::string IpcServer::paramsJson() {
    const auto metas = rt_->params().metas();
    const auto vals = rt_->params().values();
    std::ostringstream o;
    o << "{\"ev\":\"params\",\"items\":[";
    for (size_t i = 0; i < metas.size(); ++i) {
        if (i) o << ",";
        o << "{\"name\":\"" << esc(metas[i].name) << "\""
          << ",\"label\":\"" << esc(metas[i].label) << "\""
          << ",\"unit\":\"" << esc(metas[i].unit) << "\""
          << ",\"min\":" << num(metas[i].min_v)
          << ",\"max\":" << num(metas[i].max_v)
          << ",\"step\":" << num(metas[i].step)
          << ",\"value\":" << num(i < vals.size() ? vals[i] : metas[i].default_v)
          << "}";
    }
    o << "],\"controllers\":[";
    const auto ids = availableControllers();
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) o << ",";
        auto c = makeController(ids[i]);
        o << "{\"id\":\"" << ids[i] << "\",\"name\":\"" << esc(c ? c->name() : ids[i]) << "\"}";
    }
    o << "]}";
    return o.str();
}

std::string IpcServer::recordingJson() {
    const RecordingStatus r = logger_->status();
    std::ostringstream o;
    o << "{\"ev\":\"recording\""
      << ",\"active\":" << (r.active ? "true" : "false")
      << ",\"file\":\"" << esc(r.file) << "\""
      << ",\"samples\":" << r.samples
      << ",\"dropped\":" << r.dropped
      << ",\"bytes\":" << r.bytes
      << ",\"elapsed_s\":" << num(r.elapsed_s)
      << ",\"disk_free_gb\":" << num(r.disk_free_gb)
      << ",\"buffer_usage\":" << num(r.buffer_usage)
      << ",\"start_time_ns\":" << r.start_time_ns
      << "}";
    return o.str();
}

void IpcServer::handleLine(const std::string& line) {
    Json j(line);
    const std::string cmd = j.str("cmd");
    if (cmd.empty()) return;

    std::string err;
    bool ok = true;

    auto ack = [&](bool good, const std::string& msg) {
        sendJson("{\"ev\":\"ack\",\"cmd\":\"" + esc(cmd) + "\",\"ok\":" +
                 (good ? "true" : "false") + ",\"msg\":\"" + esc(msg) + "\"}");
    };

    if (cmd == "ping") {
        // GUI 启动自检：确认线格式尺寸一致，防止画出乱码曲线
        sendJson("{\"ev\":\"pong\",\"sample_size\":" + std::to_string(sizeof(Sample)) +
                 ",\"protocol\":" + std::to_string(kProtocolVersion) +
                 ",\"version\":\"" + esc(cfg_.app.version) + "\"}");
        return;
    }
    if (cmd == "get_status")    { sendJson(statusJson());    return; }
    if (cmd == "get_params")    { sendJson(paramsJson());    return; }
    if (cmd == "get_recording") { sendJson(recordingJson()); return; }

    if (cmd == "connect_bus") {
        ok = do_connect_ ? do_connect_(&err) : (err = "未配置总线连接动作", false);
    } else if (cmd == "disconnect_bus") {
        ok = do_disconnect_ ? do_disconnect_(&err) : (err = "未配置总线断开动作", false);
    } else if (cmd == "reconnect") {
        // 任务书第十节：重连成功后**禁止自动 Servo Enable**
        ok = do_reconnect_ ? do_reconnect_(&err) : (err = "未配置重连动作", false);
        if (ok) log("INFO", "重新连接完成。按要求不会自动使能，请手动点击 Servo Enable。");
    } else if (cmd == "reset_load_encoder") {
        ok = do_reset_encoder_ ? do_reset_encoder_(&err)
                               : (err = "未配置编码器重置动作", false);
    } else if (cmd == "servo_enable") {
        const StatusSnapshot s = rt_->snapshot();
        if (s.bus.slave_state != EcState::Op) {
            ok = false;
            err = "EtherCAT 未进入 OP（当前 " + std::string(toString(s.bus.slave_state)) +
                  "），禁止使能伺服";
        } else { rt_->servoEnable(); }
    } else if (cmd == "servo_disable") {
        rt_->servoDisable();
    } else if (cmd == "safe_stop") {
        // GUI 侧的"安全停机"入口：语义上与 servo_disable 相同——
        // 停止运行、置 stopping_、下 DisableVoltage 目标；真正的安全等待
        // （软停到 2.5 rpm 以下才真正切电）在 RT 主循环里统一做（见 realtime_task.cpp
        // 的 isSafeToDisableAt 门控，Task 3 已加）。这里不需要另开一个函数。
        rt_->servoDisable();
        log("INFO", "已请求安全停机：软停至 2.5 rpm 以下后撤使能");
    } else if (cmd == "fault_reset") {
        rt_->faultReset();
    } else if (cmd == "quick_stop") {
        rt_->quickStop();
    } else if (cmd == "homing") {
        ok = false;
        err = cfg_.slave.supports_homing
                  ? "Homing 尚未实现"
                  : "本驱动器不支持 Homing 模式（0x6502 = 0x38D，不含 hm 位）。"
                    "如需回零，请在 CSP 模式下用轨迹走到目标位置。";
    } else if (cmd == "set_mode") {
        OpMode m;
        if (!parseOpMode(j.str("mode").c_str(), &m)) { ok = false; err = "未知模式: " + j.str("mode"); }
        else ok = rt_->setMode(m, &err);
    } else if (cmd == "set_controller") {
        ok = rt_->setController(j.str("id"), &err);
        if (ok) sendJson(paramsJson());     // 参数表变了，GUI 会重建调参控件
    } else if (cmd == "set_param") {
        ok = rt_->params().set(j.str("name"), j.dbl("value"), &err);
    } else if (cmd == "set_target") {
        rt_->setTargetValue(j.dbl("value"));
    } else if (cmd == "set_trajectory") {
        TrajParams p;
        if (!parseTrajType(j.str("type", "constant"), &p.type)) { ok = false; err = "未知轨迹类型"; }
        else {
            p.constant_value  = j.dbl("value", 0);
            p.offset          = j.dbl("offset", 0);
            p.amplitude       = j.dbl("amplitude", 0);
            p.frequency_hz    = j.dbl("frequency_hz", 0);
            p.phase_deg       = j.dbl("phase_deg", 0);
            p.duration_s      = j.dbl("duration_s", -1);
            p.ramp_initial    = j.dbl("ramp_initial", 0);
            p.ramp_final      = j.dbl("ramp_final", 0);
            p.ramp_duration_s = j.dbl("ramp_duration_s", 1);
            p.trapz_target_deg= j.dbl("trapz_target_deg", 0);
            p.trapz_vmax_rpm  = j.dbl("trapz_vmax_rpm", 10);
            p.trapz_acc       = j.dbl("trapz_acc", 100);
            p.trapz_dec       = j.dbl("trapz_dec", 100);
            p.csv_path        = j.str("csv_path");
            p.csv_loop        = j.boolean("csv_loop");
            ok = rt_->setTrajectory(p, &err);
        }
    } else if (cmd == "start_run") {
        ok = rt_->startRun(&err);
        if (ok) log("INFO", "开始运行");
    } else if (cmd == "stop_run") {
        rt_->stopRun();
        log("INFO", "停止运行（软停，伺服保持使能）");
    } else if (cmd == "record_start") {
        RecordingMeta m;
        m.test_name   = j.str("test_name", "exp");
        m.description = j.str("description");
        const StatusSnapshot s = rt_->snapshot();
        m.operation_mode = toString(s.mode_selected);
        m.controller     = j.str("controller", "");
        m.cycle_us       = cfg_.ethercat.cycle_us;
        m.sampling_hz    = 1e6 / cfg_.ethercat.cycle_us;
        m.slave_name     = cfg_.slave.name;
        m.vendor_id      = cfg_.slave.vendor_id;
        m.product_code   = cfg_.slave.product_code;
        m.motor_counts_per_rev  = cfg_.scaling.motor_counts_per_rev;
        m.output_counts_per_rev = cfg_.scaling.output_counts_per_rev;
        m.gear_ratio            = cfg_.scaling.gear_ratio;
        m.encoder_resolution_verified = cfg_.scaling.resolution_verified;
        m.software_version = cfg_.app.version;
        {
            std::ostringstream ps;
            const auto metas = rt_->params().metas();
            const auto vals = rt_->params().values();
            ps << "{";
            for (size_t i = 0; i < metas.size(); ++i) {
                if (i) ps << ",";
                ps << "\"" << metas[i].name << "\":" << num(i < vals.size() ? vals[i] : 0);
            }
            ps << "}";
            m.control_params_json = ps.str();
        }
        ok = logger_->start(m, &err);
        if (ok) {
            rt_->setRecordEpoch(std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            rt_->setRecording(true);
            log("INFO", "开始数据采集");
        }
    } else if (cmd == "record_stop") {
        rt_->setRecording(false);
        logger_->stop();
        log("INFO", "停止数据采集");
        sendJson(recordingJson());
    } else {
        ok = false;
        err = "未知命令: " + cmd;
    }

    ack(ok, ok ? "" : err);
    if (!ok) log("ERROR", err);
}

// ── 遥测推送 ──────────────────────────────────────────────────────────────
void IpcServer::telemetryLoop() {
    const int hz = cfg_.gui.telemetry_publish_hz;
    const auto period = std::chrono::milliseconds(1000 / (hz > 0 ? hz : 50));
    auto next = std::chrono::steady_clock::now();
    int status_div = 0;

    while (!quit_.load()) {
        next += period;
        std::this_thread::sleep_until(next);
        if (!hasClient()) continue;

        const size_t n = rt_->guiRing().popBatch(tele_buf_.data(), tele_buf_.size());
        if (n > 0)
            sendFrame(static_cast<uint16_t>(FrameType::Telemetry),
                      tele_buf_.data(), n * sizeof(Sample));

        // 状态与采集信息以 10 Hz 单独推，不跟遥测同频——
        // GUI 顶栏不需要 100 Hz 刷新，省下来的是 GUI 的重绘开销
        if (++status_div >= (hz / 10 > 0 ? hz / 10 : 1)) {
            status_div = 0;
            sendJson(statusJson());
            if (logger_->status().active) sendJson(recordingJson());
        }
    }
}

}  // namespace ecjc
