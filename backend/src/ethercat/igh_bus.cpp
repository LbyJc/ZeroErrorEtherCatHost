// igh_bus.cpp —— 真实 IgH EtherCAT Master 后端。
//
// ⚠⚠ 本文件里最重要的一条规则（血泪教训，别删这段注释）：
//
//   ecrt_master_sdo_upload() / _download() 是**阻塞**调用。
//   如果在主站已 activate（OP 模式）且 RT 循环已停止的窗口里调用它，
//   此时没有任何线程再 ecrt_master_send/receive 泵总线，
//   调用线程会永远等不到应答 → 进入 D 状态（不可中断睡眠）→
//   kill -9 无效 → /dev/EtherCAT0 的 fd 永不释放 → 主站永远 Active:yes →
//   rmmod 永远 "in use" → **只能重启机器**（rmmod -f 会 panic）。
//
//   所以：阻塞式 SDO 只能在 activate 之前、或 deactivate 之后调用。
//   本文件用 phase_ 变量做运行期守卫，Active 相位直接拒绝，绝不放行。
//   OP 期间要读 SDO 一律用 ecrt_slave_config_create_sdo_request() 的异步方式。

#include "ecjc/ethercat_bus.hpp"

#include <ecrt.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <map>
#include <string>
#include <vector>

namespace ecjc {
namespace {

constexpr int64_t kNsPerSec = 1000000000LL;

int64_t timespecToNs(const struct timespec& t) {
    return static_cast<int64_t>(t.tv_sec) * kNsPerSec + t.tv_nsec;
}

EcState alToState(uint8_t al) {
    switch (al) {
        case 1: return EcState::Init;
        case 2: return EcState::PreOp;
        case 3: return EcState::Boot;
        case 4: return EcState::SafeOp;
        case 8: return EcState::Op;
        default: return EcState::Unknown;
    }
}

class IghBus : public IEtherCATBus {
public:
    ~IghBus() override { release(); }

    bool configure(const FullConfig& cfg, const StepReporter& rep,
                   std::string* err) override {
        cfg_ = cfg;
        // 防御性：即使上一次没走 release()，也不允许把上一会话的指针带进来
        clearSessionState();
        phase_ = BusPhase::PreActivate;

        // ── 1. 请求主站 ──────────────────────────────────────────────
        master_ = ecrt_request_master(cfg.ethercat.master_index);
        if (!master_) {
            *err = "无法请求 EtherCAT 主站 " + std::to_string(cfg.ethercat.master_index) +
                   "。请确认 IgH 内核模块已加载（/dev/EtherCAT0 是否存在），"
                   "并确认本进程有 root 权限。";
            rep("IgH Master Ready", false, *err);
            return false;
        }
        rep("IgH Master Ready", true, "");

        // ── 2. 从站配置 ──────────────────────────────────────────────
        sc_ = ecrt_master_slave_config(master_, cfg.slave.alias, cfg.slave.position,
                                       cfg.slave.vendor_id, cfg.slave.product_code);
        if (!sc_) {
            *err = "获取从站配置失败：位置 " + std::to_string(cfg.slave.position) +
                   " 上没有 VendorID=0x" + toHex(cfg.slave.vendor_id) +
                   " ProductCode=0x" + toHex(cfg.slave.product_code) +
                   " 的从站。请用 `ethercat slaves -v` 核对 slave.yaml。";
            rep("EtherCAT Slave Detected", false, *err);
            return false;
        }
        {
            ec_master_info_t mi{};
            if (!ecrt_master(master_, &mi) && mi.slave_count == 0) {
                *err = "主站已就绪但扫描到 0 个从站。请检查网线、从站供电、"
                       "以及 ethercat.yaml 里的网卡是否为实际连接的口。";
                rep("EtherCAT Slave Detected", false, *err);
                return false;
            }
        }
        rep("EtherCAT Slave Detected", true, cfg.slave.name);

        // ── 3. PDO 映射 ──────────────────────────────────────────────
        if (!buildPdos(err)) {
            rep("PDO Configured", false, *err);
            return false;
        }
        rep("PDO Configured", true, "");

        // off_ 此刻已经填好（buildPdos 里最后一步就是填它）。
        // Task 14 前 PDO 里没有这两个字段，标志为 false，异步 SDO 通道
        // 继续兼任主链路；Task 14 只改 pdo.yaml 加映射，这两个标志自动翻转，
        // 不用再碰这段代码——这就是"配置驱动的自动切换"。
        motor_position_in_pdo_ = off_.count("motor_position") > 0;
        twist_in_pdo_ = off_.count("twist_counts") > 0;

        // ── 4. 分布式时钟 ────────────────────────────────────────────
        if (cfg.ethercat.dc_enabled) {
            const uint32_t cycle_ns = cfg.ethercat.cycle_us * 1000u;
            ecrt_slave_config_dc(sc_, static_cast<uint16_t>(cfg.ethercat.dc_assign_activate),
                                 cycle_ns, cfg.ethercat.dc_sync0_shift_ns, 0, 0);
            rep("Distributed Clock Configured", true,
                "AssignActivate=0x" + toHex(cfg.ethercat.dc_assign_activate) +
                ", Sync0=" + std::to_string(cycle_ns) + "ns");
        } else {
            rep("Distributed Clock Configured", true, "已按配置关闭 DC（SM 同步）");
        }

        // ── 5. 启动参数下发 ──────────────────────────────────────────
        // 此刻相位是 PreActivate，阻塞式 SDO 是合法的（activate 之后就不行了）。
        // 把关键阈值写进驱动器，让配置文件成为唯一事实来源，
        // 而不是依赖"这台驱动器碰巧被谁手工设过"。
        for (const auto& s : cfg.startup_sdos) {
            uint8_t buf[8] = {0};
            size_t sz = 4;
            const int64_t v = s.value;
            if (s.type == "u8" || s.type == "i8")        { sz = 1; buf[0] = uint8_t(v); }
            else if (s.type == "u16" || s.type == "i16") { sz = 2; uint16_t t = uint16_t(v); memcpy(buf, &t, 2); }
            else                                          { sz = 4; uint32_t t = uint32_t(v); memcpy(buf, &t, 4); }

            std::string werr;
            if (!blockingSdoWrite(s.index, s.sub, buf, sz, &werr)) {
                *err = "启动参数下发失败 0x" + toHex(s.index) + " (" + s.name + "): " + werr;
                rep("Startup Parameters", false, *err);
                return false;
            }
            rep("Startup Parameters", true,
                s.name + " 0x" + toHex(s.index) + " = " + std::to_string(v));
        }

        // ── 6. 异步 SDO 请求（0x2240 电机侧位置只能这么读）────────────
        for (const auto& a : cfg.async_sdos) {
            // 未识别的 type 字符串回落到 4 字节（i32/s32）：config.cpp 里
            // async_sdo 的默认值本身就是 "i32"，这条回落只会在配置写错
            // type 时触发；读取端（pollAsyncSdo）用同一张表回落到
            // EC_READ_S32，size 与读宏两边保持一致，不会越界读。
            const size_t sz = (a.type == "u8"  || a.type == "i8")  ? 1
                             : (a.type == "u16" || a.type == "i16") ? 2 : 4;
            ec_sdo_request_t* r = ecrt_slave_config_create_sdo_request(
                sc_, a.index, a.sub, sz);
            if (!r) {
                *err = "创建异步 SDO 请求失败: 0x" + toHex(a.index);
                return false;
            }
            ecrt_sdo_request_timeout(r, 500);
            async_.push_back({r, a, 0});
        }

        return true;
    }

    bool activate(const StepReporter& rep, std::string* err) override {
        if (!master_ || !domain_) { *err = "内部错误：主站或 domain 未初始化"; return false; }

        // ── 诊断 SDO 一次性读取（Task 13）────────────────────────────
        // 此刻相位仍是 PreActivate（activate() 成功之前 phase_ 还没翻到
        // Active），guardBlocking() 会放行。这是全程唯一一处能安全阻塞读
        // 0x6093/0x6075 这类"只在启动前读一次、OP 期间绝不能碰"的对象的
        // 窗口——OP 期间调用阻塞式 SDO 会让进程进入 D 状态、fd 永不释放，
        // 只能重启机器（见文件头的血泪教训）。
        diagnostics_.clear();
        for (const auto& d : cfg_.slave.diagnostic_sdos) {
            int64_t v = 0;
            std::string e;
            if (blockingSdoReadTyped(d.index, d.sub, d.type, &v, &e)) {
                diagnostics_[d.name] = v;
            } else {
                fprintf(stderr, "[WARN] 诊断 SDO 0x%04X:%02X (%s) 读取失败: %s\n",
                        d.index, d.sub, d.name.c_str(), e.c_str());
                diagnostics_[d.name] = INT64_MIN;   // 哨兵值，metadata 里写 "read_failed"
            }
        }
        // 位置因子非 1:1 是静默错误的最大来源，单独校验一次：
        // CiA402 里 0x6064 是用户单位、由 0x6093 换算，工程若把它当原始
        // counts 用，偏差会在全部角度换算里静默传播。这里还在非 RT 的
        // 启动上下文，用普通 log 合法（RT 循环尚未开始，没有 snap_.last_error
        // 那一套确定性延迟约束）。
        {
            auto num = diagnostics_.find("position_factor_numerator");
            auto div = diagnostics_.find("position_factor_divisor");
            if (num != diagnostics_.end() && div != diagnostics_.end() &&
                num->second != INT64_MIN && div->second != INT64_MIN &&
                num->second != div->second) {
                fprintf(stderr,
                        "[WARN] 0x6093 位置因子不是 1:1（%lld/%lld）——"
                        "全部角度换算需按该因子修正！\n",
                        (long long)num->second, (long long)div->second);
            }
        }

        if (ecrt_master_activate(master_)) {
            *err = "ecrt_master_activate() 失败。常见原因：PDO 映射与 ESI 不符、"
                   "或另一个进程正占用主站（检查 `ethercat master` 的 Active 字段）。";
            rep("Master Activated", false, *err);
            return false;
        }
        phase_ = BusPhase::Active;

        domain_pd_ = ecrt_domain_data(domain_);
        if (!domain_pd_) {
            *err = "ecrt_domain_data() 返回空指针，domain 未正确建立";
            rep("Master Activated", false, *err);
            return false;
        }
        rep("Master Activated", true, "");

        // 等 SAFEOP → OP。必须持续泵总线，否则从站永远上不去。
        const unsigned timeout_ms = cfg_.ethercat.op_timeout_ms;
        const unsigned cycle_us = cfg_.ethercat.cycle_us;
        const uint64_t max_cycles = (uint64_t)timeout_ms * 1000ull / cycle_us;
        bool safeop_reported = false;
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);

        for (uint64_t i = 0; i < max_cycles; ++i) {
            t.tv_nsec += cycle_us * 1000;
            while (t.tv_nsec >= kNsPerSec) { t.tv_nsec -= kNsPerSec; t.tv_sec++; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, nullptr);

            receive();
            // 进 OP 之前保持控制字为 0，避免从站一进 OP 就带着上一次的残留命令动作
            if (domain_pd_) writeOutputs(RawIo{});
            send(timespecToNs(t));

            const BusStatus s = status();
            if (!safeop_reported && s.slave_state == EcState::SafeOp) {
                rep("SAFEOP", true, "");
                safeop_reported = true;
            }
            if (s.slave_state == EcState::Op && s.slave_operational) {
                if (!safeop_reported) rep("SAFEOP", true, "");
                rep("OP", true, "");
                return true;
            }
        }

        const BusStatus s = status();
        *err = std::string("从站未能在 ") + std::to_string(timeout_ms) +
               " ms 内进入 OP，当前停在 " + toString(s.slave_state) +
               "。请检查 PDO 映射是否与 ESI 一致、Distributed Clock 是否收敛"
               "（dmesg 里若有 \"Slave did not sync\" 可尝试在 ethercat.yaml 里把 "
               "dc_enabled 设为 false）、以及 Working Counter 是否完整。";
        rep(safeop_reported ? "OP" : "SAFEOP", false, *err);
        return false;
    }

    void deactivate() override {
        if (master_ && phase_ == BusPhase::Active) {
            ecrt_master_deactivate(master_);
            phase_ = BusPhase::PostDeactivate;
            // 注意：这之后驱动器会记录一条 0x603F=0xA000（通讯中断），
            // 不置 FAULT 位、不锁存，属正常现象。
        }
    }

    void release() override {
        if (master_) {
            if (phase_ == BusPhase::Active) deactivate();
            ecrt_release_master(master_);
            master_ = nullptr;
            domain_ = nullptr;
            domain_pd_ = nullptr;
            sc_ = nullptr;
            phase_ = BusPhase::Idle;
        }
        // ⚠ ecrt_release_master() 会连同释放该主站下的所有 SDO 请求对象。
        //   async_ 里存的是这些对象的裸指针，不清掉就全是悬空指针；
        //   下次 configure() 又往同一个 vector 追加新请求，RT 循环便会同时
        //   遍历悬空和有效两批 → 刷满 "Bad file descriptor" → 段错误。
        //   现场表现：点一次【重新连接】，后端 SIGSEGV 崩溃、systemd 重启它，
        //   运行目录被重建、ACL 丢失，于是 GUI 再也连不上。
        clearSessionState();
    }

    /// 与某一次主站会话绑定的所有缓存，释放主站时必须一并作废
    void clearSessionState() {
        async_.clear();
        off_.clear();
        names_.clear();
        entries_rx_.clear();
        entries_tx_.clear();
        pdos_rx_.clear();
        pdos_tx_.clear();
        std::memset(offsets_, 0, sizeof offsets_);
        motor_position_in_pdo_ = false;
        twist_in_pdo_ = false;
        diagnostics_.clear();
    }

    void receive() override {
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
    }

    void send(int64_t app_time_ns) override {
        ecrt_domain_queue(domain_);
        if (cfg_.ethercat.dc_enabled) {
            ecrt_master_application_time(master_, static_cast<uint64_t>(app_time_ns));
            ecrt_master_sync_reference_clock(master_);
            ecrt_master_sync_slave_clocks(master_);
        }
        ecrt_master_send(master_);
    }

    void readInputs(RawIo* io) override {
        if (!domain_pd_) return;
        auto rd16 = [&](const char* n) -> uint16_t {
            auto it = off_.find(n); return it == off_.end() ? 0 : EC_READ_U16(domain_pd_ + it->second); };
        auto rd32 = [&](const char* n) -> int32_t {
            auto it = off_.find(n); return it == off_.end() ? 0 : EC_READ_S32(domain_pd_ + it->second); };
        auto rd16s = [&](const char* n) -> int16_t {
            auto it = off_.find(n); return it == off_.end() ? 0 : EC_READ_S16(domain_pd_ + it->second); };
        auto rd8 = [&](const char* n) -> int8_t {
            auto it = off_.find(n); return it == off_.end() ? 0 : EC_READ_S8(domain_pd_ + it->second); };
        auto rdu32 = [&](const char* n) -> uint32_t {
            auto it = off_.find(n); return it == off_.end() ? 0 : EC_READ_U32(domain_pd_ + it->second); };

        io->error_code      = rd16("error_code");
        io->statusword      = rd16("statusword");
        io->position_actual = rd32("position_actual");
        io->velocity_actual = rd32("velocity_actual");
        io->torque_actual   = rd16s("torque_actual");
        io->modes_display   = rd8("modes_display");
        io->output_position = rd32("output_position");
        io->position_counts = rd32("position_counts");
        io->current_actual  = rd16s("current_actual");
        io->warning_code    = rdu32("warning_code");
        io->vendor_torque   = rd32("vendor_torque");      // 0x3B69，此前已映射但从未读
        io->torque_ratio    = rd16s("torque_ratio");      // 0x3B6A，同上
        io->dc_link_mV      = rdu32("dc_link_voltage");   // 0x6079，新增映射
        io->following_error = rd32("following_error");    // 0x60F4，新增映射

        // Task 15 放开 0x1A00 注释后由标志自动接管；标志为 false 时绝不能无条件读——
        // rd32 对缺失名返回 0，会每拍把异步 SDO 填的值清零（这正是门控存在的原因）
        if (motor_position_in_pdo_) io->motor_position = rd32("motor_position");
        if (twist_in_pdo_)          io->twist_counts   = rd32("twist_counts");
    }

    void writeOutputs(const RawIo& io) override {
        if (!domain_pd_) return;
        auto w = [&](const char* n, auto fn) {
            auto it = off_.find(n); if (it != off_.end()) fn(domain_pd_ + it->second); };
        w("controlword",       [&](uint8_t* p){ EC_WRITE_U16(p, io.controlword); });
        w("modes_of_operation",[&](uint8_t* p){ EC_WRITE_S8 (p, io.modes_of_operation); });
        w("target_position",   [&](uint8_t* p){ EC_WRITE_S32(p, io.target_position); });
        w("target_velocity",   [&](uint8_t* p){ EC_WRITE_S32(p, io.target_velocity); });
        w("target_torque",     [&](uint8_t* p){ EC_WRITE_S16(p, io.target_torque); });
        w("max_torque",        [&](uint8_t* p){ EC_WRITE_U16(p, io.max_torque); });
    }

    void pollAsyncSdo(uint64_t cycle, RawIo* io) override {
        // 非阻塞状态机：Success 取值并重新发起，Busy 就等下一拍。
        for (auto& a : async_) {
            if (cycle % static_cast<uint64_t>(a.cfg.poll_divisor) != 0) continue;
            switch (ecrt_sdo_request_state(a.req)) {
                case EC_REQUEST_SUCCESS: {
                    const uint8_t* d = ecrt_sdo_request_data(a.req);
                    int64_t v = 0;
                    // 与 configure() 里建请求的尺寸表同源：未识别的 type
                    // 字符串回落到 EC_READ_S32，和建请求时的 4 字节回落一致。
                    if      (a.cfg.type == "u8")  v = EC_READ_U8(d);
                    else if (a.cfg.type == "i8")  v = EC_READ_S8(d);
                    else if (a.cfg.type == "u16") v = EC_READ_U16(d);
                    else if (a.cfg.type == "i16") v = EC_READ_S16(d);
                    else if (a.cfg.type == "u32") v = EC_READ_U32(d);
                    else                          v = EC_READ_S32(d);

                    // 表驱动分派。名字来自 pdo.yaml，加对象只改配置 + 这张表。
                    if (a.cfg.name == "motor_position") {
                        io->motor_position_sdo = static_cast<int32_t>(v);  // 独立通道，
                                                                            // J3 对拍专用
                        // Task 14 完成前 PDO 里还没有 0x2240，主链路仍依赖 SDO 值；
                        // motor_position_in_pdo_ 一旦（Task 14 后）因 pdo.yaml 变化
                        // 而置位，这行自动不再执行，不需要再改这段代码。
                        if (!motor_position_in_pdo_)
                            io->motor_position = static_cast<int32_t>(v);
                    } else if (a.cfg.name == "dual_encoder_diff") {
                        // 同样的模式：Task 14 后若 0x2241 也进了 PDO，
                        // readInputs 会填 twist_counts，这里让开，避免两个
                        // 写入者互相覆盖。
                        if (!twist_in_pdo_)
                            io->twist_counts = static_cast<int32_t>(v);
                    } else if (a.cfg.name == "drive_temperature") {
                        io->drive_temp_C = static_cast<uint16_t>(v);
                    }
                    ecrt_sdo_request_read(a.req);
                    break;
                }
                case EC_REQUEST_UNUSED:
                    ecrt_sdo_request_read(a.req);
                    break;
                case EC_REQUEST_ERROR:
                    // 只计数，不在这里打印：RT 线程跑 SCHED_FIFO + clock_nanosleep
                    // 定拍，stdio 调用（fprintf/snprintf 到 stderr）没有确定的
                    // 上界延迟，这条路径恰又与总线不稳场景相关——最坏时刻不能
                    // 加不确定延迟。async_sdo_errors 已经汇总进 BusStatus/statusJson，
                    // 非 RT 侧（IPC/GUI）按需在那读取、要打日志也在那打。
                    ++a.errors;
                    ecrt_sdo_request_read(a.req);
                    break;
                default:   // EC_REQUEST_BUSY：本拍不动，绝不等待
                    break;
            }
        }
    }

    BusStatus status() override {
        BusStatus s{};
        if (!master_) return s;

        ec_master_state_t ms{};
        ecrt_master_state(master_, &ms);
        s.slave_count = ms.slaves_responding;
        s.link_up = ms.link_up;
        s.master_state = alToState(static_cast<uint8_t>(ms.al_states));

        if (sc_) {
            ec_slave_config_state_t ss{};
            ecrt_slave_config_state(sc_, &ss);
            s.slave_online = ss.online;
            s.slave_operational = ss.operational;
            s.slave_state = alToState(static_cast<uint8_t>(ss.al_state));
        }
        if (domain_) {
            ec_domain_state_t ds{};
            ecrt_domain_state(domain_, &ds);
            s.working_counter = ds.working_counter;
            s.wc_state = static_cast<int>(ds.wc_state);
        }
        s.dc_ok = cfg_.ethercat.dc_enabled;
        for (const auto& a : async_) s.async_sdo_errors += a.errors;
        return s;
    }

    BusPhase phase() const override { return phase_; }
    bool isMock() const override { return false; }

    bool blockingSdoRead(uint16_t index, uint8_t sub, void* buf, size_t size,
                         size_t* result_size, std::string* err) override {
        if (!guardBlocking(err)) return false;
        uint32_t abort = 0;
        if (ecrt_master_sdo_upload(master_, cfg_.slave.position, index, sub,
                                   static_cast<uint8_t*>(buf), size, result_size, &abort)) {
            *err = "SDO 读 0x" + toHex(index) + ":" + std::to_string(sub) +
                   " 失败 (abort 0x" + toHex(abort) + ")";
            return false;
        }
        return true;
    }

    bool blockingSdoWrite(uint16_t index, uint8_t sub, const void* buf, size_t size,
                          std::string* err) override {
        if (!guardBlocking(err)) return false;
        uint32_t abort = 0;
        if (ecrt_master_sdo_download(master_, cfg_.slave.position, index, sub,
                                     static_cast<uint8_t*>(const_cast<void*>(buf)),
                                     size, &abort)) {
            *err = "SDO 写 0x" + toHex(index) + ":" + std::to_string(sub) +
                   " 失败 (abort 0x" + toHex(abort) + ")";
            return false;
        }
        return true;
    }

    const std::map<std::string, int64_t>& diagnostics() const override { return diagnostics_; }

private:
    /// blockingSdoRead() 的类型化包装：按 DiagnosticSdoCfg::type 把读到的字节
    /// 解码成 int64_t。与 pollAsyncSdo() 的类型分派表同源同选择——未识别的
    /// type 字符串回落到有符号 32 位（s32），不是因为它"正确"，而是与
    /// pollAsyncSdo 保持一致，不引入第二套回落规则。
    bool blockingSdoReadTyped(uint16_t index, uint8_t sub, const std::string& type,
                              int64_t* out, std::string* err) {
        const size_t sz = (type == "u8"  || type == "i8")  ? 1
                         : (type == "u16" || type == "i16") ? 2 : 4;
        uint8_t buf[8] = {0};
        size_t rs = 0;
        if (!blockingSdoRead(index, sub, buf, sz, &rs, err)) return false;

        // 终审 finding I4②：从站实际返回的宽度与 config.yaml 里配置的 type
        // 不符时，此前会直接按配置宽度去解码 buf——多读的字节被静默忽略，
        // 少读的字节则读到 buf 里未写入的陈旧/清零区域，两种情况都是错误
        // 数值而不报错。这里显式比对拒绝，走既有 INT64_MIN -> "read_failed"
        // 的调用方路径。
        if (rs != sz) {
            char b[160];
            snprintf(b, sizeof b,
                     "0x%04X:%02X 实际宽度 %zu 与配置 type(%s, %zu 字节) 不符",
                     index, sub, rs, type.c_str(), sz);
            *err = b;
            return false;
        }

        if      (type == "u8")  *out = EC_READ_U8(buf);
        else if (type == "i8")  *out = EC_READ_S8(buf);
        else if (type == "u16") *out = EC_READ_U16(buf);
        else if (type == "i16") *out = EC_READ_S16(buf);
        else if (type == "u32") *out = EC_READ_U32(buf);
        else                    *out = EC_READ_S32(buf);   // i32 及未知类型回落
        return true;
    }

    struct AsyncReq { ec_sdo_request_t* req; AsyncSdoCfg cfg; uint64_t errors; };

    static std::string toHex(uint32_t v) {
        char b[16]; snprintf(b, sizeof b, "%x", v); return b;
    }

    /// 见文件顶部的血泪教训
    bool guardBlocking(std::string* err) {
        if (!master_) { *err = "主站未初始化"; return false; }
        if (phase_ == BusPhase::Active) {
            *err = "拒绝在主站 Active 相位执行阻塞式 SDO —— 这会让进程进入 D 状态"
                   "且只能靠重启机器恢复。请在启动前或停止后读取，"
                   "或把该对象加入 pdo.yaml 的 async_sdo 列表。";
            return false;
        }
        return true;
    }

    bool buildPdos(std::string* err) {
        // 按 pdo.yaml 组装 ec_sync_info_t。规矩：厂商 PDO 多为 Fixed="1"，
        // 我们只是**声明**与 ESI 完全一致的条目表让 IgH 比对，
        // 一致时 IgH 不会去写映射寄存器，只做 0x1C12/0x1C13 分配。
        entries_rx_.clear(); entries_tx_.clear();
        pdos_rx_.clear(); pdos_tx_.clear();

        auto build = [](const std::vector<PdoCfg>& src,
                        std::vector<std::vector<ec_pdo_entry_info_t>>& ebuf,
                        std::vector<ec_pdo_info_t>& pbuf) {
            ebuf.reserve(src.size());
            for (const auto& p : src) {
                std::vector<ec_pdo_entry_info_t> es;
                es.reserve(p.entries.size());
                for (const auto& e : p.entries)
                    es.push_back({e.index, e.sub, e.bits});
                ebuf.push_back(std::move(es));
            }
            for (size_t i = 0; i < src.size(); ++i)
                pbuf.push_back({src[i].index,
                                static_cast<unsigned>(ebuf[i].size()),
                                ebuf[i].data()});
        };
        build(cfg_.rx_pdos, entries_rx_, pdos_rx_);
        build(cfg_.tx_pdos, entries_tx_, pdos_tx_);

        ec_sync_info_t syncs[] = {
            {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
            {1, EC_DIR_INPUT,  0, nullptr, EC_WD_DISABLE},
            {2, EC_DIR_OUTPUT, static_cast<unsigned>(pdos_rx_.size()), pdos_rx_.data(), EC_WD_ENABLE},
            {3, EC_DIR_INPUT,  static_cast<unsigned>(pdos_tx_.size()), pdos_tx_.data(), EC_WD_DISABLE},
            {0xFF, EC_DIR_INVALID, 0, nullptr, EC_WD_DEFAULT}
        };
        if (ecrt_slave_config_pdos(sc_, EC_END, syncs)) {
            *err = "ecrt_slave_config_pdos() 失败。请核对 pdo.yaml 里的条目顺序与位宽"
                   "是否与 ESI 文件完全一致（厂商 PDO 是 Fixed=\"1\"，条目不允许改写）。";
            return false;
        }

        domain_ = ecrt_master_create_domain(master_);
        if (!domain_) { *err = "ecrt_master_create_domain() 失败"; return false; }

        // 注册 domain entry，收集每个字段的字节偏移。
        // ⚠ ecrt_domain_reg_pdo_entry_list() 保存的是偏移量变量的**地址**，
        //   所以存放偏移的容器在注册期间绝不能重分配。这里用定长数组，
        //   容量不够时**在越界之前**就报错返回，而不是先写坏再检查。
        std::vector<ec_pdo_entry_reg_t> regs;
        names_.clear();
        size_t n = 0;
        bool overflow = false;

        auto reg = [&](const std::vector<PdoCfg>& list) {
            for (const auto& p : list)
                for (const auto& e : p.entries) {
                    if (e.type == "pad" || e.index == 0) continue;
                    if (n >= kMaxPdoEntries) { overflow = true; return; }
                    offsets_[n] = 0;
                    names_.push_back(e.name);
                    regs.push_back({cfg_.slave.alias, cfg_.slave.position,
                                    cfg_.slave.vendor_id, cfg_.slave.product_code,
                                    e.index, e.sub,
                                    &offsets_[n], nullptr});
                    ++n;
                }
        };
        reg(cfg_.rx_pdos);
        reg(cfg_.tx_pdos);
        if (overflow) {
            *err = "PDO 条目超过 " + std::to_string(kMaxPdoEntries) +
                   " 个，请调大 igh_bus.cpp 里的 kMaxPdoEntries";
            return false;
        }
        regs.push_back({});

        if (ecrt_domain_reg_pdo_entry_list(domain_, regs.data())) {
            *err = "ecrt_domain_reg_pdo_entry_list() 失败：某个 PDO 条目在从站上找不到。"
                   "请逐条核对 pdo.yaml 的 index/subindex。";
            return false;
        }
        for (size_t i = 0; i < names_.size(); ++i) off_[names_[i]] = offsets_[i];
        return true;
    }

    FullConfig cfg_;
    ec_master_t* master_ = nullptr;
    ec_domain_t* domain_ = nullptr;
    ec_slave_config_t* sc_ = nullptr;
    uint8_t* domain_pd_ = nullptr;
    BusPhase phase_ = BusPhase::Idle;

    static constexpr size_t kMaxPdoEntries = 64;

    std::vector<std::vector<ec_pdo_entry_info_t>> entries_rx_, entries_tx_;
    std::vector<ec_pdo_info_t> pdos_rx_, pdos_tx_;
    unsigned int offsets_[kMaxPdoEntries] = {};
    std::vector<std::string> names_;
    std::map<std::string, unsigned int> off_;
    std::vector<AsyncReq> async_;

    /// activate() 前一次性阻塞读到的诊断 SDO（Task 13）。会话绑定，
    /// clearSessionState() 里一并清掉，避免重连后残留上一次会话的旧值。
    std::map<std::string, int64_t> diagnostics_;

    // Task 14 之前恒为 false：PDO 里还没有这两个字段，异步 SDO 通道
    // 兼任主链路。构造时机见 configure()：buildPdos() 填完 off_ 之后立即置位。
    bool motor_position_in_pdo_ = false;
    bool twist_in_pdo_ = false;
};

}  // namespace

std::unique_ptr<IEtherCATBus> makeIghBus() { return std::make_unique<IghBus>(); }

}  // namespace ecjc
