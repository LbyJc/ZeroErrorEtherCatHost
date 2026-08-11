// main.cpp —— ecjc-backend 入口。
//
// 职责：装配各模块、把 GUI 的【启动主站】/【停止主站】/【重新连接】
// 翻译成对总线的真实操作，并保证停机顺序安全（任务书第九节）。
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>

#include "ecjc/config.hpp"
#include "ecjc/data_logger.hpp"
#include "ecjc/ethercat_bus.hpp"
#include "ecjc/ipc_server.hpp"
#include "ecjc/realtime_task.hpp"

using namespace ecjc;

namespace {
volatile std::sig_atomic_t g_quit = 0;
void onSignal(int) { g_quit = 1; }

/// 单实例保护。
///
/// 起因：IpcServer 启动时无条件 unlink 掉 socket 再 bind，
/// 于是第二个实例会**悄悄抢走** socket，而第一个实例还活着、还可能占着
/// EtherCAT 主站。现场一度同时跑了 3 个后端，排查起来很费劲。
/// flock 是最合适的手段：进程无论怎么死（包括 kill -9），锁都会被内核释放。
int acquireInstanceLock(const std::string& sock_path) {
    const std::string lock_path = sock_path + ".lock";
    std::error_code ec;
    std::filesystem::create_directories(
        std::filesystem::path(lock_path).parent_path(), ec);

    const int fd = ::open(lock_path.c_str(), O_CREAT | O_RDWR, 0644);
    if (fd < 0) {
        fprintf(stderr, "无法创建锁文件 %s: %s\n", lock_path.c_str(), strerror(errno));
        return -1;
    }
    if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr,
                "已有另一个 ecjc-backend 实例在运行（锁文件 %s）。\n"
                "请先停止它：  pkill -x ecjc-backend\n", lock_path.c_str());
        ::close(fd);
        return -1;
    }
    char buf[32];
    const int n = snprintf(buf, sizeof buf, "%d\n", ::getpid());
    if (::ftruncate(fd, 0) != 0) { /* 非致命 */ }
    if (::write(fd, buf, static_cast<size_t>(n)) != n) { /* 非致命 */ }
    return fd;   // 故意不关：进程存活期间持锁
}

/// 关闭兜底：正常停机流程若卡住，10 秒后强制退出。
/// 现场遇到过 SIGTERM 之后进程释放了主站却一直不退，留下孤儿进程。
/// 宁可粗暴退出，也不要留一个既不干活又占着名字的僵尸。
void armShutdownWatchdog() {
    std::thread([] {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        fprintf(stderr, "[WARN] 正常停机超过 10 秒未完成，强制退出\n");
        ::_exit(0);
    }).detach();
}

void usage(const char* argv0) {
    printf(
        "用法: %s [选项]\n"
        "  --config <目录>   配置目录（默认 ./config）\n"
        "  --mock            无硬件仿真模式，不接触真实 EtherCAT 主站\n"
        "  --socket <路径>   覆盖 IPC socket 路径\n"
        "  --help\n", argv0);
}
}  // namespace

int main(int argc, char** argv) {
    std::string config_dir = "config";
    std::string socket_override;
    bool mock = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--mock" || a == "--simulation") mock = true;
        else if (a == "--config" && i + 1 < argc) config_dir = argv[++i];
        else if (a == "--socket" && i + 1 < argc) socket_override = argv[++i];
        else if (a == "--help" || a == "-h") { usage(argv[0]); return 0; }
        else { fprintf(stderr, "未知参数: %s\n", a.c_str()); usage(argv[0]); return 2; }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);   // GUI 突然消失时不要杀掉后端

    // ── 配置 ──────────────────────────────────────────────────────────
    FullConfig cfg;
    std::string err;
    if (!loadConfig(config_dir, &cfg, &err)) {
        fprintf(stderr, "配置加载失败: %s\n", err.c_str());
        return 1;
    }
    printf("[INFO] 配置已加载: %s\n", config_dir.c_str());
    printf("[INFO] 模式: %s | 周期: %u us | 网卡: %s\n",
           mock ? "MOCK (无硬件)" : "真实 EtherCAT",
           cfg.ethercat.cycle_us, cfg.ethercat.interface.c_str());
    if (!cfg.scaling.resolution_verified)
        printf("[WARN] 输出侧编码器分辨率 %.0f counts/rev 未经物理转角验证，"
               "若实为其一半则所有 rpm 数值翻倍\n", cfg.scaling.output_counts_per_rev);

    // ── 装配 ──────────────────────────────────────────────────────────
    auto bus = mock ? makeMockBus() : makeIghBus();
    RealtimeTask rt(cfg, bus.get());
    DataLogger logger(cfg, &rt.logRing());
    IpcServer ipc(cfg, &rt, &logger);

    // 启动主站的完整流程（任务书第七、八节）。每一步都回报 GUI。
    auto connect = [&](std::string* e) -> bool {
        if (bus->phase() == BusPhase::Active) { *e = "主站已在运行"; return false; }
        auto rep = [&](const std::string& s, bool ok, const std::string& m) {
            ipc.reportStep(s, ok, m);
        };
        ipc.reportStep("Configuration Loaded", true, config_dir);

        // 网卡检查：报的是"哪个网卡不存在"，不是 error -1
        if (!mock) {
            const std::string sysnet = "/sys/class/net/" + cfg.ethercat.interface;
            if (::access(sysnet.c_str(), F_OK) != 0) {
                *e = "配置的网卡 " + cfg.ethercat.interface +
                     " 不存在，请检查 config/ethercat.yaml 的 interface 字段";
                ipc.reportStep("Network Interface Found", false, *e);
                return false;
            }
        }
        ipc.reportStep("Network Interface Found", true, cfg.ethercat.interface);

        if (!bus->configure(cfg, rep, e)) return false;

        // ⚠ 顺序要紧：先把 RT 线程拉起来（它要做 mlockall、8MB 栈预触、
        //   设 SCHED_FIFO，这些都要时间），此时它被 bus_active_ 挡住不碰总线。
        //   等 activate 完成后翻标志，RT 循环下一拍就接管。
        //   反过来先 activate 再 start，中间那几十毫秒没人泵总线，
        //   SM 看门狗超时会把刚进 OP 的从站打回 SAFEOP。
        if (!rt.start(e)) return false;
        ipc.reportStep("Backend Running", true, "");

        if (!bus->activate(rep, e)) return false;
        rt.setBusActive(true);

        ipc.reportStep("Realtime Task Running", true, "");
        ipc.log("INFO", "EtherCAT System Ready");
        return true;
    };

    // 停止主站的安全顺序（任务书第九节）：
    // 停轨迹 → 目标归零 → Disable Operation → 停 RT → deactivate → 释放
    auto disconnect = [&](std::string* e) -> bool {
        (void)e;
        rt.stopRun();
        // 等软停真正走完，而不是拍一个固定的 300ms。
        // 按 csv_decel_rpm_per_s=200，从 3000 电机 rpm 减到零需 15 s。
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (std::chrono::steady_clock::now() < deadline) {
            const auto st = rt.snapshot();
            if (isSafeToDisableAt(st.joint.output_vel_rpm, st.stopping)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        rt.servoDisable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 手册：制动器动作约 150ms
        rt.requestStop();
        rt.join();
        rt.setBusActive(false);      // RT 已退出，明确交回总线所有权
        bus->deactivate();
        bus->release();
        ipc.log("INFO", "主站已安全停止");
        return true;
    };

    auto reconnect = [&](std::string* e) -> bool {
        std::string ignored;
        disconnect(&ignored);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return connect(e);
    };

    // 0x730F（编码器电池欠压）的补救：向 0x2242 写 1。
    // 必须在非 Active 相位做——阻塞式 SDO 的铁律。
    auto reset_encoder = [&](std::string* e) -> bool {
        if (bus->phase() == BusPhase::Active) {
            *e = "请先【停止主站】再重置负载端编码器："
                 "该操作需要阻塞式 SDO，而主站处于 Active 相位时执行会导致进程死锁。";
            return false;
        }
        const uint16_t one = 1;
        if (!bus->blockingSdoWrite(0x2242, 0x00, &one, sizeof one, e)) return false;
        ipc.log("INFO", "已向 0x2242 写 1，负载端编码器多圈计数已重置，0x730F 应已清除");
        return true;
    };

    ipc.setBusActions(connect, disconnect, reconnect);
    ipc.setResetEncoderAction(reset_encoder);

    const std::string sock = !socket_override.empty() ? socket_override
                           : (::geteuid() == 0 ? cfg.app.socket_path
                                               : cfg.app.socket_path_dev);
    const int lock_fd = acquireInstanceLock(sock);
    if (lock_fd < 0) return 1;

    if (!ipc.start(sock, &err)) {
        fprintf(stderr, "IPC 启动失败: %s\n", err.c_str());
        return 1;
    }
    printf("[INFO] IPC 监听于 %s\n", sock.c_str());

    if (cfg.app.auto_start_master) {
        std::string e;
        if (!connect(&e)) fprintf(stderr, "[WARN] 自动启动主站失败: %s\n", e.c_str());
    } else {
        printf("[INFO] 等待 GUI 下达【启动主站】命令（app.yaml 里 auto_start_master=false）\n");
    }

    while (!g_quit) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    printf("\n[INFO] 收到退出信号，安全停机中...\n");
    fflush(stdout);
    armShutdownWatchdog();
    std::string e;
    if (bus->phase() == BusPhase::Active) disconnect(&e);
    logger.stop();
    ipc.stop();
    ::close(lock_fd);
    printf("[INFO] 已退出\n");
    return 0;
}
