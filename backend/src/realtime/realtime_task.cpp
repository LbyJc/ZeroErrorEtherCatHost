#include "ecjc/realtime_task.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>

namespace ecjc {
namespace {

constexpr int64_t kNsPerSec = 1000000000LL;

inline int64_t nowMonotonicNs() {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<int64_t>(t.tv_sec) * kNsPerSec + t.tv_nsec;
}
inline int64_t nowRealtimeNs() {
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return static_cast<int64_t>(t.tv_sec) * kNsPerSec + t.tv_nsec;
}
inline void addNs(struct timespec* t, int64_t ns) {
    t->tv_nsec += ns;
    while (t->tv_nsec >= kNsPerSec) { t->tv_nsec -= kNsPerSec; t->tv_sec++; }
}

}  // namespace

RealtimeTask::RealtimeTask(FullConfig cfg, IEtherCATBus* bus)
    : cfg_(std::move(cfg)), bus_(bus), scaling_(cfg_.scaling) {
    log_ring_.init(cfg_.realtime.log_ring_capacity);
    gui_ring_.init(cfg_.realtime.gui_ring_capacity);

    // 默认轨迹 + 默认控制器。构造发生在这里（非 RT 线程），RT 循环内不再分配。
    std::string err;
    TrajParams tp = cfg_.trajectory;
    tp.type = TrajType::Constant;
    tp.constant_value = 0.0;
    traj_ = makeTrajectory(tp, &err).release();
    ctrl_ = makeController(cfg_.controller.default_id).release();
    if (!ctrl_) ctrl_ = makeController("passthrough").release();
    ctrl_id_ = ctrl_->id();

    ParameterRegistry reg;
    ctrl_->declareParams(reg);
    params_.redefine(reg.metas());
    params_.refreshIfChanged(&param_view_);
}

RealtimeTask::~RealtimeTask() {
    requestStop();
    join();
    delete traj_;
    delete ctrl_;
    reapRetired();
    delete pending_traj_.exchange(nullptr);
    delete pending_ctrl_.exchange(nullptr);
}

// ── 启停 ──────────────────────────────────────────────────────────────────
bool RealtimeTask::start(std::string* err) {
    if (thread_running_.load()) { *err = "实时任务已在运行"; return false; }
    quit_.store(false);
    th_ = std::thread([this] { threadMain(); });
    // 等线程真正进入循环，最多 2 秒
    for (int i = 0; i < 200 && !thread_running_.load(); ++i)
        usleep(10000);
    if (!thread_running_.load()) {
        *err = "实时线程启动超时";
        return false;
    }
    return true;
}

void RealtimeTask::requestStop() { quit_.store(true, std::memory_order_release); }

void RealtimeTask::join() { if (th_.joinable()) th_.join(); }

void RealtimeTask::applyRtScheduling() {
    if (cfg_.realtime.lock_memory) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            // 拿不到就继续跑，但要让 GUI 看得见——不是静默降级
            snprintf(snap_.last_error, sizeof snap_.last_error,
                     "mlockall 失败（需要 root 或提高 memlock 限制），可能出现缺页抖动");
        }
    }
    // 栈预触：提前写一遍，避免运行中触发缺页
    {
        const size_t n = static_cast<size_t>(cfg_.realtime.stack_prefault_kb) * 1024;
        volatile char* p = static_cast<volatile char*>(alloca(0));
        (void)p;
        std::vector<char> prefault(n, 0);
        for (size_t i = 0; i < n; i += 4096) prefault[i] = 1;
    }

    struct sched_param sp{};
    sp.sched_priority = cfg_.realtime.sched_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        snprintf(snap_.last_error, sizeof snap_.last_error,
                 "无法设置 SCHED_FIFO 优先级 %d（需要 root / LimitRTPRIO），"
                 "当前以普通调度运行，抖动会明显变大",
                 cfg_.realtime.sched_priority);
    }

    if (!cfg_.realtime.cpu_affinity.empty()) {
        cpu_set_t set;
        CPU_ZERO(&set);
        const long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
        for (int c : cfg_.realtime.cpu_affinity)
            if (c >= 0 && c < ncpu) CPU_SET(c, &set);
        pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    }
}

// ── RT 主循环 ─────────────────────────────────────────────────────────────
void RealtimeTask::threadMain() {
    applyRtScheduling();

    const int64_t cycle_ns = static_cast<int64_t>(cfg_.ethercat.cycle_us) * 1000;
    struct timespec wakeup;
    clock_gettime(CLOCK_MONOTONIC, &wakeup);

    stats_ = RtStats{};
    stats_.jitter_min_ns = INT64_MAX;
    thread_running_.store(true, std::memory_order_release);

    while (!quit_.load(std::memory_order_acquire)) {
        addNs(&wakeup, cycle_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, nullptr);

        // 抖动 = 实际唤醒时刻 - 期望唤醒时刻。
        // 本机是 PREEMPT_DYNAMIC 内核，这个数会有几十微秒，属正常；
        // 我们如实统计而不是假装硬实时。
        const int64_t now = nowMonotonicNs();
        const int64_t expected =
            static_cast<int64_t>(wakeup.tv_sec) * kNsPerSec + wakeup.tv_nsec;
        const int64_t jit = now - expected;
        stats_.jitter_ns = jit;
        if (jit > stats_.jitter_max_ns) stats_.jitter_max_ns = jit;
        if (jit < stats_.jitter_min_ns) stats_.jitter_min_ns = jit;
        jitter_sum_ += static_cast<double>(jit);
        stats_.cycles++;
        stats_.jitter_mean_ns = jitter_sum_ / static_cast<double>(stats_.cycles);
        if (jit > cycle_ns) stats_.deadline_miss++;

        cycle(now);
    }

    // 退出前把输出清零并撤使能，不要让从站带着最后一条命令留在 OP。
    //
    // 终审 finding I3：这是 RT 侧**第二个**真的会把控制字拍成 DisableVoltage
    // 的地方（第一个是上面 cycle() 里那份门控）。requestStop() 可能在任意
    // 时刻被别的路径直接调用——RealtimeTask 析构函数就是一个，main.cpp 的
    // 等待循环超时后 break 也照样走到这，C2 场景下看门狗杀进程前的收尾同样
    // 可能撞上这里。原来这段无条件拍 50 拍 DisableVoltage、完全不看当前速度，
    // 等于绕开了 cycle() 里千辛万苦加上的软停门控——高速旋转时一样会把制动器
    // 烧了。这里必须用同一套判据（shouldHoldEnableForDisableGate /
    // isSafeToDisableAt / disableWaitTimedOut）、同一个 disable_wait_cycles_
    // 计数器（不重新清零、不另开一份预算，续用 cycle() 里已经攒的等待时间），
    // 分两段做：
    //   阶段一：门控等待——不安全就继续维持 EnableOperation，直到变安全或
    //           超时（与主循环共用 disable_timeout_cycles 这个预算）。
    //   阶段二：确认撤能——有界 50 拍，把控制字真正拍成 DisableVoltage。
    {
        while (bus_->phase() == BusPhase::Active) {
            bus_->receive();
            bus_->readInputs(&raw_);
            scaling_.toPhysical(raw_, &joint_);

            if (!shouldHoldEnableForDisableGate(cia_.state(), joint_.output_vel_rpm,
                                                 /*stopping=*/false)) {
                break;   // 已经安全，或者本来就没有力矩：直接进入阶段二
            }

            cia_.setTarget(Cia402Target::EnableOperation);
            ++disable_wait_cycles_;
            if (disableWaitTimedOut(disable_wait_cycles_, cfg_.stop_ramp.disable_timeout_cycles))
                break;   // 超时兜底：不再等，进入阶段二强制撤能

            const uint16_t cw = cia_.update(raw_.statusword);
            RawIo z = raw_;
            z.controlword = cw;
            // 不追新的运动指令，只维持使能——目标钉在当前实测值上
            z.target_position = raw_.position_actual;
            z.target_velocity = 0;
            z.target_torque = 0;
            bus_->writeOutputs(z);
            bus_->send(nowMonotonicNs());
            addNs(&wakeup, cycle_ns);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, nullptr);
        }

        cia_.setTarget(Cia402Target::DisableVoltage);
        for (int i = 0; i < 50 && bus_->phase() == BusPhase::Active; ++i) {
            bus_->receive();
            bus_->readInputs(&raw_);
            const uint16_t cw = cia_.update(raw_.statusword);
            RawIo z{};
            z.controlword = cw;
            z.modes_of_operation = raw_.modes_of_operation;
            bus_->writeOutputs(z);
            bus_->send(nowMonotonicNs());
            addNs(&wakeup, cycle_ns);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, nullptr);
        }
    }
    thread_running_.store(false, std::memory_order_release);
}

void RealtimeTask::swapPending() {
    if (TrajectoryBase* nt = pending_traj_.exchange(nullptr, std::memory_order_acq_rel)) {
        TrajectoryBase* old = traj_;
        traj_ = nt;
        traj_->start(joint_, static_cast<OpMode>(desired_mode_.load()));
        // 旧对象交还给 IPC 线程释放，RT 线程绝不 delete
        TrajectoryBase* prev = retired_traj_.exchange(old, std::memory_order_acq_rel);
        (void)prev;   // 上一个还没被回收的情况极罕见，reapRetired 会兜住
    }
    if (ControllerBase* nc = pending_ctrl_.exchange(nullptr, std::memory_order_acq_rel)) {
        ControllerBase* old = ctrl_;
        ctrl_ = nc;
        ctrl_->reset();
        retired_ctrl_.exchange(old, std::memory_order_acq_rel);
    }
}

void RealtimeTask::cycle(int64_t now_ns) {
    // 主站尚未 activate 时不碰总线：此时 domain 数据指针还不存在。
    // RT 线程先起来把 mlockall / 栈预触 / 调度优先级这些重活干完并空转，
    // 等 activate 完成后翻一个原子标志立刻接管——
    // 这样"activate 循环停止"到"RT 循环开始泵总线"之间没有空窗，
    // 不会因为几十毫秒没有过程数据而触发 SM 看门狗、把从站打回 SAFEOP。
    if (!bus_active_.load(std::memory_order_acquire)) return;

    swapPending();

    // ── 1. 收 ──────────────────────────────────────────────────────────
    bus_->receive();
    bus_->readInputs(&raw_);
    bus_->pollAsyncSdo(cycle_count_, &raw_);
    const BusStatus bus_st = bus_->status();

    // ── 2. 物理量换算 ───────────────────────────────────────────────────
    scaling_.toPhysical(raw_, &joint_);

    // ── 关节转动时长（2026-08-18）───────────────────────────────────────
    // 按实测转速判定（不是下发目标）：堵转不计时、软停尾段（<阈值）不计时。
    // total 不分运行内外——使能状态下被重力蠕动拖着转、CSP 挪位置都算磨损时间。
    if (reset_session_moving_req_.exchange(false, std::memory_order_acq_rel))
        session_moving_ns_ = 0;
    if (isJointMoving(joint_.output_vel_rpm)) {
        const int64_t dt_ns = static_cast<int64_t>(cfg_.ethercat.cycle_us) * 1000;
        session_moving_ns_ += dt_ns;
        total_moving_ns_.fetch_add(dt_ns, std::memory_order_relaxed);
    }

    // ── 3. 参数 / CiA402 ────────────────────────────────────────────────
    params_.refreshIfChanged(&param_view_);

    if (fault_reset_req_.exchange(false, std::memory_order_acq_rel))
        cia_.requestFaultReset(20);

    // start_run 时清掉上一次的 last_error（snap_ 归 RT 线程所有，seqlock 写侧，
    // IPC 线程不能直接碰，走和 fault_reset 一样的原子请求模式）。
    // 这样 GUI 只要看到非空 last_error 就一定是"本次或上次运行"的新错误。
    if (clear_error_req_.exchange(false, std::memory_order_acq_rel)) {
        snap_.last_error[0] = '\0';
        last_error_latched_ = false;
    }

    // 撤使能类目标必须等软停真正完成。否则控制字下一拍就切电，
    // 而斜坡还在数——手册 §7.1：>2.5 rpm 抱闸会永久损坏运动组件。
    //
    // 但"等软停"只在驱动器**当前确已带着力矩**（OperationEnabled）时才有意义
    // （见 shouldHoldEnableForDisableGate 的注释，终审 finding I2）：否则外力
    // 反驱关节超过安全转速时，这条门控会错误地把已经掉到更低状态的驱动器
    // 又拉回 EnableOperation——没有操作员动作的"自励磁"。
    {
        const auto want = static_cast<Cia402Target>(desired_target_.load(std::memory_order_relaxed));
        const bool is_disable = (want == Cia402Target::DisableVoltage);
        if (is_disable && shouldHoldEnableForDisableGate(cia_.state(), joint_.output_vel_rpm,
                                                           stopping_.load(std::memory_order_relaxed))) {
            cia_.setTarget(Cia402Target::EnableOperation);   // 维持使能，让斜坡把速度压下来
                                                              // （条件自持：只要还在 OperationEnabled
                                                              // 且不安全，下一拍解码出来的状态还是
                                                              // OperationEnabled，会继续进这个分支）
            ++disable_wait_cycles_;
            if (disableWaitTimedOut(disable_wait_cycles_, cfg_.stop_ramp.disable_timeout_cycles)) {
                cia_.setTarget(want);                        // 超时兜底，避免永远停不下来
                if (!last_error_latched_) {
                    // 只在超时刚发生的这一拍写一次，不要每拍重跑 %f snprintf
                    // 且永久覆盖 last_error（Minor finding）。
                    // stopping/斜坡余量是诊断信息：2026-08-18 真机上出现过从
                    // 低速停也走满超时的未解之谜（mock 复现不出），下次发生时
                    // 这两个数能直接指认是"斜坡没走完"还是"转速压不下去"。
                    snprintf(snap_.last_error, sizeof snap_.last_error,
                             "软停超时（%llu 拍），强制撤使能，转速 %.2f rpm"
                             "（stopping=%d 斜坡余量 %.2f rpm）",
                             (unsigned long long)disable_wait_cycles_,
                             joint_.output_vel_rpm,
                             stopping_.load(std::memory_order_relaxed) ? 1 : 0,
                             out_.target_vel_rpm);
                    last_error_latched_ = true;
                }
            }
        } else {
            disable_wait_cycles_ = 0;
            last_error_latched_ = false;   // 复位，下次软停超时要能重新报警
            cia_.setTarget(want);
        }
    }
    const uint16_t cw_out = cia_.update(raw_.statusword);
    const Cia402State cia_state = cia_.state();

    // ── 4. 运行模式 ─────────────────────────────────────────────────────
    const OpMode want_mode = static_cast<OpMode>(desired_mode_.load(std::memory_order_relaxed));
    raw_.modes_of_operation = static_cast<int8_t>(want_mode);
    mode_matched_ = (raw_.modes_display == static_cast<int8_t>(want_mode));

    // ── 5. 安全联锁 ─────────────────────────────────────────────────────
    // 任何一条不满足就必须退出运行并按斜坡归零（任务书第四十节）
    const bool ec_ok = (bus_st.slave_state == EcState::Op) && bus_st.slave_operational;
    const bool wc_ok = (bus_st.wc_state == 2);
    const bool servo_ok = (cia_state == Cia402State::OperationEnabled);
    const bool faulted = (cia_state == Cia402State::Fault ||
                          cia_state == Cia402State::FaultReactionActive);

    if (!wc_ok) stats_.wc_errors++;

    bool running = run_req_.load(std::memory_order_relaxed);
    if (running && (!ec_ok || !servo_ok || faulted || !mode_matched_)) {
        // 掉出安全条件 → 立即转入软停，而不是直接断使能
        run_req_.store(false, std::memory_order_relaxed);
        stopping_.store(true, std::memory_order_relaxed);
        running = false;
        snprintf(snap_.last_error, sizeof snap_.last_error,
                 "运行中断：EtherCAT=%s Servo=%s 模式匹配=%s",
                 ec_ok ? "OK" : "非OP", servo_ok ? "OK" : "未使能",
                 mode_matched_ ? "OK" : "不一致");
    }

    // ── 6. 轨迹 + 控制器 ────────────────────────────────────────────────
    if (running) {
        csp_idle_hold_.latched = false;   // 运行期间轨迹接管目标，停止后第一拍重新锁存
        if (run_start_ns_ == 0) {
            run_start_ns_ = now_ns;
            traj_->start(joint_, want_mode);
            ctrl_->reset();
            hold_position_deg_ = joint_.output_pos_unwrapped_deg;
        }
        const double t = static_cast<double>(now_ns - run_start_ns_) / 1e9;

        // Constant 轨迹的目标值可在线改，这里每拍取一次原子量
        ref_ = Setpoint{};
        traj_->eval(t, &ref_);
        // 只有 Constant 轨迹才用 GUI 的【目标值】实时覆盖。
        // 不能按"duration<0"判断——正弦/三角波把 duration 设为 -1 表示无限循环，
        // 那样会把一条正弦压成常值。
        if (traj_is_constant_.load(std::memory_order_relaxed)) {
            const double v = target_value_.load(std::memory_order_relaxed);
            switch (want_mode) {
                case OpMode::CSP: ref_.pos_deg = v; break;
                case OpMode::CSV: ref_.vel_rpm = v; break;
                case OpMode::CST: ref_.trq_Nm  = v; break;
                default: break;
            }
        }
        if (ref_.finished) {
            run_req_.store(false, std::memory_order_relaxed);
            stopping_.store(true, std::memory_order_relaxed);
        }

        ControlContext ctx;
        ctx.t = t;
        ctx.dt = cfg_.ethercat.cycle_us * 1e-6;
        ctx.mode = want_mode;
        ctx.ref = ref_;
        ctx.act = joint_;
        ctx.p = &param_view_;
        ctrl_->update(ctx, &out_);
    } else {
        run_start_ns_ = 0;
        safeStopRamp(&out_);
        ref_ = Setpoint{};
    }

    // ── 7. 输出限幅 ─────────────────────────────────────────────────────
    const double dt = cfg_.ethercat.cycle_us * 1e-6;
    const double tlim = cfg_.controller.torque_Nm_limit;
    out_.target_trq_Nm = std::clamp(out_.target_trq_Nm, -tlim, tlim);
    // 力矩变化率限制，防止阶跃冲击
    const double max_step = cfg_.controller.torque_rate_Nm_per_s * dt;
    const double d = out_.target_trq_Nm - last_out_.target_trq_Nm;
    if (std::fabs(d) > max_step)
        out_.target_trq_Nm = last_out_.target_trq_Nm + (d > 0 ? max_step : -max_step);

    // 速度同样要限变化率。真机上验证过：不限的话 Constant 轨迹一开跑就是
    // 0→100 rpm 的阶跃，驱动器跟不上，状态字 bit7 置位、
    // 0x3B68 报 0xFF00「软速度误差警告」。停止路径本来就有斜坡，起步路径漏了。
    // 只在运行中限；软停有自己的减速斜坡，不要叠加两层限幅。
    if (running) {
        const double vmax_step = cfg_.controller.velocity_rate_rpm_per_s * dt;
        const double dv = out_.target_vel_rpm - last_out_.target_vel_rpm;
        if (std::fabs(dv) > vmax_step)
            out_.target_vel_rpm = last_out_.target_vel_rpm +
                                  (dv > 0 ? vmax_step : -vmax_step);
    }
    // CSP 位置目标同样要限变化率（输出侧 °/s）。CSP 下驱动器不做 profile
    // 规划，Constant 轨迹的绝对角度目标一步下发就是位置阶跃，会被下面的
    // 阶跃兜底拒绝并软停——2026-08-13 真机：目标 202°、当前 109°，
    // 点【开始运行】第一拍即中止，GUI 看起来"没反应"。软件斜坡起点是
    // 上一拍的下发值（空闲时 = 锁存的保持位置，正好无缝衔接）。
    //
    // 只对 Constant 轨迹生效：正弦/三角/梯形/CSV 都从当前位置连续起步，
    // 没有阶跃风险（阶跃兜底仍在兜底），而寿命摆臂正弦 ±30° @0.25Hz 的
    // 峰值速度是 47.1 °/s——若被 15 °/s 斜坡限住会把正弦削成三角波，
    // 实验波形就错了。
    if (running && want_mode == OpMode::CSP &&
        traj_is_constant_.load(std::memory_order_relaxed)) {
        const double pmax_step = cfg_.controller.csp_position_rate_deg_per_s * dt;
        out_.target_pos_deg = slewLimit(out_.target_pos_deg,
                                        last_out_.target_pos_deg, pmax_step);
    }
    last_out_ = out_;

    // ── 8. 组装输出 PDO ─────────────────────────────────────────────────
    raw_.controlword = cw_out;
    raw_.max_torque = static_cast<uint16_t>(
        std::lround(cfg_.controller.torque_Nm_limit /
                    (cfg_.scaling.rated_torque_mNm / 1000.0) / cfg_.scaling.torque_scale));
    // 兜底：CSP 下驱动器不做 profile 限制，目标与实测差得太远就是一次位置阶跃。
    // 宁可拒绝下发并报错，也不要让它冲过去。
    // 注意：这条判定对 want_mode==CSP 恒生效，不只是停止瞬态——所以
    // stop_ramp.csp_hold_position=true（显式要求保持"Run 起始位置"）只在该位置与
    // 当前实测偏差 ≤ csp_target_jump_deg_max 时才会被真正履行，超过阈值会在下一拍
    // 被这里钳回当前实测位置并强制软停（失效到安全侧，但配置的字面语义不再可靠）。
    if (want_mode == OpMode::CSP) {
        const auto guard = cspTargetJumpGuard(out_.target_pos_deg,
                                               joint_.output_pos_unwrapped_deg,
                                               cfg_.scaling.csp_target_jump_deg_max);
        out_.target_pos_deg = guard.safe_target_deg;
        if (guard.triggered) {
            stopping_.store(true, std::memory_order_relaxed);
            run_req_.store(false, std::memory_order_relaxed);
            // 空闲保持的锁存值也重置到实测位置：不然锁存目标与实测偏差
            // 一旦超阈值（如外力强行搬动关节），这里会每拍触发、每拍重跑
            // snprintf——接受现实，从新位置继续保持。
            csp_idle_hold_.hold_deg = joint_.output_pos_unwrapped_deg;
            snprintf(snap_.last_error, sizeof snap_.last_error,
                     "CSP 目标位置阶跃 %.3f° 超过上限 %.3f°，已拒绝下发并软停",
                     std::fabs(guard.err_deg), cfg_.scaling.csp_target_jump_deg_max);
        }
    }
    switch (want_mode) {
        case OpMode::CSP:
            raw_.target_position = scaling_.degToTargetPosition(out_.target_pos_deg);
            raw_.target_velocity = 0;
            raw_.target_torque   = 0;
            break;
        case OpMode::CSV:
            raw_.target_velocity = scaling_.rpmToTargetVelocity(
                out_.target_vel_rpm, cfg_.scaling.target_velocity_is_motor_side);
            // CSV 下 0x607A 不生效，但让它跟随实际位置，避免切模式时突跳
            raw_.target_position = raw_.position_actual;
            raw_.target_torque   = 0;
            break;
        case OpMode::CST:
            raw_.target_torque   = scaling_.nmToTargetTorque(out_.target_trq_Nm);
            raw_.target_position = raw_.position_actual;
            raw_.target_velocity = 0;
            break;
        default:
            raw_.target_position = raw_.position_actual;
            raw_.target_velocity = 0;
            raw_.target_torque = 0;
            break;
    }
    bus_->writeOutputs(raw_);

    // ── 9. 采样入环 ─────────────────────────────────────────────────────
    Sample s;
    buildSample(&s, now_ns);
    log_ring_.push(s);      // 满了就丢并计数，绝不阻塞
    gui_ring_.push(s);
    stats_.dropped_log = log_ring_.dropped();
    stats_.dropped_gui = gui_ring_.dropped();

    // ── 10. 应用层状态机 ────────────────────────────────────────────────
    if (!ec_ok)                       app_state_ = AppState::EtherCatReady;
    if (bus_->phase() != BusPhase::Active) app_state_ = AppState::Disconnected;
    else if (faulted)                 app_state_ = AppState::Fault;
    else if (running)                 app_state_ = AppState::Running;
    else if (stopping_.load())        app_state_ = AppState::Stopping;
    else if (servo_ok)                app_state_ = mode_matched_ ? AppState::ReadyToRun
                                                                : AppState::ServoEnabled;
    else if (ec_ok)                   app_state_ = AppState::ServoDisabled;
    else                              app_state_ = AppState::EtherCatReady;

    // ── 11. 发 ──────────────────────────────────────────────────────────
    // ⚠ 必须用 CLOCK_MONOTONIC，且与 activate() 等待循环里用的是**同一个时钟**。
    //   曾经这里错用 CLOCK_REALTIME：activate 阶段喂 monotonic(~1e13)、
    //   RT 阶段突然喂 realtime(~1.7e18)，应用时间瞬间跳变 1.7e18 ns，
    //   从站 DC 直接失锁 → 刚进 OP 就掉回 SAFEOP。
    //   （样本时间戳用 CLOCK_REALTIME 是另一回事，那是给数据文件看的墙上时间。）
    bus_->send(now_ns);

    // ── 12. 快照（seqlock 写侧，永不阻塞）────────────────────────────────
    snap_seq_.fetch_add(1, std::memory_order_release);       // → 奇数
    std::atomic_thread_fence(std::memory_order_release);
    snap_.bus = bus_st;
    snap_.app_state = app_state_;
    snap_.cia_state = cia_state;
    snap_.controlword = cw_out;
    snap_.statusword = raw_.statusword;
    snap_.error_code = raw_.error_code;
    snap_.warning_code = raw_.warning_code;
    snap_.mode_display = raw_.modes_display;
    snap_.mode_selected = want_mode;
    snap_.running = running;
    snap_.mode_matched = mode_matched_;
    snap_.stopping = stopping_.load(std::memory_order_relaxed);
    snap_.stats = stats_;
    snap_.joint = joint_;
    snap_.ref = ref_;
    snap_.run_time_s = run_start_ns_ ? double(now_ns - run_start_ns_) / 1e9 : 0.0;
    snap_.moving_time_s = double(session_moving_ns_) / 1e9;
    snap_.total_moving_time_s =
        double(total_moving_ns_.load(std::memory_order_relaxed)) / 1e9;
    std::atomic_thread_fence(std::memory_order_release);
    snap_seq_.fetch_add(1, std::memory_order_release);       // → 偶数

    ++cycle_count_;
}

// ── 软停：任务书第二十一节 ─────────────────────────────────────────────────
// 停止运行 ≠ Servo Disable。CSV 按减速度降到 0，CST 按 ramp 降到 0，CSP 保持当前位置。
void RealtimeTask::safeStopRamp(ControlOutput* o) {
    const double dt = cfg_.ethercat.cycle_us * 1e-6;
    const OpMode m = static_cast<OpMode>(desired_mode_.load(std::memory_order_relaxed));
    bool done = false;
    switch (m) {
        case OpMode::CSV: {
            const double step = cfg_.stop_ramp.csv_decel_rpm_per_s * dt;
            if (o->target_vel_rpm > step)       o->target_vel_rpm -= step;
            else if (o->target_vel_rpm < -step) o->target_vel_rpm += step;
            else { o->target_vel_rpm = 0.0; done = true; }
            break;
        }
        case OpMode::CST: {
            const double step = cfg_.stop_ramp.cst_decel_Nm_per_s * dt;
            if (o->target_trq_Nm > step)       o->target_trq_Nm -= step;
            else if (o->target_trq_Nm < -step) o->target_trq_Nm += step;
            else { o->target_trq_Nm = 0.0; done = true; }
            break;
        }
        case OpMode::CSP:
        default: {
            // 2026-08-13 真机 finding：这里原来直接下发"当前实测位置"，
            // 参考每拍跟着反馈走 → 位置环零刚度，摆臂重力以粘滑蠕动把关节
            // 持续拖走（使能不点 Run，电机侧 0~20 rpm 波动下坠）。
            // 现在"可保持"期间锁存一次并钉住；未使能/模式未生效时退回跟随。
            const bool can_hold = (cia_.state() == Cia402State::OperationEnabled) &&
                                  mode_matched_ && m == OpMode::CSP;
            const double held = cspIdleHoldTarget(&csp_idle_hold_, can_hold,
                                                  joint_.output_pos_unwrapped_deg);
            o->target_pos_deg = cspStopTarget(cfg_.stop_ramp, hold_position_deg_, held);
            done = true;
            break;
        }
    }
    if (done) stopping_.store(false, std::memory_order_relaxed);
}

void RealtimeTask::buildSample(Sample* s, int64_t now_ns) {
    // elapsed 用【单调钟】算（now_ns 是本周期的 CLOCK_MONOTONIC，epoch 也是
    // 单调钟基准）：450h 持续运行工况断网跑，中途插网线 NTP 跳变校时/手动改
    // 时间都动不了文件内时间轴。system_time_ns 保留墙钟，做跨文件绝对对齐
    // 与人读对照——它跳只影响绝对时刻标注，seq 与 elapsed 保证文件内完整。
    const int64_t epoch = record_epoch_ns_.load(std::memory_order_relaxed);
    s->system_time_ns = nowRealtimeNs();
    s->elapsed_time_s = epoch ? double(now_ns - epoch) / 1e9 : 0.0;

    s->motor_position_unwrapped_deg  = joint_.motor_pos_unwrapped_deg;
    s->motor_position_deg            = joint_.motor_pos_deg;
    s->motor_velocity_rpm            = joint_.motor_vel_rpm;
    s->output_position_unwrapped_deg = joint_.output_pos_unwrapped_deg;
    s->output_position_deg           = joint_.output_pos_deg;
    s->output_velocity_rpm           = joint_.output_vel_rpm;
    s->motor_current_A               = joint_.current_A;
    s->actual_torque_Nm              = joint_.torque_Nm;
    s->target_position_deg           = ref_.pos_deg;
    s->target_velocity_rpm           = ref_.vel_rpm;
    s->target_torque_Nm              = out_.target_trq_Nm;
    s->position_error_deg            = ref_.pos_deg - joint_.output_pos_unwrapped_deg;
    // 速度误差必须与目标值同侧比：target_velocity_is_motor_side 决定
    // ref_.vel_rpm 是电机侧还是输出侧（2026-08-13 起配置为输出侧）
    s->velocity_error_rpm            = ref_.vel_rpm -
        (cfg_.scaling.target_velocity_is_motor_side ? joint_.motor_vel_rpm
                                                    : joint_.output_vel_rpm);
    s->motor_torque_Nm               = joint_.torque_Nm / cfg_.scaling.gear_ratio;
    s->torque_est_Nm                 = raw_.vendor_torque / 1000.0;

    s->motor_position_raw  = joint_.motor_pos_raw;
    s->output_position_raw = joint_.output_pos_raw;
    s->twist_counts            = raw_.twist_counts;
    s->following_error_counts  = raw_.following_error;
    s->torque_est_mNm          = raw_.vendor_torque;
    s->aux_position_raw        = raw_.output_position;
    s->position_counts_raw     = raw_.position_counts;
    s->motor_position_sdo      = raw_.motor_position_sdo;
    s->dc_link_voltage_mV      = raw_.dc_link_mV;
    s->warning_code            = raw_.warning_code;
    s->working_counter     = snap_.bus.working_counter;
    s->seq                 = static_cast<uint32_t>(cycle_count_);

    s->controlword = raw_.controlword;
    s->statusword  = raw_.statusword;
    s->error_code             = raw_.error_code;
    s->temperature_drive_C    = raw_.drive_temp_C;
    s->torque_actual_permille = raw_.torque_actual;
    s->torque_ratio           = raw_.torque_ratio;
    s->operation_mode = raw_.modes_display;
    s->cia402_state   = static_cast<uint8_t>(cia_.state());
    s->ethercat_state = static_cast<uint8_t>(snap_.bus.slave_state);
    s->flags = 0;
    if (run_req_.load(std::memory_order_relaxed)) s->flags |= kFlagRunning;
    if (recording_.load(std::memory_order_relaxed)) s->flags |= kFlagRecording;
    if (cia_.state() == Cia402State::Fault) s->flags |= kFlagFault;
}

// ── IPC 线程侧接口 ────────────────────────────────────────────────────────
void RealtimeTask::servoEnable() {
    desired_target_.store(static_cast<int>(Cia402Target::EnableOperation));
}
void RealtimeTask::servoDisable() {
    run_req_.store(false);
    stopping_.store(true);
    desired_target_.store(static_cast<int>(Cia402Target::DisableVoltage));
}
void RealtimeTask::faultReset() { fault_reset_req_.store(true); }
void RealtimeTask::quickStop() {
    run_req_.store(false);
    desired_target_.store(static_cast<int>(Cia402Target::QuickStop));
}

bool RealtimeTask::setMode(OpMode m, std::string* err) {
    if (run_req_.load()) {
        *err = "运行中禁止切换运行模式，请先【停止运行】";
        return false;
    }
    if (m == OpMode::Homing && !cfg_.slave.supports_homing) {
        *err = "本驱动器不支持 Homing（0x6502 = 0x" +
               std::to_string(cfg_.slave.supported_modes_raw) + "，不含 hm 位）";
        return false;
    }
    desired_mode_.store(static_cast<int>(m));
    return true;
}

bool RealtimeTask::setController(const std::string& id, std::string* err) {
    if (run_req_.load()) { *err = "运行中禁止切换控制器，请先【停止运行】"; return false; }
    auto c = makeController(id);
    if (!c) { *err = "未知控制器 id: " + id; return false; }

    ParameterRegistry reg;
    c->declareParams(reg);
    params_.redefine(reg.metas());
    {
        std::lock_guard<std::mutex> lk(ctrl_id_mu_);
        ctrl_id_ = id;
    }
    reapRetired();
    // 交给 RT 线程，旧的由 RT 交还后在 reapRetired() 里释放
    ControllerBase* old_pending = pending_ctrl_.exchange(c.release());
    delete old_pending;      // 上一次还没被 RT 取走的，直接丢弃
    return true;
}

bool RealtimeTask::setTrajectory(const TrajParams& p, std::string* err) {
    if (run_req_.load()) { *err = "运行中禁止切换轨迹，请先【停止运行】"; return false; }
    auto t = makeTrajectory(p, err);   // CSV 文件的磁盘读取发生在这里，不在 RT 线程
    if (!t) return false;
    traj_is_constant_.store(p.type == TrajType::Constant, std::memory_order_relaxed);
    reapRetired();
    TrajectoryBase* old_pending = pending_traj_.exchange(t.release());
    delete old_pending;
    return true;
}

void RealtimeTask::reapRetired() {
    delete retired_traj_.exchange(nullptr);
    delete retired_ctrl_.exchange(nullptr);
}

void RealtimeTask::setTargetValue(double v) { target_value_.store(v); }

bool RealtimeTask::checkRunPreconditions(std::string* err) {
    const StatusSnapshot s = snapshot();
    if (s.bus.slave_state != EcState::Op || !s.bus.slave_operational) {
        *err = "EtherCAT 未进入 OP（当前 " + std::string(toString(s.bus.slave_state)) +
               "），禁止开始运动";
        return false;
    }
    if (s.cia_state != Cia402State::OperationEnabled) {
        *err = "伺服未使能（CiA402 当前 " + std::string(toString(s.cia_state)) +
               "），请先点【Servo Enable】";
        return false;
    }
    if (!s.mode_matched) {
        *err = "运行模式尚未生效：0x6060 已写入 " +
               std::string(toString(s.mode_selected)) + "，但 0x6061 回读为 " +
               std::to_string(static_cast<int>(s.mode_display)) + "，请稍候或重选模式";
        return false;
    }
    if (s.cia_state == Cia402State::Fault) { *err = "驱动器处于故障状态，请先 Fault Reset"; return false; }
    return true;
}

bool RealtimeTask::startRun(std::string* err) {
    if (!checkRunPreconditions(err)) return false;
    clear_error_req_.store(true);   // 新一次运行开始，旧的中止原因作废
    reset_session_moving_req_.store(true);   // 本次转动时长按运行分段，开始即清零
    stopping_.store(false);
    run_start_ns_ = 0;
    run_req_.store(true);
    return true;
}

void RealtimeTask::stopRun() {
    run_req_.store(false);
    stopping_.store(true);
}

StatusSnapshot RealtimeTask::snapshot() {
    // seqlock 读侧：拷贝前后 seq 一致且为偶数才算读到一致快照
    StatusSnapshot out;
    for (int i = 0; i < 64; ++i) {
        const uint64_t s1 = snap_seq_.load(std::memory_order_acquire);
        if (s1 & 1) continue;
        std::atomic_thread_fence(std::memory_order_acquire);
        out = snap_;
        std::atomic_thread_fence(std::memory_order_acquire);
        if (snap_seq_.load(std::memory_order_acquire) == s1) return out;
    }
    return out;   // 极端情况下返回可能不一致的快照，但只影响一帧显示
}

}  // namespace ecjc
