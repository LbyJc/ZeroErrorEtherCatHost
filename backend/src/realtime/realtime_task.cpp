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

    // 退出前把输出清零并撤使能，不要让从站带着最后一条命令留在 OP
    for (int i = 0; i < 50 && bus_->phase() == BusPhase::Active; ++i) {
        bus_->receive();
        RawIo z{};
        z.controlword = cw::kCmdDisableVoltage;
        z.modes_of_operation = raw_.modes_of_operation;
        bus_->writeOutputs(z);
        bus_->send(nowMonotonicNs());
        addNs(&wakeup, cycle_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, nullptr);
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

    // ── 3. 参数 / CiA402 ────────────────────────────────────────────────
    params_.refreshIfChanged(&param_view_);

    if (fault_reset_req_.exchange(false, std::memory_order_acq_rel))
        cia_.requestFaultReset(20);

    // 撤使能类目标必须等软停真正完成。否则控制字下一拍就切电，
    // 而斜坡还在数——手册 §7.1：>2.5 rpm 抱闸会永久损坏运动组件。
    {
        const auto want = static_cast<Cia402Target>(desired_target_.load(std::memory_order_relaxed));
        const bool is_disable = (want == Cia402Target::DisableVoltage);
        if (is_disable && !isSafeToDisableAt(joint_.output_vel_rpm,
                                             stopping_.load(std::memory_order_relaxed))) {
            cia_.setTarget(Cia402Target::EnableOperation);   // 维持使能，让斜坡把速度压下来
            ++disable_wait_cycles_;
            if (disableWaitTimedOut(disable_wait_cycles_, cfg_.stop_ramp.disable_timeout_cycles)) {
                cia_.setTarget(want);                        // 超时兜底，避免永远停不下来
                snprintf(snap_.last_error, sizeof snap_.last_error,
                         "软停超时（%llu 拍），强制撤使能，转速 %.2f rpm",
                         (unsigned long long)disable_wait_cycles_, joint_.output_vel_rpm);
            }
        } else {
            disable_wait_cycles_ = 0;
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
        default:
            o->target_pos_deg = cspStopTarget(cfg_.stop_ramp,
                                              hold_position_deg_,
                                              joint_.output_pos_unwrapped_deg);
            done = true;
            break;
    }
    if (done) stopping_.store(false, std::memory_order_relaxed);
}

void RealtimeTask::buildSample(Sample* s, int64_t now_ns) {
    const int64_t epoch = record_epoch_ns_.load(std::memory_order_relaxed);
    s->system_time_ns = nowRealtimeNs();
    s->elapsed_time_s = epoch ? double(s->system_time_ns - epoch) / 1e9 : 0.0;

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
    s->velocity_error_rpm            = ref_.vel_rpm - joint_.motor_vel_rpm;

    s->motor_position_raw  = joint_.motor_pos_raw;
    s->output_position_raw = joint_.output_pos_raw;
    s->working_counter     = snap_.bus.working_counter;
    s->seq                 = static_cast<uint32_t>(cycle_count_);

    s->controlword = raw_.controlword;
    s->statusword  = raw_.statusword;
    s->operation_mode = raw_.modes_display;
    s->cia402_state   = static_cast<uint8_t>(cia_.state());
    s->ethercat_state = static_cast<uint8_t>(snap_.bus.slave_state);
    s->flags = 0;
    if (run_req_.load(std::memory_order_relaxed)) s->flags |= kFlagRunning;
    if (recording_.load(std::memory_order_relaxed)) s->flags |= kFlagRecording;
    if (cia_.state() == Cia402State::Fault) s->flags |= kFlagFault;
    (void)now_ns;
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
