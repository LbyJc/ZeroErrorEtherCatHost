// types.hpp —— Backend 与 GUI 之间的线格式(wire format)。
//
// ⚠ 这个文件是 C++ 与 Python 的契约。改动 Sample 的任何字段，
//   必须同步改 gui/ipc_client.py 里的 SAMPLE_FORMAT，否则 GUI 会画出乱码曲线。
//   为此 Sample 有 static_assert 锁尺寸，GUI 连上时也会校验一次尺寸，不一致直接报错。
#pragma once

#include <cstdint>
#include <cstddef>

namespace ecjc {

// ── 帧 ────────────────────────────────────────────────────────────────────
constexpr uint32_t kFrameMagic = 0x434A4345u;   // 'ECJC' (小端)
constexpr uint16_t kProtocolVersion = 3;

enum class FrameType : uint16_t {
    Telemetry = 1,     // payload = N × Sample，二进制
    Json      = 2,     // payload = UTF-8 JSON，低频命令/状态/日志
};

struct FrameHeader {
    uint32_t magic;
    uint16_t type;
    uint16_t version;
    uint32_t length;   // payload 字节数，不含本头
};
static_assert(sizeof(FrameHeader) == 12, "FrameHeader 必须是 12 字节");

// ── CiA402 ────────────────────────────────────────────────────────────────
enum class Cia402State : uint8_t {
    NotReadyToSwitchOn = 0,
    SwitchOnDisabled   = 1,
    ReadyToSwitchOn    = 2,
    SwitchedOn         = 3,
    OperationEnabled   = 4,
    QuickStopActive    = 5,
    FaultReactionActive= 6,
    Fault              = 7,
    Unknown            = 8,
};
const char* toString(Cia402State s);

// CiA402 运行模式 (0x6060 / 0x6061)
enum class OpMode : int8_t {
    None    = 0,
    PP      = 1,   // Profile Position
    PV      = 3,   // Profile Velocity
    PT      = 4,   // Profile Torque
    Homing  = 6,
    CSP     = 8,   // Cyclic Synchronous Position
    CSV     = 9,   // Cyclic Synchronous Velocity
    CST     = 10,  // Cyclic Synchronous Torque
};
const char* toString(OpMode m);
bool parseOpMode(const char* name, OpMode* out);

// EtherCAT AL 状态
enum class EcState : uint8_t {
    Unknown = 0, Init = 1, PreOp = 2, Boot = 3, SafeOp = 4, Op = 8,
};
const char* toString(EcState s);

// 应用层状态机（任务书第四十一节）
enum class AppState : uint8_t {
    Disconnected  = 0,
    MasterReady   = 1,
    EtherCatReady = 2,
    ServoDisabled = 3,
    ServoEnabled  = 4,
    ReadyToRun    = 5,
    Running       = 6,
    Stopping      = 7,
    Fault         = 8,
};
const char* toString(AppState s);

// ── 遥测样本 ──────────────────────────────────────────────────────────────
// 字段顺序按自然对齐排布（8 字节的在前），保证无内部填充、跨语言布局稳定。
// Python 对应格式串: "<q16d8i4I4H2hbBBB" —— 以 gui/ipc_client.py 的 SAMPLE_FORMAT 为准，
// 两边尺寸不一致时 GUI 连接握手会直接报错。
struct Sample {
    int64_t  system_time_ns;                  // Unix epoch 纳秒
    double   elapsed_time_s;                  // 从本次采集开始计时

    double   motor_position_unwrapped_deg;    // 多圈，不丢圈数
    double   motor_position_deg;              // wrapped 到 [0,360)
    double   motor_velocity_rpm;
    double   output_position_unwrapped_deg;
    double   output_position_deg;
    double   output_velocity_rpm;
    double   motor_current_A;
    double   actual_torque_Nm;
    double   target_position_deg;
    double   target_velocity_rpm;
    double   target_torque_Nm;
    double   position_error_deg;
    double   velocity_error_rpm;
    double   motor_torque_Nm;                 // 电机轴侧 = actual_torque_Nm ÷ 减速比
    double   torque_est_Nm;                   // 0x3B69 mNm→Nm，厂商传递力矩估计
                                              //   （单位经 0x3B6A≈0x3B69/额定31Nm 真机交叉验证）

    int32_t  motor_position_raw;              // 原始计数，必须保留
    int32_t  output_position_raw;
    int32_t  twist_counts;                    // 0x2241，电机侧计数
    int32_t  following_error_counts;          // 0x60F4
    int32_t  torque_est_mNm;                  // 0x3B69
    int32_t  aux_position_raw;                // 0x20A0
    int32_t  position_counts_raw;             // 0x6063
    int32_t  motor_position_sdo;              // 0x2240 异步 SDO 通道（对拍用）
    uint32_t dc_link_voltage_mV;              // 0x6079
    uint32_t warning_code;                    // 0x3B68
    uint32_t working_counter;
    uint32_t seq;                             // 单调递增，GUI 用它检测丢包

    uint16_t controlword;
    uint16_t statusword;
    uint16_t error_code;                      // 0x603F
    uint16_t temperature_drive_C;             // 0x22A2。⚠ 驱动器温度，非绕组非壳体；
                                               //   异步 SDO，采样时刻与本样本不同步
    int16_t  torque_actual_permille;          // 0x6077 原始千分比
    int16_t  torque_ratio;                    // 0x3B6A

    int8_t   operation_mode;                  // 0x6061 实际值
    uint8_t  cia402_state;                    // Cia402State
    uint8_t  ethercat_state;                  // EcState
    uint8_t  flags;                           // bit0 running, bit1 recording, bit2 fault
};
static_assert(sizeof(Sample) == 200,
              "Sample 布局变了！同步更新 gui/ipc_client.py 的 SAMPLE_FORMAT 与 kProtocolVersion");
static_assert(offsetof(Sample, motor_position_raw) == 136, "Sample 布局意外填充");

constexpr uint8_t kFlagRunning   = 0x01;
constexpr uint8_t kFlagRecording = 0x02;
constexpr uint8_t kFlagFault     = 0x04;

// ── 从站原始 PDO 镜像（RT 线程内部用，不上线）────────────────────────────
struct RawIo {
    // 输入
    uint16_t error_code    = 0;
    uint16_t statusword    = 0;
    int32_t  position_actual = 0;   // 0x6064
    int32_t  velocity_actual = 0;   // 0x606C
    int16_t  torque_actual   = 0;   // 0x6077
    int8_t   modes_display   = 0;   // 0x6061
    int32_t  output_position = 0;   // 0x20A0
    int32_t  position_counts = 0;   // 0x6063
    int16_t  current_actual  = 0;   // 0x6078
    uint32_t warning_code    = 0;   // 0x3B68
    int32_t  motor_position  = 0;   // 0x2240，异步 SDO 取得
    int32_t  vendor_torque   = 0;   // 0x3B69，mNm（此前已映射进 PDO 但从未读取）
    int16_t  torque_ratio    = 0;   // 0x3B6A（同上）
    int32_t  twist_counts    = 0;   // 0x2241，电机侧 17 位计数
    int32_t  following_error = 0;   // 0x60F4
    uint32_t dc_link_mV      = 0;   // 0x6079
    uint16_t drive_temp_C    = 0;   // 0x22A2，异步 SDO，与 PDO 不同步
    int32_t  motor_position_sdo = 0;// 0x2240 的异步 SDO 通道，J3 对拍专用。
                                    // 必须与 motor_position（PDO 通道）分开，
                                    // 否则两个写入者竞争同一字段，值会抖动
    // 输出
    int32_t  target_position = 0;   // 0x607A
    int32_t  target_velocity = 0;   // 0x60FF
    int16_t  target_torque   = 0;   // 0x6071
    uint16_t max_torque      = 0;   // 0x6072
    uint16_t controlword     = 0;   // 0x6040
    int8_t   modes_of_operation = 0;// 0x6060
};

// ── 物理量 ────────────────────────────────────────────────────────────────
struct JointState {
    double motor_pos_deg = 0, motor_pos_unwrapped_deg = 0, motor_vel_rpm = 0;
    double output_pos_deg = 0, output_pos_unwrapped_deg = 0, output_vel_rpm = 0;
    double current_A = 0, torque_Nm = 0;
    int32_t motor_pos_raw = 0, output_pos_raw = 0;
};

struct Setpoint {
    double pos_deg = 0, vel_rpm = 0, trq_Nm = 0;
    bool   finished = false;
};

struct ControlOutput {
    double target_pos_deg = 0, target_vel_rpm = 0, target_trq_Nm = 0;
};

// ── RT 线程统计 ───────────────────────────────────────────────────────────
struct RtStats {
    uint64_t cycles = 0;
    int64_t  jitter_ns = 0;          // 本周期
    int64_t  jitter_max_ns = 0;
    int64_t  jitter_min_ns = 0;
    double   jitter_mean_ns = 0;
    uint64_t deadline_miss = 0;      // 唤醒晚于一个完整周期
    uint64_t wc_errors = 0;
    uint64_t dropped_log = 0;
    uint64_t dropped_gui = 0;
};

}  // namespace ecjc
