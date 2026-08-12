#include "ecjc/data_logger.hpp"

#include <hdf5.h>

#include <sys/statvfs.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <mutex>

namespace ecjc {
namespace fs = std::filesystem;
namespace {

constexpr size_t kBatch = 1000;          // 一次写 1000 个样本（1 kHz 下 = 1 秒）
constexpr hsize_t kChunk = 4096;

std::string nowString() {
    const auto now = std::chrono::system_clock::now();
    const auto t = std::chrono::system_clock::to_time_t(now);
    const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                        now.time_since_epoch()).count() % 1000000;
    struct tm tmv;
    localtime_r(&t, &tmv);
    char buf[64];
    strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", &tmv);
    char out[80];
    snprintf(out, sizeof out, "%s.%06ld", buf, static_cast<long>(us));
    return out;
}

std::string fileStamp() {
    const auto t = std::time(nullptr);
    struct tm tmv;
    localtime_r(&t, &tmv);
    char buf[32];
    strftime(buf, sizeof buf, "%Y%m%d_%H%M%S", &tmv);
    return buf;
}

double diskFreeGb(const std::string& path) {
    struct statvfs s;
    if (statvfs(path.c_str(), &s) != 0) return -1.0;
    return double(s.f_bavail) * double(s.f_frsize) / 1e9;
}

// 一个字段 = 一个可扩展 dataset
struct Column {
    const char* name;
    hid_t type;
    hid_t dset = -1;
    hsize_t rows = 0;
};

}  // namespace

struct DataLogger::Impl {
    hid_t file = -1;
    hid_t group = -1;
    std::vector<Column> cols;
    std::vector<double> scratch;      // 一列一批的临时缓冲，启动时分配好
    std::vector<int64_t> scratch_i64;
    std::vector<int32_t> scratch_i32;
    std::vector<uint32_t> scratch_u32;
    std::vector<uint16_t> scratch_u16;
    std::vector<int16_t>  scratch_i16;
    std::vector<uint8_t>  scratch_u8;
    std::vector<int8_t>   scratch_i8;
    std::vector<Sample> batch;
};

DataLogger::DataLogger(const FullConfig& cfg, SpscRing<Sample>* ring)
    : cfg_(cfg), ring_(ring), impl_(std::make_unique<Impl>()) {
    impl_->batch.resize(kBatch);
    impl_->scratch.resize(kBatch);
    impl_->scratch_i64.resize(kBatch);
    impl_->scratch_i32.resize(kBatch);
    impl_->scratch_u32.resize(kBatch);
    impl_->scratch_u16.resize(kBatch);
    impl_->scratch_i16.resize(kBatch);
    impl_->scratch_u8.resize(kBatch);
    impl_->scratch_i8.resize(kBatch);
    th_ = std::thread([this] { threadMain(); });
}

DataLogger::~DataLogger() {
    quit_.store(true);
    if (th_.joinable()) th_.join();
    closeFile();
}

bool DataLogger::start(const RecordingMeta& meta, std::string* err) {
    if (active_.load()) { *err = "数据采集已在进行中"; return false; }
    meta_ = meta;
    meta_.start_time = nowString();

    const double free_gb = diskFreeGb(cfg_.app.data_dir);
    if (free_gb >= 0 && free_gb < min_free_gb_) {
        *err = "磁盘剩余空间仅 " + std::to_string(free_gb) +
               " GB，低于安全阈值 " + std::to_string(min_free_gb_) + " GB，拒绝开始采集";
        return false;
    }
    if (!openFile(err)) return false;

    {
        std::lock_guard<std::mutex> lk(st_mu_);
        st_ = RecordingStatus{};
        st_.active = true;
        st_.file = impl_ ? cur_path_ : "";
        st_.start_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        st_.disk_free_gb = free_gb;
    }
    active_.store(true);
    return true;
}

void DataLogger::stop() {
    if (!active_.load()) return;
    active_.store(false);
    // 让 Logger 线程把 ring 里剩下的写完再关文件
    for (int i = 0; i < 100 && ring_->size() > 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    meta_.end_time = nowString();
    closeFile();
    std::lock_guard<std::mutex> lk(st_mu_);
    st_.active = false;
}

RecordingStatus DataLogger::status() const {
    std::lock_guard<std::mutex> lk(st_mu_);
    RecordingStatus s = st_;
    s.buffer_usage = ring_->usage();
    s.dropped = ring_->dropped();
    return s;
}

bool DataLogger::openFile(std::string* err) {
    std::error_code ec;
    fs::create_directories(cfg_.app.data_dir, ec);

    std::string base = meta_.test_name.empty() ? "exp" : meta_.test_name;
    for (auto& c : base) if (c == '/' || c == ' ') c = '_';
    cur_path_ = (fs::path(cfg_.app.data_dir) /
                 (base + "_" + fileStamp() + ".h5")).string();

    impl_->file = H5Fcreate(cur_path_.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    if (impl_->file < 0) {
        *err = "无法创建数据文件 " + cur_path_ + "（检查目录权限与磁盘空间）";
        return false;
    }
    impl_->group = H5Gcreate2(impl_->file, "/experiment", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    // ── 写 metadata（任务书第四十九节）──────────────────────────────
    auto attrStr = [&](const char* k, const std::string& v) {
        hid_t s = H5Screate(H5S_SCALAR);
        hid_t t = H5Tcopy(H5T_C_S1);
        H5Tset_size(t, v.size() + 1);
        hid_t a = H5Acreate2(impl_->group, k, t, s, H5P_DEFAULT, H5P_DEFAULT);
        H5Awrite(a, t, v.c_str());
        H5Aclose(a); H5Tclose(t); H5Sclose(s);
    };
    auto attrDbl = [&](const char* k, double v) {
        hid_t s = H5Screate(H5S_SCALAR);
        hid_t a = H5Acreate2(impl_->group, k, H5T_NATIVE_DOUBLE, s, H5P_DEFAULT, H5P_DEFAULT);
        H5Awrite(a, H5T_NATIVE_DOUBLE, &v);
        H5Aclose(a); H5Sclose(s);
    };

    attrStr("test_name", meta_.test_name);
    attrStr("description", meta_.description);
    attrStr("operation_mode", meta_.operation_mode);
    attrStr("controller", meta_.controller);
    attrStr("control_params", meta_.control_params_json);
    attrStr("slave_name", meta_.slave_name);
    attrStr("software_version", meta_.software_version);
    attrStr("start_time", meta_.start_time);
    attrDbl("cycle_us", meta_.cycle_us);
    attrDbl("sampling_hz", meta_.sampling_hz);
    {
        // 存十六进制字符串而不是 double：追溯时 0x5A65726F 才认得出来，
        // 1516597871.0 没人能一眼看懂
        char b[24];
        snprintf(b, sizeof b, "0x%08X", meta_.vendor_id);
        attrStr("vendor_id", b);
        snprintf(b, sizeof b, "0x%08X", meta_.product_code);
        attrStr("product_code", b);
    }
    attrDbl("motor_encoder_counts_per_rev", meta_.motor_counts_per_rev);
    attrDbl("output_encoder_counts_per_rev", meta_.output_counts_per_rev);
    attrDbl("gear_ratio", meta_.gear_ratio);
    attrStr("encoder_resolution_verified",
            meta_.encoder_resolution_verified ? "true"
                : "false (输出侧分辨率未经物理转角验证；若实为2^18则所有rpm翻倍)");

    // ── 标定来源声明（Task 13：审核定稿口径，逐字照抄，不得改写）────────
    attrStr("gear_ratio_source",
            "手册§12 n_out=n_motor/(X+1) + §2表2-1；实测 Δ0x2240/Δ0x6064=30.234"
            "（自身精度仅 0.05%，足以排除 120 不足以细分）");
    attrStr("encoder_accuracy_note",
            "型号说明称 HM 可提供 20 位/±7角秒；表2-2 eRob80H 选装配置列 19Bit/±10角秒；待厂家澄清");
    attrStr("torque_est_source",
            "0x3B69 与 0x2241 同源同刻计算，驱动器手册称『估计』，全机无独立力传感元件；"
            "不得用于刚度退化判定");
    attrStr("temperature_scope",
            "仅驱动器温度 0x22A2，非绕组非壳体；单位未经手册核实（eTuner 界面显示为 ℃）；"
            "异步 SDO，采样时刻与 PDO 不同步");
    attrStr("twist_sign_convention",
            "theta_twist = theta_out - theta_in/i（与计划§4.4 TE 同号）");
    attrStr("git_commit",   meta_.git_commit);
    attrStr("config_sha256", meta_.config_sha256);
    // 终审 finding I3：sm_in_* 系列诊断 SDO（0x1C33 相关）是 activate **之前**
    // 读的（PreActivate 相位），而映射写入发生在 activate 之后的 PREOP→SAFEOP
    // 迁移里——所以这些值反映的是上一次会话的映射与累计计数，不是本次采集期间
    // 的实况。本次会话真正的 SM 时序，运行中用 tools/verify_pdo_remap.py 的 J6
    // 现场读取（那条路径在 OP 状态下问 0x1C33，读到的才是当次会话的数）。
    attrStr("sm_diagnostics_scope",
            "activate 前快照：反映上一次会话的映射与累计计数，非本次采集期间；"
            "本次会话的 SM 时序用 tools/verify_pdo_remap.py 的 J6 在运行中读取");
    // 全部诊断 SDO 实读值（activate 前一次性读，见 IEtherCATBus::diagnostics()）
    for (const auto& kv : meta_.diagnostics) {
        if (kv.second == INT64_MIN) attrStr(("sdo_" + kv.first).c_str(), "read_failed");
        else                        attrDbl(("sdo_" + kv.first).c_str(), (double)kv.second);
    }

    // ── 建列 ───────────────────────────────────────────────────────
    // 列名/类型的唯一数据源是 ECJC_SAMPLE_COLUMNS（data_logger.hpp）：
    // 建列顺序与写入顺序由同一份宏展开保证一致，不再是靠 k++ 的位置耦合。
    static const struct { const char* n; hid_t t; } kCols[] = {
#define X(name, h5type, tag, expr) {#name, h5type},
        ECJC_SAMPLE_COLUMNS(X)
#undef X
    };

    impl_->cols.clear();
    for (const auto& c : kCols) {
        hsize_t dims[1] = {0}, maxd[1] = {H5S_UNLIMITED}, chunk[1] = {kChunk};
        hid_t space = H5Screate_simple(1, dims, maxd);
        hid_t plist = H5Pcreate(H5P_DATASET_CREATE);
        H5Pset_chunk(plist, 1, chunk);
        H5Pset_deflate(plist, 4);      // 压缩：10 小时实验从 ~6.6GB 降到 2~3GB
        hid_t d = H5Dcreate2(impl_->group, c.n, c.t, space, H5P_DEFAULT, plist, H5P_DEFAULT);
        H5Pclose(plist); H5Sclose(space);
        if (d < 0) { *err = std::string("创建数据列失败: ") + c.n; return false; }
        impl_->cols.push_back(Column{c.n, c.t, d, 0});
    }
    return true;
}

void DataLogger::closeFile() {
    if (!impl_ || impl_->file < 0) return;
    if (!meta_.end_time.empty() && impl_->group >= 0) {
        hid_t s = H5Screate(H5S_SCALAR);
        hid_t t = H5Tcopy(H5T_C_S1);
        H5Tset_size(t, meta_.end_time.size() + 1);
        hid_t a = H5Acreate2(impl_->group, "end_time", t, s, H5P_DEFAULT, H5P_DEFAULT);
        H5Awrite(a, t, meta_.end_time.c_str());
        H5Aclose(a); H5Tclose(t); H5Sclose(s);
    }
    for (auto& c : impl_->cols) if (c.dset >= 0) H5Dclose(c.dset);
    impl_->cols.clear();
    if (impl_->group >= 0) { H5Gclose(impl_->group); impl_->group = -1; }
    H5Fclose(impl_->file);
    impl_->file = -1;
}

bool DataLogger::writeBatch(const Sample* s, size_t n, std::string* err) {
    if (impl_->file < 0 || n == 0) return true;

    auto extend = [&](Column& c, const void* data) -> bool {
        hsize_t newsize[1] = {c.rows + n};
        if (H5Dset_extent(c.dset, newsize) < 0) return false;
        hid_t fs = H5Dget_space(c.dset);
        hsize_t start[1] = {c.rows}, count[1] = {n};
        H5Sselect_hyperslab(fs, H5S_SELECT_SET, start, nullptr, count, nullptr);
        hid_t ms = H5Screate_simple(1, count, nullptr);
        const herr_t r = H5Dwrite(c.dset, c.type, ms, fs, H5P_DEFAULT, data);
        H5Sclose(ms); H5Sclose(fs);
        if (r < 0) return false;
        c.rows += n;
        return true;
    };

    size_t k = 0;
    auto& d   = impl_->scratch;
    auto& i64 = impl_->scratch_i64;
    auto& i32 = impl_->scratch_i32;
    auto& u32 = impl_->scratch_u32;
    auto& u16 = impl_->scratch_u16;
    auto& i16 = impl_->scratch_i16;
    auto& u8  = impl_->scratch_u8;
    auto& i8  = impl_->scratch_i8;

#define COL_D(expr)  { for (size_t i=0;i<n;++i) d[i]  = (expr); if(!extend(impl_->cols[k++], d.data()))   goto fail; }
#define COL_I64(expr){ for (size_t i=0;i<n;++i) i64[i]= (expr); if(!extend(impl_->cols[k++], i64.data())) goto fail; }
#define COL_I32(expr){ for (size_t i=0;i<n;++i) i32[i]= (expr); if(!extend(impl_->cols[k++], i32.data())) goto fail; }
#define COL_U32(expr){ for (size_t i=0;i<n;++i) u32[i]= (expr); if(!extend(impl_->cols[k++], u32.data())) goto fail; }
#define COL_U16(expr){ for (size_t i=0;i<n;++i) u16[i]= (expr); if(!extend(impl_->cols[k++], u16.data())) goto fail; }
#define COL_I16(expr){ for (size_t i=0;i<n;++i) i16[i]= (expr); if(!extend(impl_->cols[k++], i16.data())) goto fail; }
#define COL_U8(expr) { for (size_t i=0;i<n;++i) u8[i] = (expr); if(!extend(impl_->cols[k++], u8.data()))  goto fail; }
#define COL_I8(expr) { for (size_t i=0;i<n;++i) i8[i] = (expr); if(!extend(impl_->cols[k++], i8.data()))  goto fail; }

    // 写入顺序由 ECJC_SAMPLE_COLUMNS 这一份宏保证与建列顺序（kCols[]）恒等，
    // 不再依赖人工数手动对齐 k++。
#define X(name, h5type, tag, expr) COL_##tag(expr)
    ECJC_SAMPLE_COLUMNS(X)
#undef X

#undef COL_D
#undef COL_I64
#undef COL_I32
#undef COL_U32
#undef COL_U16
#undef COL_I16
#undef COL_U8
#undef COL_I8

    // 运行期校验（终审 finding M1，如实描述）：防的是"建列部分失败的错位"——
    // openFile() 里 kCols[] 的某一列 H5Dcreate2 失败时会立刻 return false，
    // impl_->cols 因此可能只建到半途；这里检查本批 X-macro 实际写出的列数 k
    // 是否等于 impl_->cols.size()，能在这种"建列表比预期短"的情形下挡住
    // 错位写入，而不是宣称能捕获宏被改动的每一种展开错误组合——那种情形下
    // COL_##tag 多半直接是编译错误，轮不到这里来查。
    if (k != impl_->cols.size()) {
        *err = "列写入数与建列数不一致：写了 " + std::to_string(k) +
               " 列，应为 " + std::to_string(impl_->cols.size()) +
               " 列——X-macro 展开点被改动过";
        return false;
    }
    return true;

fail:
    *err = "写 HDF5 失败（磁盘满或文件损坏），已停止采集";
    return false;
}

size_t sampleColumnCount() {
    size_t c = 0;
#define X(name, h5type, tag, expr) ++c;
    ECJC_SAMPLE_COLUMNS(X)
#undef X
    return c;
}

std::vector<std::string> sampleColumnNames() {
    std::vector<std::string> v;
#define X(name, h5type, tag, expr) v.push_back(#name);
    ECJC_SAMPLE_COLUMNS(X)
#undef X
    return v;
}

void DataLogger::threadMain() {
    auto last_flush = std::chrono::steady_clock::now();

    while (!quit_.load()) {
        if (!active_.load()) {
            // 不采集时也要把 ring 排空，否则 RT 侧会一直计 dropped
            Sample tmp;
            while (ring_->pop(&tmp)) {}
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        const size_t n = ring_->popBatch(impl_->batch.data(), kBatch);
        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        } else {
            std::string err;
            if (!writeBatch(impl_->batch.data(), n, &err)) {
                active_.store(false);
                std::lock_guard<std::mutex> lk(st_mu_);
                st_.active = false;
                continue;
            }
            std::lock_guard<std::mutex> lk(st_mu_);
            st_.samples += n;
            st_.elapsed_s = st_.samples / (meta_.sampling_hz > 0 ? meta_.sampling_hz : 1000.0);
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_flush > std::chrono::seconds(30)) {
            // 30 秒一次 flush：兼顾崩溃鲁棒性与写放大。
            // 不每批 flush —— 那会让 10 小时实验的磁盘 IO 翻好几倍。
            if (impl_->file >= 0) H5Fflush(impl_->file, H5F_SCOPE_GLOBAL);
            last_flush = now;

            std::lock_guard<std::mutex> lk(st_mu_);
            st_.disk_free_gb = diskFreeGb(cfg_.app.data_dir);
            std::error_code ec;
            st_.bytes = fs::exists(cur_path_) ? fs::file_size(cur_path_, ec) : 0;
            if (st_.disk_free_gb >= 0 && st_.disk_free_gb < min_free_gb_) {
                active_.store(false);
                st_.active = false;
            }
        }
    }
}

}  // namespace ecjc
