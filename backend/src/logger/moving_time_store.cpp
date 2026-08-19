#include "ecjc/moving_time_store.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <unistd.h>

namespace ecjc {

namespace {

// mkdir -p：逐级建父目录。已存在不算错。
bool makeParents(const std::string& file_path, std::string* err) {
    const auto slash = file_path.rfind('/');
    if (slash == std::string::npos || slash == 0) return true;
    const std::string dir = file_path.substr(0, slash);
    std::string cur;
    for (size_t i = 0; i < dir.size(); ++i) {
        cur += dir[i];
        if (dir[i] != '/' && i + 1 < dir.size()) continue;
        if (cur == "/" || cur.empty()) continue;
        if (::mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) {
            *err = "mkdir " + cur + " 失败: " + std::strerror(errno);
            return false;
        }
    }
    if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
        *err = "mkdir " + dir + " 失败: " + std::strerror(errno);
        return false;
    }
    return true;
}

}  // namespace

int64_t MovingTimeStore::load() const {
    FILE* f = ::fopen(path_.c_str(), "r");
    if (!f) return 0;
    long long v = 0;
    const int n = ::fscanf(f, "%lld", &v);
    ::fclose(f);
    if (n != 1 || v < 0) return 0;
    return static_cast<int64_t>(v);
}

bool MovingTimeStore::save(int64_t total_ns, std::string* err) const {
    std::string e;
    if (!makeParents(path_, &e)) { if (err) *err = e; return false; }

    const std::string tmp = path_ + ".tmp";
    FILE* f = ::fopen(tmp.c_str(), "w");
    if (!f) {
        if (err) *err = "打开 " + tmp + " 失败: " + std::strerror(errno);
        return false;
    }
    ::fprintf(f, "%lld\n", static_cast<long long>(total_ns));
    ::fflush(f);
    ::fsync(::fileno(f));          // rename 原子性只保护目录项，内容要先落盘
    ::fclose(f);
    if (::rename(tmp.c_str(), path_.c_str()) != 0) {
        if (err) *err = "rename 到 " + path_ + " 失败: " + std::strerror(errno);
        ::unlink(tmp.c_str());
        return false;
    }
    return true;
}

}  // namespace ecjc
