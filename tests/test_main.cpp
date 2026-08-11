#include "test_framework.hpp"

namespace tf {

std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}

static int g_failures = 0;
int failures() { return g_failures; }

void fail(const std::string& file, int line, const std::string& msg) {
    ++g_failures;
    fprintf(stderr, "  [FAIL] %s:%d  %s\n", file.c_str(), line, msg.c_str());
}

}  // namespace tf

int main() {
    int failed_cases = 0;
    for (auto& c : tf::registry()) {
        const int before = tf::failures();
        printf("[RUN ] %s\n", c.name.c_str());
        c.fn();
        if (tf::failures() > before) { printf("[FAIL] %s\n", c.name.c_str()); ++failed_cases; }
        else printf("[ OK ] %s\n", c.name.c_str());
    }
    printf("\n%zu 个用例, %d 个失败\n", tf::registry().size(), failed_cases);
    return failed_cases == 0 ? 0 : 1;
}
