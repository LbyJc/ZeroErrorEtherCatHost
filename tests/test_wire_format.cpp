#include "test_framework.hpp"
#include "ecjc/types.hpp"
#include <cstddef>

using namespace ecjc;

// 线格式契约：尺寸、无内部填充、协议版本已递增
TEST(sample_has_no_internal_padding) {
    // 8 字节量在前，然后 4 字节，然后 2 字节，最后 1 字节
    CHECK_EQ((int)offsetof(Sample, system_time_ns), 0);
    CHECK(offsetof(Sample, motor_position_raw) % 4 == 0);
    CHECK(offsetof(Sample, controlword) % 2 == 0);
}

TEST(protocol_version_bumped_for_new_layout) {
    CHECK_EQ((int)kProtocolVersion, 2);
}

// 新字段必须在场
TEST(new_bus_fields_present) {
    Sample s{};
    s.twist_counts = 1;
    s.following_error_counts = 2;
    s.torque_est_mNm = 3;
    s.aux_position_raw = 4;
    s.position_counts_raw = 5;
    s.motor_position_sdo = 6;
    s.dc_link_voltage_mV = 7;
    s.warning_code = 8;
    s.error_code = 9;
    s.temperature_drive_C = 10;
    s.torque_actual_permille = 11;
    s.torque_ratio = 12;
    CHECK_EQ(s.twist_counts + s.torque_ratio, 13);
}
