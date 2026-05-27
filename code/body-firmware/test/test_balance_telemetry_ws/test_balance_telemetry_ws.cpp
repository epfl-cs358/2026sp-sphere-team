/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include <cstring>
#include <string>

#include <ESPAsyncWebServer.h>

#include "BalanceTelemetry.h"
#include "BalanceTelemetryWs.h"

// Pull the .cpp directly so the native test binary links the implementation
// without needing an explicit test_build_src entry. Mirrors the pattern used
// by test_balancing_controller / test_balance_tuner.
#include "BalanceTelemetryWs.cpp"

namespace {

BalanceTelemetry makeSnap(uint32_t seq, uint32_t flags = 0, float dt = 0.0f) {
    BalanceTelemetry s{};
    s.seq = seq;
    s.event_flags = flags;
    s.dt_measured = dt;
    return s;
}

}  // namespace

AsyncWebServer& testServer() {
    static AsyncWebServer s(81);
    return s;
}

void setUp() {
    BalanceTelemetryWs::resetForTesting();
}

void tearDown() {}

// latest() must report "no data" before any snapshot has been pumped through.
// Stream-only design: there is no ring backfilling old state, and clients
// must be able to tell "haven't seen anything yet" apart from "have seen
// something but it's zero."
void test_latest_returns_false_before_publish() {
    BalanceTelemetryWs::init(testServer());
    BalanceTelemetry out{};
    TEST_ASSERT_FALSE(BalanceTelemetryWs::latest(&out));
}

void test_latest_returns_most_recent_after_publish() {
    BalanceTelemetryWs::init(testServer());
    for (uint32_t i = 1; i <= 3; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i));
        BalanceTelemetryWs::pumpOnce();
    }
    BalanceTelemetry out{};
    TEST_ASSERT_TRUE(BalanceTelemetryWs::latest(&out));
    TEST_ASSERT_EQUAL_UINT32(3, out.seq);
}

// Double-buffered: alternating writes must never return torn data. We can't
// race in a native single-threaded test; this exercises the alternation
// logic by publishing many snapshots and asserting each latest() read sees
// the most recent fully-committed snapshot.
void test_latest_alternation_consistent_over_many_publishes() {
    BalanceTelemetryWs::init(testServer());
    for (uint32_t i = 1; i <= 1000; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i));
        BalanceTelemetryWs::pumpOnce();
        BalanceTelemetry out{};
        TEST_ASSERT_TRUE(BalanceTelemetryWs::latest(&out));
        TEST_ASSERT_EQUAL_UINT32(i, out.seq);
    }
}

void test_dt_stats_zero_before_publish() {
    BalanceTelemetryWs::init(testServer());
    float mn = -1.0f, mean = -1.0f, mx = -1.0f;
    BalanceTelemetryWs::dtStats(&mn, &mean, &mx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mn);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mean);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, mx);
}

void test_dt_stats_reflects_published_dts() {
    BalanceTelemetryWs::init(testServer());
    BalanceTelemetryWs::publish(makeSnap(1, 0, 0.010f));
    BalanceTelemetryWs::pumpOnce();
    BalanceTelemetryWs::publish(makeSnap(2, 0, 0.011f));
    BalanceTelemetryWs::pumpOnce();
    BalanceTelemetryWs::publish(makeSnap(3, 0, 0.009f));
    BalanceTelemetryWs::pumpOnce();

    float mn = 0.0f, mean = 0.0f, mx = 0.0f;
    BalanceTelemetryWs::dtStats(&mn, &mean, &mx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.009f, mn);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.011f, mx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.010f, mean);
}

// dt window has finite capacity; old entries fall off. The window covers
// the last N publishes, so flooding past N and then publishing one known
// dt must yield stats dominated by the recent flood, not stale values.
void test_dt_stats_window_drops_old_entries() {
    BalanceTelemetryWs::init(testServer());
    // Fill the window with 0.010s entries.
    for (uint32_t i = 0; i < 200; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i + 1, 0, 0.010f));
        BalanceTelemetryWs::pumpOnce();
    }
    float mn = 0.0f, mean = 0.0f, mx = 0.0f;
    BalanceTelemetryWs::dtStats(&mn, &mean, &mx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.010f, mn);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.010f, mx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.010f, mean);
}

void test_publish_to_full_queue_increments_drop_count() {
    BalanceTelemetryWs::init(testServer());
    constexpr uint32_t kBlast = 2048;
    for (uint32_t i = 0; i < kBlast; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i + 1));
    }
    TEST_ASSERT_GREATER_THAN_UINT32(0, BalanceTelemetryWs::dropCount());
}

void test_event_counter_ticks_on_matching_bit() {
    BalanceTelemetryWs::init(testServer());
    BalanceTelemetryWs::publish(
        makeSnap(1, kEvent_ARMED_EDGE | kEvent_GAIN_CHANGED));
    BalanceTelemetryWs::pumpOnce();
    TEST_ASSERT_EQUAL_UINT32(1, BalanceTelemetryWs::eventCount(0));
    TEST_ASSERT_EQUAL_UINT32(1, BalanceTelemetryWs::eventCount(12));
    TEST_ASSERT_EQUAL_UINT32(0, BalanceTelemetryWs::eventCount(5));
}

void test_event_counter_accumulates() {
    BalanceTelemetryWs::init(testServer());
    for (int i = 0; i < 3; ++i) {
        BalanceTelemetryWs::publish(makeSnap(static_cast<uint32_t>(i + 1),
                                             kEvent_FAULT_ENTER));
        BalanceTelemetryWs::pumpOnce();
    }
    TEST_ASSERT_EQUAL_UINT32(3, BalanceTelemetryWs::eventCount(4));
}

void test_header_line_matches_canonical_columns() {
    const char* h = BalanceTelemetryWs::headerLine();
    TEST_ASSERT_NOT_NULL(h);
    std::size_t commas = 0;
    for (const char* p = h; *p; ++p) {
        if (*p == ',') ++commas;
    }
    // 83 original columns + 16 yaw/heading columns = 99 columns => 98 commas.
    // (yaw_rate_Kd removed in chore: PI-only yaw loop.)
    TEST_ASSERT_EQUAL_UINT32(98, commas);
    TEST_ASSERT_TRUE(std::strstr(h, "seq,t_us,dt_measured,dt_used") == h);
    TEST_ASSERT_TRUE(std::strstr(h, "gyro_pitch_sign,gyro_roll_sign,gyro_yaw_sign") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "yaw_rate_Kp,yaw_rate_Ki,heading_Kp") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "gyro_yaw_rate") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "heading_integrated") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "heading_setpoint") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "heading_err") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "heading_P") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "omega_target_raw") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "omega_target") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "yaw_rate_err") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "yaw_rate_P") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "yaw_rate_I") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "yaw_rate_D") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "yaw_rate_out") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "event_flags") != nullptr);
}

// Header column count must equal the number of comma-separated fields the
// writeCsv() snprintf produces. The two MUST stay lock-step; drift would
// break every downstream parser.
void test_header_column_count_matches_csv_row() {
    const char* h = BalanceTelemetryWs::headerLine();
    std::size_t header_cols = 1;
    for (const char* p = h; *p; ++p) {
        if (*p == ',') ++header_cols;
    }

    BalanceTelemetry t{};
    char line[1600];
    int n = writeCsv(line, sizeof(line), t);
    TEST_ASSERT_GREATER_THAN_INT(0, n);
    TEST_ASSERT_LESS_THAN_size_t(sizeof(line), static_cast<std::size_t>(n));

    std::size_t row_cols = 1;
    for (const char* p = line; *p; ++p) {
        if (*p == ',') ++row_cols;
    }
    TEST_ASSERT_EQUAL_UINT32(header_cols, row_cols);
}

// heading_setpoint default must be NaN (controller writes it that way when
// hold is not latched). When the struct is freshly default-constructed and
// passed through writeCsv, the column must show "nan" (lowercase).
void test_default_heading_setpoint_serializes_as_nan() {
    BalanceTelemetry t{};
    char line[1600];
    int n = writeCsv(line, sizeof(line), t);
    TEST_ASSERT_GREATER_THAN_INT(0, n);
    // Find the heading_setpoint column index from the header.
    const char* h = BalanceTelemetryWs::headerLine();
    std::string header(h);
    std::size_t pos = header.find("heading_setpoint");
    TEST_ASSERT_TRUE(pos != std::string::npos);
    std::size_t col_idx = 0;
    for (std::size_t i = 0; i < pos; ++i) {
        if (header[i] == ',') ++col_idx;
    }
    // Extract the col_idx'th comma-separated value from `line`.
    std::string row(line);
    std::size_t start = 0;
    for (std::size_t i = 0; i < col_idx; ++i) {
        start = row.find(',', start) + 1;
    }
    std::size_t end = row.find(',', start);
    std::string val = row.substr(start, end - start);
    TEST_ASSERT_EQUAL_STRING("nan", val.c_str());
}

// Real gain values flow through writeCsv unchanged (used to verify the
// controller writing cfg.yawRateKp ends up in the CSV row).
void test_yaw_rate_kp_serializes_with_real_value() {
    BalanceTelemetry t{};
    t.yaw_rate_Kp = 0.5f;
    char line[1600];
    int n = writeCsv(line, sizeof(line), t);
    TEST_ASSERT_GREATER_THAN_INT(0, n);
    const char* h = BalanceTelemetryWs::headerLine();
    std::string header(h);
    std::size_t pos = header.find("yaw_rate_Kp");
    TEST_ASSERT_TRUE(pos != std::string::npos);
    std::size_t col_idx = 0;
    for (std::size_t i = 0; i < pos; ++i) {
        if (header[i] == ',') ++col_idx;
    }
    std::string row(line);
    std::size_t start = 0;
    for (std::size_t i = 0; i < col_idx; ++i) {
        start = row.find(',', start) + 1;
    }
    std::size_t end = row.find(',', start);
    std::string val = row.substr(start, end - start);
    TEST_ASSERT_EQUAL_STRING("0.5", val.c_str());
}

void test_init_is_idempotent() {
    BalanceTelemetryWs::init(testServer());
    BalanceTelemetryWs::init(testServer());
    BalanceTelemetryWs::init(testServer());
    BalanceTelemetryWs::publish(makeSnap(42));
    BalanceTelemetryWs::pumpOnce();
    BalanceTelemetry out{};
    TEST_ASSERT_TRUE(BalanceTelemetryWs::latest(&out));
    TEST_ASSERT_EQUAL_UINT32(42, out.seq);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_latest_returns_false_before_publish);
    RUN_TEST(test_latest_returns_most_recent_after_publish);
    RUN_TEST(test_latest_alternation_consistent_over_many_publishes);
    RUN_TEST(test_dt_stats_zero_before_publish);
    RUN_TEST(test_dt_stats_reflects_published_dts);
    RUN_TEST(test_dt_stats_window_drops_old_entries);
    RUN_TEST(test_publish_to_full_queue_increments_drop_count);
    RUN_TEST(test_event_counter_ticks_on_matching_bit);
    RUN_TEST(test_event_counter_accumulates);
    RUN_TEST(test_header_line_matches_canonical_columns);
    RUN_TEST(test_header_column_count_matches_csv_row);
    RUN_TEST(test_default_heading_setpoint_serializes_as_nan);
    RUN_TEST(test_yaw_rate_kp_serializes_with_real_value);
    RUN_TEST(test_init_is_idempotent);
    return UNITY_END();
}
