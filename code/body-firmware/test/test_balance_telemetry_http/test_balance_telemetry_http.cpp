/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>

#include <cstdint>
#include <cstring>
#include <string>

#include <ESPAsyncWebServer.h>

#include "BalanceTelemetry.h"
#include "BalanceTelemetryWs.h"

// Pull both .cpp's directly so the native test binary links the
// implementations without needing explicit test_build_src entries. Mirrors
// the pattern used by test_balance_telemetry_ws.
#include "BalanceTelemetryWs.cpp"  // NOLINT(bugprone-suspicious-include)

#include "BalanceTelemetryHttpApi.h"
#include "BalanceTelemetryHttpApi.cpp"  // NOLINT(bugprone-suspicious-include)

namespace {

BalanceTelemetry makeSnap(uint32_t seq,
                          uint32_t event_flags = 0,
                          float dt_measured = 0.01f) {
    BalanceTelemetry s{};
    s.seq = seq;
    s.event_flags = event_flags;
    s.dt_measured = dt_measured;
    return s;
}

// Count non-overlapping occurrences of needle in haystack.
std::size_t countSubstr(const std::string& haystack, const std::string& needle) {
    if (needle.empty()) return 0;
    std::size_t n = 0;
    std::size_t pos = 0;
    while ((pos = haystack.find(needle, pos)) != std::string::npos) {
        ++n;
        pos += needle.size();
    }
    return n;
}

// Canonical 83-column CSV header — duplicated here (independent witness)
// so a drift in BalanceTelemetryHttpApi.cpp's header is caught structurally.
constexpr const char* kExpectedHeader =
    "seq,t_us,dt_measured,dt_used,"
    "cmd_vx_raw,cmd_vy_raw,cmd_omega_raw,"
    "cmd_vx,cmd_vy,cmd_omega,cmd_age_ms,"
    "quat_w,quat_x,quat_y,quat_z,"
    "accel_x,accel_y,accel_z,"
    "gyro_x_raw,gyro_y_raw,gyro_z_raw,"
    "gx,gy,gz,tilt_mag_sin,"
    "pitch_actual,roll_actual,"
    "gyro_pitch_rate,gyro_roll_rate,"
    "pitch_target,roll_target,"
    "pitch_err,pitch_P,pitch_I,pitch_D,pitch_out_raw,pitch_out,"
    "roll_err,roll_P,roll_I,roll_D,roll_out_raw,roll_out,"
    "body_vx_cmd,body_vy_cmd,body_omega_cmd,"
    "wheel_target_rpm_0,wheel_target_rpm_1,wheel_target_rpm_2,"
    "wheel_meas_rpm_0,wheel_meas_rpm_1,wheel_meas_rpm_2,"
    "wheel_P_0,wheel_P_1,wheel_P_2,"
    "wheel_I_0,wheel_I_1,wheel_I_2,"
    "wheel_D_0,wheel_D_1,wheel_D_2,"
    "wheel_out_0,wheel_out_1,wheel_out_2,"
    "pitch_Kp,pitch_Ki,pitch_Kd,"
    "roll_Kp,roll_Ki,roll_Kd,"
    "pitch_deadband,roll_deadband,"
    "max_output_velocity,"
    "envelope_enter_sin,envelope_exit_sin,"
    "gyro_pitch_sign,gyro_roll_sign,"
    "tilt_per_velocity,max_tilt_setpoint,"
    "armed_state,in_fault,cmd_stale,"
    "event_flags";

// The 83 column names, in order. Used to assert /telemetry/latest names
// every field and /telemetry/schema lists every column.
constexpr const char* kAllColumns[] = {
    "seq", "t_us", "dt_measured", "dt_used",
    "cmd_vx_raw", "cmd_vy_raw", "cmd_omega_raw",
    "cmd_vx", "cmd_vy", "cmd_omega", "cmd_age_ms",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "accel_x", "accel_y", "accel_z",
    "gyro_x_raw", "gyro_y_raw", "gyro_z_raw",
    "gx", "gy", "gz", "tilt_mag_sin",
    "pitch_actual", "roll_actual",
    "gyro_pitch_rate", "gyro_roll_rate",
    "pitch_target", "roll_target",
    "pitch_err", "pitch_P", "pitch_I", "pitch_D", "pitch_out_raw", "pitch_out",
    "roll_err",  "roll_P",  "roll_I",  "roll_D",  "roll_out_raw",  "roll_out",
    "body_vx_cmd", "body_vy_cmd", "body_omega_cmd",
    "wheel_target_rpm_0", "wheel_target_rpm_1", "wheel_target_rpm_2",
    "wheel_meas_rpm_0", "wheel_meas_rpm_1", "wheel_meas_rpm_2",
    "wheel_P_0", "wheel_P_1", "wheel_P_2",
    "wheel_I_0", "wheel_I_1", "wheel_I_2",
    "wheel_D_0", "wheel_D_1", "wheel_D_2",
    "wheel_out_0", "wheel_out_1", "wheel_out_2",
    "pitch_Kp", "pitch_Ki", "pitch_Kd",
    "roll_Kp",  "roll_Ki",  "roll_Kd",
    "pitch_deadband", "roll_deadband",
    "max_output_velocity",
    "envelope_enter_sin", "envelope_exit_sin",
    "gyro_pitch_sign", "gyro_roll_sign",
    "tilt_per_velocity", "max_tilt_setpoint",
    "armed_state", "in_fault", "cmd_stale",
    "event_flags",
};

constexpr std::size_t kAllColumnsCount =
    sizeof(kAllColumns) / sizeof(kAllColumns[0]);

constexpr const char* kAllEventNames[16] = {
    "ARMED_EDGE", "DISARMED_EDGE", "KILLED_EDGE", "KILL_CLEARED",
    "FAULT_ENTER", "FAULT_EXIT",
    "PITCH_DEADBAND_RESET", "ROLL_DEADBAND_RESET",
    "PITCH_I_SATURATED", "ROLL_I_SATURATED",
    "PITCH_OUT_SATURATED", "ROLL_OUT_SATURATED",
    "GAIN_CHANGED", "CONFIG_SAVED", "CONFIG_RESET", "STEP_INJECTED",
};

}  // namespace

void setUp() {
    // init() materialises the heap-backed ring on first call (idempotent
    // thereafter). Tests below publish + pumpOnce and read back via
    // snapshotRecent() — that path requires the ring to exist.
    static AsyncWebServer s(81);
    BalanceTelemetryWs::init(s);
    BalanceTelemetryWs::resetForTesting();
}

void tearDown() {}

// 1. /telemetry/latest mentions every column name as a JSON key.
void test_latest_json_includes_every_column() {
    BalanceTelemetryWs::publish(makeSnap(7, 0));
    BalanceTelemetryWs::pumpOnce();

    String s = BalanceTelemetryHttpApi::buildLatestJson();
    std::string body(s.c_str(), s.length());

    for (std::size_t i = 0; i < kAllColumnsCount; ++i) {
        std::string key = std::string("\"") + kAllColumns[i] + "\"";
        if (body.find(key) == std::string::npos) {
            char msg[128];
            std::snprintf(msg, sizeof(msg),
                          "latest JSON missing key: %s", kAllColumns[i]);
            TEST_FAIL_MESSAGE(msg);
        }
    }
}

// 2. /telemetry/recent?n=N returns oldest-to-newest, last N entries.
void test_recent_json_returns_n_snapshots_oldest_to_newest() {
    for (uint32_t i = 1; i <= 10; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i));
        BalanceTelemetryWs::pumpOnce();
    }
    String s = BalanceTelemetryHttpApi::buildRecentJson(5);
    std::string body(s.c_str(), s.length());

    // Should be {"snapshots":[...]} with 5 elements.
    TEST_ASSERT_TRUE(body.find("\"snapshots\"") != std::string::npos);

    // First element seq=6, last seq=10. We assert by checking the first
    // occurrence of "seq":6 precedes any later seq, and that "seq":10
    // appears after "seq":9 etc.
    auto p6 = body.find("\"seq\":6");
    auto p7 = body.find("\"seq\":7");
    auto p8 = body.find("\"seq\":8");
    auto p9 = body.find("\"seq\":9");
    auto p10 = body.find("\"seq\":10");
    TEST_ASSERT_TRUE(p6 != std::string::npos);
    TEST_ASSERT_TRUE(p7 != std::string::npos);
    TEST_ASSERT_TRUE(p8 != std::string::npos);
    TEST_ASSERT_TRUE(p9 != std::string::npos);
    TEST_ASSERT_TRUE(p10 != std::string::npos);
    TEST_ASSERT_TRUE(p6 < p7);
    TEST_ASSERT_TRUE(p7 < p8);
    TEST_ASSERT_TRUE(p8 < p9);
    TEST_ASSERT_TRUE(p9 < p10);

    // None of seq=1..5 should appear (they were superseded).
    TEST_ASSERT_TRUE(body.find("\"seq\":1,") == std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"seq\":5,") == std::string::npos);
}

// 3. /telemetry/recent?n=N is bounded by the ring size.
void test_recent_json_caps_at_ring_size() {
    constexpr uint32_t kTotal =
        static_cast<uint32_t>(BalanceTelemetryWs::kRingSize) + 88;
    for (uint32_t i = 1; i <= kTotal; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i));
        BalanceTelemetryWs::pumpOnce();
    }
    String s = BalanceTelemetryHttpApi::buildRecentJson(1000);
    std::string body(s.c_str(), s.length());

    // Count "seq": occurrences = number of snapshot objects.
    std::size_t snaps = countSubstr(body, "\"seq\":");
    TEST_ASSERT_TRUE(snaps <= BalanceTelemetryWs::kRingSize);
    TEST_ASSERT_TRUE(snaps > 0);
}

// 4. /telemetry/stats shape — top-level keys and inner dt_ms keys present.
void test_stats_json_shape() {
    BalanceTelemetryWs::publish(makeSnap(1, 0, 0.010f));
    BalanceTelemetryWs::pumpOnce();
    BalanceTelemetryWs::publish(makeSnap(2, 0, 0.011f));
    BalanceTelemetryWs::pumpOnce();

    String s = BalanceTelemetryHttpApi::buildStatsJson();
    std::string body(s.c_str(), s.length());

    TEST_ASSERT_TRUE(body.find("\"seq\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"uptime_s\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"dt_ms\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"in_fault_count\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"events_since_boot\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"ws_drops\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"ring_overruns\"") != std::string::npos);

    // dt_ms inner shape.
    TEST_ASSERT_TRUE(body.find("\"min\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"mean\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"max\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"jitter\"") != std::string::npos);

    // All 16 event names.
    for (std::size_t i = 0; i < 16; ++i) {
        std::string key = std::string("\"") + kAllEventNames[i] + "\"";
        if (body.find(key) == std::string::npos) {
            char msg[128];
            std::snprintf(msg, sizeof(msg),
                          "stats events_since_boot missing: %s",
                          kAllEventNames[i]);
            TEST_FAIL_MESSAGE(msg);
        }
    }
}

// 5. Event-bit publish reflects in stats.events_since_boot counts.
void test_stats_event_counts_reflect_published() {
    BalanceTelemetryWs::publish(makeSnap(1, kEvent_FAULT_ENTER));
    BalanceTelemetryWs::pumpOnce();
    BalanceTelemetryWs::publish(makeSnap(2, kEvent_FAULT_ENTER));
    BalanceTelemetryWs::pumpOnce();
    BalanceTelemetryWs::publish(makeSnap(3, kEvent_ARMED_EDGE));
    BalanceTelemetryWs::pumpOnce();

    String s = BalanceTelemetryHttpApi::buildStatsJson();
    std::string body(s.c_str(), s.length());

    TEST_ASSERT_TRUE(body.find("\"FAULT_ENTER\":2") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"ARMED_EDGE\":1") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"in_fault_count\":2") != std::string::npos);
}

// 6. /telemetry/schema lists all 83 columns with name + desc.
void test_schema_json_lists_all_columns() {
    String s = BalanceTelemetryHttpApi::buildSchemaJson();
    std::string body(s.c_str(), s.length());

    TEST_ASSERT_TRUE(body.find("\"columns\"") != std::string::npos);
    TEST_ASSERT_TRUE(body.find("\"event_bits\"") != std::string::npos);

    // Every column name appears (as a "name":"..." entry).
    for (std::size_t i = 0; i < kAllColumnsCount; ++i) {
        std::string key = std::string("\"name\":\"") + kAllColumns[i] + "\"";
        if (body.find(key) == std::string::npos) {
            char msg[160];
            std::snprintf(msg, sizeof(msg),
                          "schema missing column: %s", kAllColumns[i]);
            TEST_FAIL_MESSAGE(msg);
        }
    }
    // 83 desc entries (one per column).
    std::size_t descs = countSubstr(body, "\"desc\"");
    TEST_ASSERT_EQUAL_UINT32(83, descs);

    // All 16 event_bits decoded.
    for (std::size_t i = 0; i < 16; ++i) {
        std::string key = std::string("\"") + kAllEventNames[i] + "\"";
        if (body.find(key) == std::string::npos) {
            char msg[160];
            std::snprintf(msg, sizeof(msg),
                          "schema event_bits missing: %s", kAllEventNames[i]);
            TEST_FAIL_MESSAGE(msg);
        }
    }
}

// 7. /telemetry/header returns the exact canonical CSV header.
void test_header_text_matches_csv_order() {
    String s = BalanceTelemetryHttpApi::buildHeader();
    std::string body(s.c_str(), s.length());

    TEST_ASSERT_EQUAL_STRING(kExpectedHeader, body.c_str());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_latest_json_includes_every_column);
    RUN_TEST(test_recent_json_returns_n_snapshots_oldest_to_newest);
    RUN_TEST(test_recent_json_caps_at_ring_size);
    RUN_TEST(test_stats_json_shape);
    RUN_TEST(test_stats_event_counts_reflect_published);
    RUN_TEST(test_schema_json_lists_all_columns);
    RUN_TEST(test_header_text_matches_csv_order);
    return UNITY_END();
}
