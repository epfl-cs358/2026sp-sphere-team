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

BalanceTelemetry makeSnap(uint32_t seq, uint32_t flags = 0) {
    BalanceTelemetry s{};
    s.seq = seq;
    s.event_flags = flags;
    return s;
}

}  // namespace

void setUp() {
    BalanceTelemetryWs::resetForTesting();
}

void tearDown() {}

void test_publish_to_full_queue_increments_drop_count() {
    // Publish many snapshots without draining. Whatever queue depth the
    // implementation picks, sustained over-publish must register drops.
    constexpr uint32_t kBlast = 2048;
    for (uint32_t i = 0; i < kBlast; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i + 1));
    }
    // Some drops must have occurred — we don't pin the exact queue depth,
    // only that not all 2048 fit, which is true for any reasonable choice.
    TEST_ASSERT_GREATER_THAN_UINT32(0, BalanceTelemetryWs::dropCount());
}

void test_snapshot_recent_returns_last_n() {
    for (uint32_t i = 1; i <= 10; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i));
        BalanceTelemetryWs::pumpOnce();
    }
    BalanceTelemetry buf[5]{};
    std::size_t n = BalanceTelemetryWs::snapshotRecent(buf, 5);
    TEST_ASSERT_EQUAL_UINT32(5, n);
    for (std::size_t i = 0; i < 5; ++i) {
        TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(6 + i), buf[i].seq);
    }
}

void test_snapshot_recent_caps_at_max_n() {
    for (uint32_t i = 1; i <= 5; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i));
        BalanceTelemetryWs::pumpOnce();
    }
    BalanceTelemetry buf[100]{};
    std::size_t n = BalanceTelemetryWs::snapshotRecent(buf, 100);
    TEST_ASSERT_EQUAL_UINT32(5, n);
    for (std::size_t i = 0; i < 5; ++i) {
        TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(i + 1), buf[i].seq);
    }
}

void test_event_counter_ticks_on_matching_bit() {
    BalanceTelemetryWs::publish(
        makeSnap(1, kEvent_ARMED_EDGE | kEvent_GAIN_CHANGED));
    BalanceTelemetryWs::pumpOnce();
    TEST_ASSERT_EQUAL_UINT32(1, BalanceTelemetryWs::eventCount(0));   // ARMED_EDGE
    TEST_ASSERT_EQUAL_UINT32(1, BalanceTelemetryWs::eventCount(12));  // GAIN_CHANGED
    TEST_ASSERT_EQUAL_UINT32(0, BalanceTelemetryWs::eventCount(5));   // FAULT_EXIT
}

void test_event_counter_accumulates() {
    for (int i = 0; i < 3; ++i) {
        BalanceTelemetryWs::publish(makeSnap(static_cast<uint32_t>(i + 1),
                                             kEvent_FAULT_ENTER));
        BalanceTelemetryWs::pumpOnce();
    }
    TEST_ASSERT_EQUAL_UINT32(3, BalanceTelemetryWs::eventCount(4));  // FAULT_ENTER
}

void test_ring_wraps_after_kRingSize() {
    constexpr uint32_t kTotal =
        static_cast<uint32_t>(BalanceTelemetryWs::kRingSize) + 10;
    for (uint32_t i = 1; i <= kTotal; ++i) {
        BalanceTelemetryWs::publish(makeSnap(i));
        BalanceTelemetryWs::pumpOnce();
    }
    BalanceTelemetry buf[10]{};
    std::size_t n = BalanceTelemetryWs::snapshotRecent(buf, 10);
    TEST_ASSERT_EQUAL_UINT32(10, n);
    for (std::size_t i = 0; i < 10; ++i) {
        TEST_ASSERT_EQUAL_UINT32(kTotal - 9 + static_cast<uint32_t>(i),
                                 buf[i].seq);
    }
    TEST_ASSERT_EQUAL_UINT32(kTotal, BalanceTelemetryWs::writeIdx());
}

void test_header_line_matches_canonical_columns() {
    const char* h = BalanceTelemetryWs::headerLine();
    TEST_ASSERT_NOT_NULL(h);
    // 83 comma-separated names → 82 commas.
    std::size_t commas = 0;
    for (const char* p = h; *p; ++p) {
        if (*p == ',') ++commas;
    }
    TEST_ASSERT_EQUAL_UINT32(82, commas);
    // Spot-check a few field names to catch column-order drift.
    TEST_ASSERT_TRUE(std::strstr(h, "seq,t_us,dt_measured,dt_used") == h);
    TEST_ASSERT_TRUE(std::strstr(h, "gyro_pitch_sign,gyro_roll_sign") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(h, "event_flags") != nullptr);
}

void test_init_is_idempotent() {
    AsyncWebServer server(81);
    BalanceTelemetryWs::init(server);
    BalanceTelemetryWs::init(server);
    BalanceTelemetryWs::init(server);
    // No assertion needed — must not crash, must not double-spawn anything.
    // We sanity-check that publish/pump still works after repeated init.
    BalanceTelemetryWs::publish(makeSnap(42));
    BalanceTelemetryWs::pumpOnce();
    BalanceTelemetry buf[1]{};
    TEST_ASSERT_EQUAL_UINT32(1, BalanceTelemetryWs::snapshotRecent(buf, 1));
    TEST_ASSERT_EQUAL_UINT32(42, buf[0].seq);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_publish_to_full_queue_increments_drop_count);
    RUN_TEST(test_snapshot_recent_returns_last_n);
    RUN_TEST(test_snapshot_recent_caps_at_max_n);
    RUN_TEST(test_event_counter_ticks_on_matching_bit);
    RUN_TEST(test_event_counter_accumulates);
    RUN_TEST(test_ring_wraps_after_kRingSize);
    RUN_TEST(test_header_line_matches_canonical_columns);
    RUN_TEST(test_init_is_idempotent);
    return UNITY_END();
}
