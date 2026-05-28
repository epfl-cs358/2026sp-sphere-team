/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <fff.h>
DEFINE_FFF_GLOBALS;

#include <unity.h>

#include "BalanceConfig.h"
#include "Preferences.h"
#include "RobotConstants.h"

// FFF fake bodies for the Preferences shim, used to assert that
// `balance save` reaches NVS.
DEFINE_FAKE_VALUE_FUNC(bool, prefs_begin, const char*, bool);
DEFINE_FAKE_VALUE_FUNC(size_t, prefs_getBytes, const char*, void*, size_t);
DEFINE_FAKE_VALUE_FUNC(size_t, prefs_putBytes, const char*, const void*, size_t);
DEFINE_FAKE_VOID_FUNC(prefs_end);

#include "BalanceConfigStorage.h"
#include "BalanceConfigStorage.cpp"

#include "ArmingState.h"
#include "ArmingState.cpp"

#include "BalanceTelemetry.h"
#include "BalanceTuner.h"
#include "BalanceTuner.cpp"

// ---- output capture ------------------------------------------------------

static std::vector<std::string> g_captured;

static void capture_print(const String& s) {
    g_captured.push_back(std::string(s.c_str()));
}

static bool captured_contains(const char* needle) {
    for (const auto& line : g_captured) {
        if (line.find(needle) != std::string::npos) return true;
    }
    return false;
}

// ---- save-blob capture ---------------------------------------------------

static std::vector<uint8_t> g_put_blob;

static size_t custom_putBytes(const char* /*key*/, const void* value, size_t len) {
    g_put_blob.assign(static_cast<const uint8_t*>(value),
                      static_cast<const uint8_t*>(value) + len);
    return len;
}

// ---- unity hooks ---------------------------------------------------------

static BalanceTuner g_tuner;

void setUp() {
    RESET_FAKE(prefs_begin);
    RESET_FAKE(prefs_getBytes);
    RESET_FAKE(prefs_putBytes);
    RESET_FAKE(prefs_end);
    FFF_RESET_HISTORY();
    prefs_begin_fake.return_val = true;
    prefs_putBytes_fake.custom_fake = custom_putBytes;
    g_put_blob.clear();

    g_captured.clear();
    g_tuner.setPrint(&capture_print);
    g_tuner.begin(RobotConstants::balanceConfig());
    // `g_tuner` is file-scope; prior tests' mutations leave bits in
    // `_pendingEvents`. Flush so each test sees a fresh accumulator.
    (void)g_tuner.consumePending();
}

void tearDown() {}

// ---- tests ---------------------------------------------------------------

void test_show_dumps_current_config() {
    g_tuner.handle(String("balance show"));
    TEST_ASSERT_TRUE(captured_contains("pitchKp"));
    TEST_ASSERT_TRUE(captured_contains("rollKp"));
    TEST_ASSERT_TRUE(captured_contains("tiltPerVelocity"));
    TEST_ASSERT_TRUE(captured_contains("maxTiltSetpoint"));
    TEST_ASSERT_TRUE(captured_contains("envelopeEnterSin"));
    TEST_ASSERT_TRUE(captured_contains("envelopeExitSin"));
    TEST_ASSERT_TRUE(captured_contains("gyroPitchSign"));
    TEST_ASSERT_TRUE(captured_contains("gyroRollSign"));
}

void test_show_pids() {
    g_tuner.handle(String("balance show pids"));
    TEST_ASSERT_TRUE(captured_contains("pitchKp"));
    TEST_ASSERT_TRUE(captured_contains("pitchKi"));
    TEST_ASSERT_TRUE(captured_contains("pitchKd"));
    TEST_ASSERT_TRUE(captured_contains("rollKp"));
    TEST_ASSERT_TRUE(captured_contains("rollKi"));
    TEST_ASSERT_TRUE(captured_contains("rollKd"));
    // PIDs-only view must not include unrelated fields.
    TEST_ASSERT_FALSE(captured_contains("tiltPerVelocity"));
    TEST_ASSERT_FALSE(captured_contains("envelopeEnterSin"));
}

void test_set_valid_field_updates_slot() {
    g_tuner.handle(String("balance set pitchKp 2.5"));
    const BalanceConfig* live = g_tuner.slot().load();
    TEST_ASSERT_NOT_NULL(live);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.5f, live->pitchKp);

    g_captured.clear();
    g_tuner.handle(String("balance show"));
    TEST_ASSERT_TRUE(captured_contains("2.5"));
}

void test_set_atomic_swap_uses_spare_buffer() {
    const BalanceConfig* p0 = g_tuner.slot().load();
    TEST_ASSERT_NOT_NULL(p0);

    g_tuner.handle(String("balance set pitchKp 2.0"));
    const BalanceConfig* p1 = g_tuner.slot().load();
    TEST_ASSERT_NOT_NULL(p1);
    TEST_ASSERT_TRUE(p1 != p0);

    g_tuner.handle(String("balance set pitchKp 3.0"));
    const BalanceConfig* p2 = g_tuner.slot().load();
    TEST_ASSERT_NOT_NULL(p2);
    // After two swaps, the pointer flips back to the first buffer.
    TEST_ASSERT_TRUE(p2 == p0);

    g_tuner.handle(String("balance set pitchKp 4.0"));
    const BalanceConfig* p3 = g_tuner.slot().load();
    TEST_ASSERT_TRUE(p3 == p1);
}

void test_set_invalid_negative_kp_rejected() {
    const BalanceConfig* before = g_tuner.slot().load();
    float orig_kp = before->pitchKp;

    g_tuner.handle(String("balance set pitchKp -1.0"));

    const BalanceConfig* after = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, orig_kp, after->pitchKp);
    TEST_ASSERT_TRUE(captured_contains("error"));
}

void test_set_invalid_envelope_order_rejected() {
    // envelopeEnterSin defaults to 0.866. Try to push envelopeExitSin above it.
    const BalanceConfig* before = g_tuner.slot().load();
    float orig_exit = before->envelopeExitSin;

    g_tuner.handle(String("balance set envelopeExitSin 0.95"));

    const BalanceConfig* after = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, orig_exit, after->envelopeExitSin);
    TEST_ASSERT_TRUE(captured_contains("error"));
}

void test_set_max_tilt_above_fault_envelope_rejected() {
    const BalanceConfig* before = g_tuner.slot().load();
    float orig = before->maxTiltSetpoint;

    // 1.10 rad ≈ 63°; envelopeEnterSin defaults to 0.866 → asin ≈ 1.047 rad.
    g_tuner.handle(String("balance set maxTiltSetpoint 1.10"));

    const BalanceConfig* after = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, orig, after->maxTiltSetpoint);
    TEST_ASSERT_TRUE(captured_contains("error"));
}

void test_set_unknown_key_rejected() {
    const BalanceConfig* before = g_tuner.slot().load();

    g_tuner.handle(String("balance set foo 1.0"));

    const BalanceConfig* after = g_tuner.slot().load();
    // No swap should have occurred.
    TEST_ASSERT_TRUE(before == after);
    TEST_ASSERT_TRUE(captured_contains("error"));
}

void test_reset_reverts_to_defaults() {
    g_tuner.handle(String("balance set pitchKp 7.5"));
    g_tuner.handle(String("balance set rollKp 6.25"));

    g_tuner.handle(String("balance reset"));

    BalanceConfig defaults = RobotConstants::balanceConfig();
    const BalanceConfig* live = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, defaults.pitchKp,  live->pitchKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, defaults.rollKp,   live->rollKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, defaults.maxTiltSetpoint, live->maxTiltSetpoint);
}

void test_save_invokes_storage() {
    g_tuner.handle(String("balance set pitchKp 1.75"));

    g_tuner.handle(String("balance save"));

    TEST_ASSERT_EQUAL_UINT(1, prefs_putBytes_fake.call_count);
    TEST_ASSERT_EQUAL_UINT(sizeof(uint16_t) + sizeof(BalanceConfig), g_put_blob.size());

    BalanceConfig persisted;
    std::memcpy(&persisted, g_put_blob.data() + sizeof(uint16_t), sizeof(BalanceConfig));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.75f, persisted.pitchKp);
}

void test_status_prints_pitch_roll_armed() {
    g_tuner.handle(String("balance status"));
    TEST_ASSERT_TRUE(captured_contains("pitch"));
    TEST_ASSERT_TRUE(captured_contains("roll"));
    TEST_ASSERT_TRUE(captured_contains("armed"));
}

// ---- pending-events tests -----------------------------------------------

void test_consume_pending_empty_returns_zero() {
    TEST_ASSERT_EQUAL_UINT32(0u, g_tuner.consumePending());
}

void test_try_set_success_ors_gain_changed_bit() {
    String err;
    bool ok = g_tuner.trySet(String("pitchKp"), 1.5f, err);
    TEST_ASSERT_TRUE(ok);
    uint32_t first = g_tuner.consumePending();
    TEST_ASSERT_TRUE((first & kEvent_GAIN_CHANGED) != 0u);
    // Single-consume semantics.
    TEST_ASSERT_EQUAL_UINT32(0u, g_tuner.consumePending());
}

void test_try_set_failure_does_not_or_bit() {
    String err;
    bool ok = g_tuner.trySet(String("nonexistent_key"), 1.0f, err);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_EQUAL_UINT32(0u, g_tuner.consumePending());
}

void test_save_nvs_ors_config_saved_bit() {
    bool ok = g_tuner.saveNvs();
    TEST_ASSERT_TRUE(ok);
    uint32_t bits = g_tuner.consumePending();
    TEST_ASSERT_TRUE((bits & kEvent_CONFIG_SAVED) != 0u);
}

void test_reset_to_defaults_ors_config_reset_bit() {
    g_tuner.resetToDefaults();
    uint32_t bits = g_tuner.consumePending();
    TEST_ASSERT_TRUE((bits & kEvent_CONFIG_RESET) != 0u);
}

void test_try_set_yaw_kp_updates_live_config() {
    String err;
    bool ok = g_tuner.trySet(String("yawRateKp"), 0.5f, err);
    TEST_ASSERT_TRUE(ok);
    const BalanceConfig* live = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, live->yawRateKp);
}

void test_try_set_yaw_kp_negative_rejected() {
    const BalanceConfig* before = g_tuner.slot().load();
    float orig = before->yawRateKp;

    String err;
    bool ok = g_tuner.trySet(String("yawRateKp"), -1.0f, err);
    TEST_ASSERT_FALSE(ok);

    const BalanceConfig* after = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, orig, after->yawRateKp);
}

void test_try_set_heading_kp_negative_rejected() {
    String err;
    bool ok = g_tuner.trySet(String("headingKp"), -0.1f, err);
    TEST_ASSERT_FALSE(ok);
}

void test_try_set_gyro_yaw_sign_out_of_range_rejected() {
    String err;
    bool ok = g_tuner.trySet(String("gyroYawSign"), 2.0f, err);
    TEST_ASSERT_FALSE(ok);
}

void test_try_set_gyro_yaw_sign_negative_one_accepted() {
    String err;
    bool ok = g_tuner.trySet(String("gyroYawSign"), -1.0f, err);
    TEST_ASSERT_TRUE(ok);
    const BalanceConfig* live = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, live->gyroYawSign);
}

void test_try_set_gyro_pitch_sign_out_of_range_rejected() {
    String err;
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroPitchSign"),  0.0f, err));
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroPitchSign"),  2.0f, err));
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroPitchSign"), -0.5f, err));
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroPitchSign"),
                                     std::numeric_limits<float>::quiet_NaN(), err));
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroPitchSign"),
                                     std::numeric_limits<float>::infinity(), err));
}

void test_try_set_gyro_pitch_sign_minus_one_accepted() {
    String err;
    bool ok = g_tuner.trySet(String("gyroPitchSign"), -1.0f, err);
    TEST_ASSERT_TRUE(ok);
    const BalanceConfig* live = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, live->gyroPitchSign);
}

void test_try_set_gyro_roll_sign_out_of_range_rejected() {
    String err;
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroRollSign"),  0.0f, err));
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroRollSign"),  2.0f, err));
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroRollSign"), -0.5f, err));
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroRollSign"),
                                     std::numeric_limits<float>::quiet_NaN(), err));
    TEST_ASSERT_FALSE(g_tuner.trySet(String("gyroRollSign"),
                                     std::numeric_limits<float>::infinity(), err));
}

void test_try_set_gyro_roll_sign_minus_one_accepted() {
    String err;
    bool ok = g_tuner.trySet(String("gyroRollSign"), -1.0f, err);
    TEST_ASSERT_TRUE(ok);
    const BalanceConfig* live = g_tuner.slot().load();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.0f, live->gyroRollSign);
}

void test_show_includes_new_yaw_keys() {
    g_tuner.handle(String("balance show"));
    TEST_ASSERT_TRUE(captured_contains("yawRateKp"));
    TEST_ASSERT_TRUE(captured_contains("yawRateKi"));
    TEST_ASSERT_FALSE(captured_contains("yawRateKd"));
    TEST_ASSERT_TRUE(captured_contains("headingKp"));
    TEST_ASSERT_TRUE(captured_contains("gyroYawSign"));
}

void test_show_pids_includes_yaw_gains_not_sign() {
    g_tuner.handle(String("balance show pids"));
    TEST_ASSERT_TRUE(captured_contains("yawRateKp"));
    TEST_ASSERT_TRUE(captured_contains("yawRateKi"));
    TEST_ASSERT_FALSE(captured_contains("yawRateKd"));
    TEST_ASSERT_TRUE(captured_contains("headingKp"));
    TEST_ASSERT_FALSE(captured_contains("gyroYawSign"));
}

// yawRateKd was removed (PI-only yaw loop; D-term would numerically
// differentiate the gyro signal). Setting it via the tuner must return
// "unknown key" so operators don't silently write to a stale field.
void test_try_set_yaw_rate_kd_rejected_as_unknown() {
    String err;
    bool ok = g_tuner.trySet(String("yawRateKd"), 0.5f, err);
    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_TRUE(std::string(err.c_str()).find("unknown") != std::string::npos);
}

void test_multiple_events_or_together() {
    String err;
    bool ok = g_tuner.trySet(String("pitchKp"), 2.25f, err);
    TEST_ASSERT_TRUE(ok);
    g_tuner.resetToDefaults();
    uint32_t bits = g_tuner.consumePending();
    TEST_ASSERT_TRUE((bits & kEvent_GAIN_CHANGED) != 0u);
    TEST_ASSERT_TRUE((bits & kEvent_CONFIG_RESET) != 0u);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_show_dumps_current_config);
    RUN_TEST(test_show_pids);
    RUN_TEST(test_set_valid_field_updates_slot);
    RUN_TEST(test_set_atomic_swap_uses_spare_buffer);
    RUN_TEST(test_set_invalid_negative_kp_rejected);
    RUN_TEST(test_set_invalid_envelope_order_rejected);
    RUN_TEST(test_set_max_tilt_above_fault_envelope_rejected);
    RUN_TEST(test_set_unknown_key_rejected);
    RUN_TEST(test_reset_reverts_to_defaults);
    RUN_TEST(test_save_invokes_storage);
    RUN_TEST(test_status_prints_pitch_roll_armed);
    RUN_TEST(test_consume_pending_empty_returns_zero);
    RUN_TEST(test_try_set_success_ors_gain_changed_bit);
    RUN_TEST(test_try_set_failure_does_not_or_bit);
    RUN_TEST(test_save_nvs_ors_config_saved_bit);
    RUN_TEST(test_reset_to_defaults_ors_config_reset_bit);
    RUN_TEST(test_try_set_yaw_kp_updates_live_config);
    RUN_TEST(test_try_set_yaw_kp_negative_rejected);
    RUN_TEST(test_try_set_heading_kp_negative_rejected);
    RUN_TEST(test_try_set_gyro_yaw_sign_out_of_range_rejected);
    RUN_TEST(test_try_set_gyro_yaw_sign_negative_one_accepted);
    RUN_TEST(test_try_set_gyro_pitch_sign_out_of_range_rejected);
    RUN_TEST(test_try_set_gyro_pitch_sign_minus_one_accepted);
    RUN_TEST(test_try_set_gyro_roll_sign_out_of_range_rejected);
    RUN_TEST(test_try_set_gyro_roll_sign_minus_one_accepted);
    RUN_TEST(test_show_includes_new_yaw_keys);
    RUN_TEST(test_show_pids_includes_yaw_gains_not_sign);
    RUN_TEST(test_try_set_yaw_rate_kd_rejected_as_unknown);
    RUN_TEST(test_multiple_events_or_together);
    return UNITY_END();
}
