/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

#include <fff.h>
DEFINE_FFF_GLOBALS;

#include <unity.h>
#include "BalanceConfig.h"
#include "Preferences.h"
#include "RobotConstants.h"

// Define the FFF fake bodies declared in the stub Preferences.h before
// pulling in the storage implementation under test.
DEFINE_FAKE_VALUE_FUNC(bool, prefs_begin, const char*, bool);
DEFINE_FAKE_VALUE_FUNC(size_t, prefs_getBytes, const char*, void*, size_t);
DEFINE_FAKE_VALUE_FUNC(size_t, prefs_putBytes, const char*, const void*, size_t);
DEFINE_FAKE_VOID_FUNC(prefs_end);

#include "BalanceConfigStorage.h"
#include "BalanceConfigStorage.cpp"

static_assert(std::is_trivially_copyable<BalanceConfig>::value,
              "BalanceConfig must remain trivially-copyable (POD)");

// Captured payload from the most recent prefs_putBytes call.
static std::vector<uint8_t> g_put_blob;
// Scripted payload returned by prefs_getBytes; size returned independently
// via g_get_return_size so tests can simulate size-mismatch failures.
static std::vector<uint8_t> g_get_blob;
static size_t g_get_return_size = 0;

static size_t custom_getBytes(const char* /*key*/, void* buf, size_t maxLen) {
    size_t copy_n = g_get_blob.size() < maxLen ? g_get_blob.size() : maxLen;
    if (copy_n > 0 && buf != nullptr) {
        std::memcpy(buf, g_get_blob.data(), copy_n);
    }
    return g_get_return_size;
}

static size_t custom_putBytes(const char* /*key*/, const void* value, size_t len) {
    g_put_blob.assign(static_cast<const uint8_t*>(value),
                      static_cast<const uint8_t*>(value) + len);
    return len;
}

void setUp() {
    RESET_PREFERENCES_FAKES();
    prefs_begin_fake.return_val = true;
    prefs_getBytes_fake.custom_fake = custom_getBytes;
    prefs_putBytes_fake.custom_fake = custom_putBytes;
    g_put_blob.clear();
    g_get_blob.clear();
    g_get_return_size = 0;
}

void tearDown() {}

static BalanceConfig makeNonDefaultConfig() {
    BalanceConfig cfg = RobotConstants::balanceConfig();
    cfg.tiltPerVelocity   = 0.42f;
    cfg.maxTiltSetpoint   = 0.31f;
    cfg.pitchKp = 2.25f; cfg.pitchKi = 0.05f; cfg.pitchKd = 0.18f;
    cfg.rollKp  = 1.95f; cfg.rollKi  = 0.04f; cfg.rollKd  = 0.16f;
    cfg.maxOutputVelocity = 0.85f;
    cfg.envelopeEnterSin  = 0.900f;
    cfg.envelopeExitSin   = 0.800f;
    cfg.gyroPitchSign     = -1.0f;
    cfg.gyroRollSign      =  1.0f;
    cfg.yawRateKp         = 0.75f;
    cfg.yawRateKi         = 0.02f;
    cfg.headingKp         = 1.25f;
    cfg.gyroYawSign       = -1.0f;
    return cfg;
}

void test_default_config_values_match_spec() {
    BalanceConfig cfg = RobotConstants::balanceConfig();

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.30f, cfg.tiltPerVelocity);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.35f, cfg.maxTiltSetpoint);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.50f, cfg.pitchKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,  cfg.pitchKi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.15f, cfg.pitchKd);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.50f, cfg.rollKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,  cfg.rollKi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.15f, cfg.rollKd);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.00f, cfg.maxOutputVelocity);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.866f, cfg.envelopeEnterSin);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.819f, cfg.envelopeExitSin);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, cfg.gyroPitchSign);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, cfg.gyroRollSign);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cfg.yawRateKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cfg.yawRateKi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cfg.headingKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, cfg.gyroYawSign);
}

void test_config_is_trivially_copyable() {
    TEST_ASSERT_TRUE(std::is_trivially_copyable<BalanceConfig>::value);
}

void test_envelope_invariant_in_defaults() {
    BalanceConfig cfg = RobotConstants::balanceConfig();
    TEST_ASSERT_TRUE(cfg.envelopeExitSin < cfg.envelopeEnterSin);
}

void test_save_then_load_round_trip() {
    BalanceConfig original = makeNonDefaultConfig();

    BalanceConfigStorage::save(original);

    // Wire the captured put-blob into the get path so load() reads what save() wrote.
    g_get_blob = g_put_blob;
    g_get_return_size = g_get_blob.size();

    BalanceConfig loaded = RobotConstants::balanceConfig();  // start non-equal
    bool ok = BalanceConfigStorage::load(loaded);
    TEST_ASSERT_TRUE(ok);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.tiltPerVelocity,   loaded.tiltPerVelocity);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.maxTiltSetpoint,   loaded.maxTiltSetpoint);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.pitchKp,           loaded.pitchKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.pitchKi,           loaded.pitchKi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.pitchKd,           loaded.pitchKd);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.rollKp,            loaded.rollKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.rollKi,            loaded.rollKi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.rollKd,            loaded.rollKd);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.maxOutputVelocity, loaded.maxOutputVelocity);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.envelopeEnterSin,  loaded.envelopeEnterSin);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.envelopeExitSin,   loaded.envelopeExitSin);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.gyroPitchSign,     loaded.gyroPitchSign);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.gyroRollSign,      loaded.gyroRollSign);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.yawRateKp,         loaded.yawRateKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.yawRateKi,         loaded.yawRateKi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.headingKp,         loaded.headingKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, original.gyroYawSign,       loaded.gyroYawSign);
}

void test_load_with_missing_blob_returns_defaults() {
    g_get_blob.clear();
    g_get_return_size = 0;  // NVS miss

    BalanceConfig sentinel = makeNonDefaultConfig();
    BalanceConfig out = sentinel;
    bool ok = BalanceConfigStorage::load(out);

    TEST_ASSERT_FALSE(ok);
    // load() leaves `out` unchanged on miss.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sentinel.pitchKp, out.pitchKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sentinel.tiltPerVelocity, out.tiltPerVelocity);
}

void test_load_with_wrong_size_returns_defaults() {
    // Populate a plausible-looking but truncated blob.
    g_get_blob.assign(sizeof(uint16_t) + sizeof(BalanceConfig) - 1, 0xAB);
    g_get_return_size = sizeof(BalanceConfig) - 1;  // wrong size

    BalanceConfig sentinel = makeNonDefaultConfig();
    BalanceConfig out = sentinel;
    bool ok = BalanceConfigStorage::load(out);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sentinel.pitchKp, out.pitchKp);
}

void test_load_with_wrong_version_returns_defaults() {
    BalanceConfig payload = makeNonDefaultConfig();
    uint16_t bad_version = 0;
    g_get_blob.resize(sizeof(uint16_t) + sizeof(BalanceConfig));
    std::memcpy(g_get_blob.data(), &bad_version, sizeof(uint16_t));
    std::memcpy(g_get_blob.data() + sizeof(uint16_t), &payload, sizeof(BalanceConfig));
    g_get_return_size = g_get_blob.size();

    BalanceConfig sentinel = RobotConstants::balanceConfig();
    BalanceConfig out = sentinel;
    bool ok = BalanceConfigStorage::load(out);

    TEST_ASSERT_FALSE(ok);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, sentinel.pitchKp, out.pitchKp);
}

void test_save_writes_version_prefix() {
    BalanceConfig cfg = makeNonDefaultConfig();
    BalanceConfigStorage::save(cfg);

    TEST_ASSERT_EQUAL_UINT(sizeof(uint16_t) + sizeof(BalanceConfig), g_put_blob.size());
    // Little-endian VERSION = 4 → 0x04 0x00.
    TEST_ASSERT_EQUAL_UINT8(0x04, g_put_blob[0]);
    TEST_ASSERT_EQUAL_UINT8(0x00, g_put_blob[1]);

    BalanceConfig roundtrip;
    std::memcpy(&roundtrip, g_put_blob.data() + sizeof(uint16_t), sizeof(BalanceConfig));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg.pitchKp,         roundtrip.pitchKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg.envelopeExitSin, roundtrip.envelopeExitSin);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg.gyroPitchSign,   roundtrip.gyroPitchSign);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg.gyroYawSign,     roundtrip.gyroYawSign);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg.yawRateKp,       roundtrip.yawRateKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfg.headingKp,       roundtrip.headingKp);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_default_config_values_match_spec);
    RUN_TEST(test_config_is_trivially_copyable);
    RUN_TEST(test_envelope_invariant_in_defaults);

    RUN_TEST(test_save_then_load_round_trip);
    RUN_TEST(test_load_with_missing_blob_returns_defaults);
    RUN_TEST(test_load_with_wrong_size_returns_defaults);
    RUN_TEST(test_load_with_wrong_version_returns_defaults);
    RUN_TEST(test_save_writes_version_prefix);

    return UNITY_END();
}
