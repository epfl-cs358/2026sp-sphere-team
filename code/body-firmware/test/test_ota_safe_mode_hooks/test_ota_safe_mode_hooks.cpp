#include <unity.h>

#include "ArmingState.h"
#include "ArmingState.cpp"
#include "OtaSafeMode.cpp"

namespace OtaSafeMode {
namespace detail {
void onOtaStartForTest();
}
}  // namespace OtaSafeMode

using ArmingState::State;

void setUp() {
    ArmingState::begin();
}

void tearDown() {}

void test_on_ota_start_disarms() {
    ArmingState::arm();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Armed),
                      static_cast<int>(ArmingState::get()));

    OtaSafeMode::detail::onOtaStartForTest();

    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));
    TEST_ASSERT_TRUE(OtaSafeMode::isUpdating());
}

void test_on_ota_start_from_disarmed_is_noop() {
    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));

    OtaSafeMode::detail::onOtaStartForTest();

    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));
}

void test_on_ota_start_from_killed_stays_killed() {
    ArmingState::kill();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Killed),
                      static_cast<int>(ArmingState::get()));

    OtaSafeMode::detail::onOtaStartForTest();

    TEST_ASSERT_EQUAL(static_cast<int>(State::Killed),
                      static_cast<int>(ArmingState::get()));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_on_ota_start_disarms);
    RUN_TEST(test_on_ota_start_from_disarmed_is_noop);
    RUN_TEST(test_on_ota_start_from_killed_stays_killed);

    return UNITY_END();
}
