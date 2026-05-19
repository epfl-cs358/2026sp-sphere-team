/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 *
 * test_websocket_control_dispatch — exercises WebSocketCommandProducer::dispatchFrame
 * with FFF-faked ArmingState namespace functions. The transport layer is not
 * exercised here; this test isolates the frame-routing helper.
 */

#include <unity.h>
#include <fff.h>
#include <atomic>
#include <cstdint>

#include "CommandFrameParser.h"
#include "BodyVelocity.h"
#include "sync/CommandLatch.h"
#include "WebSocketCommandProducer.h"

DEFINE_FFF_GLOBALS;

// FFF fakes shadowing the real ArmingState namespace symbols. The native test
// binary links these instead of ArmingState.cpp.
namespace ArmingState {
FAKE_VOID_FUNC(arm);
FAKE_VOID_FUNC(disarm);
FAKE_VOID_FUNC(kill);
FAKE_VOID_FUNC(clearKill);
FAKE_VALUE_FUNC(State, get);
}  // namespace ArmingState

static CommandLatch<BodyVelocity>* latch = nullptr;
static std::atomic<uint32_t>* frameCount = nullptr;

void setUp() {
    latch = new CommandLatch<BodyVelocity>();
    frameCount = new std::atomic<uint32_t>(0);
    RESET_FAKE(ArmingState::arm);
    RESET_FAKE(ArmingState::disarm);
    RESET_FAKE(ArmingState::kill);
    RESET_FAKE(ArmingState::clearKill);
    RESET_FAKE(ArmingState::get);
    FFF_RESET_HISTORY();
}

void tearDown() {
    delete frameCount;
    delete latch;
    frameCount = nullptr;
    latch = nullptr;
}

static FrameParseResult makeControl(ControlVerb v) {
    FrameParseResult r;
    r.kind = FrameKind::Control;
    r.control = v;
    r.error = FrameParseError::None;
    return r;
}

static FrameParseResult makeVelocity(float vx, float vy, float omega) {
    FrameParseResult r;
    r.kind = FrameKind::Velocity;
    r.velocity = BodyVelocity{vx, vy, omega};
    r.error = FrameParseError::None;
    return r;
}

void test_control_arm_frame_invokes_arming_state_arm() {
    WebSocketCommandProducer::dispatchFrame(*latch, *frameCount,
                                            makeControl(ControlVerb::Arm));
    TEST_ASSERT_EQUAL_UINT32(1, ArmingState::arm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::disarm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::kill_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::clearKill_fake.call_count);
    TEST_ASSERT_FALSE(latch->read().has_value());
}

void test_control_disarm() {
    WebSocketCommandProducer::dispatchFrame(*latch, *frameCount,
                                            makeControl(ControlVerb::Disarm));
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::arm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(1, ArmingState::disarm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::kill_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::clearKill_fake.call_count);
    TEST_ASSERT_FALSE(latch->read().has_value());
}

void test_control_kill() {
    WebSocketCommandProducer::dispatchFrame(*latch, *frameCount,
                                            makeControl(ControlVerb::Kill));
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::arm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::disarm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(1, ArmingState::kill_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::clearKill_fake.call_count);
    TEST_ASSERT_FALSE(latch->read().has_value());
}

void test_control_clearkill() {
    WebSocketCommandProducer::dispatchFrame(*latch, *frameCount,
                                            makeControl(ControlVerb::ClearKill));
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::arm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::disarm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::kill_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(1, ArmingState::clearKill_fake.call_count);
    TEST_ASSERT_FALSE(latch->read().has_value());
}

void test_websocket_control_dispatch_echoes_armstate_on_query() {
    // QueryArmState must invoke the supplied query callback exactly once and
    // pass through the current arming state (read by the callback itself, not
    // by dispatchFrame — dispatch stays decoupled from ArmingState reads).
    int callbackCount = 0;
    auto cb = [&callbackCount](ArmingState::State) { ++callbackCount; };

    WebSocketCommandProducer::dispatchFrame(*latch, *frameCount,
                                            makeControl(ControlVerb::QueryArmState),
                                            cb);

    TEST_ASSERT_EQUAL_INT(1, callbackCount);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::arm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::disarm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::kill_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::clearKill_fake.call_count);
    TEST_ASSERT_FALSE(latch->read().has_value());
}

void test_velocity_frame_writes_latch_does_not_invoke_arming() {
    WebSocketCommandProducer::dispatchFrame(*latch, *frameCount,
                                            makeVelocity(0.5f, -0.25f, 1.5f));
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::arm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::disarm_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::kill_fake.call_count);
    TEST_ASSERT_EQUAL_UINT32(0, ArmingState::clearKill_fake.call_count);
    auto got = latch->read();
    TEST_ASSERT_TRUE(got.has_value());
    TEST_ASSERT_EQUAL_FLOAT(0.5f, got->vx);
    TEST_ASSERT_EQUAL_FLOAT(-0.25f, got->vy);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, got->omega);
    TEST_ASSERT_EQUAL_UINT32(1, frameCount->load());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_control_arm_frame_invokes_arming_state_arm);
    RUN_TEST(test_control_disarm);
    RUN_TEST(test_control_kill);
    RUN_TEST(test_control_clearkill);
    RUN_TEST(test_websocket_control_dispatch_echoes_armstate_on_query);
    RUN_TEST(test_velocity_frame_writes_latch_does_not_invoke_arming);
    return UNITY_END();
}
