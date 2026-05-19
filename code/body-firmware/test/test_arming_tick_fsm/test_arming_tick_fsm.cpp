/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 *
 * test_arming_tick_fsm — unit tests for the edge-dispatch FSM extracted from
 * controlTask. Each callback must fire exactly once on its edge, and never on
 * a same-state tick.
 */

#include <unity.h>

#include "ArmingState.h"
#include "ArmingTickFsm.h"

using ArmingState::State;

static int armedEdgeCount = 0;
static int disarmEdgeCount = 0;
static int killedEdgeCount = 0;

static ArmingFsmCallbacks makeCallbacks() {
    return ArmingFsmCallbacks{
        []() { ++armedEdgeCount; },
        []() { ++disarmEdgeCount; },
        []() { ++killedEdgeCount; },
    };
}

void setUp() {
    armedEdgeCount = 0;
    disarmEdgeCount = 0;
    killedEdgeCount = 0;
}

void tearDown() {}

void test_disarmed_to_armed_fires_armed_edge_once() {
    State prev = State::Disarmed;
    armingFsmTick(State::Armed, prev, makeCallbacks());
    TEST_ASSERT_EQUAL(1, armedEdgeCount);
    TEST_ASSERT_EQUAL(0, disarmEdgeCount);
    TEST_ASSERT_EQUAL(0, killedEdgeCount);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Armed), static_cast<int>(prev));
}

void test_armed_to_disarmed_fires_disarm_edge_once() {
    State prev = State::Armed;
    armingFsmTick(State::Disarmed, prev, makeCallbacks());
    TEST_ASSERT_EQUAL(0, armedEdgeCount);
    TEST_ASSERT_EQUAL(1, disarmEdgeCount);
    TEST_ASSERT_EQUAL(0, killedEdgeCount);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed), static_cast<int>(prev));
}

void test_any_to_killed_fires_killed_edge_once_from_armed() {
    State prev = State::Armed;
    armingFsmTick(State::Killed, prev, makeCallbacks());
    TEST_ASSERT_EQUAL(0, armedEdgeCount);
    TEST_ASSERT_EQUAL(0, disarmEdgeCount);
    TEST_ASSERT_EQUAL(1, killedEdgeCount);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Killed), static_cast<int>(prev));
}

void test_disarmed_to_killed_fires_killed_edge_once() {
    State prev = State::Disarmed;
    armingFsmTick(State::Killed, prev, makeCallbacks());
    TEST_ASSERT_EQUAL(0, armedEdgeCount);
    TEST_ASSERT_EQUAL(0, disarmEdgeCount);
    TEST_ASSERT_EQUAL(1, killedEdgeCount);
}

void test_same_state_disarmed_fires_nothing() {
    State prev = State::Disarmed;
    armingFsmTick(State::Disarmed, prev, makeCallbacks());
    TEST_ASSERT_EQUAL(0, armedEdgeCount);
    TEST_ASSERT_EQUAL(0, disarmEdgeCount);
    TEST_ASSERT_EQUAL(0, killedEdgeCount);
}

void test_same_state_armed_fires_nothing() {
    State prev = State::Armed;
    armingFsmTick(State::Armed, prev, makeCallbacks());
    TEST_ASSERT_EQUAL(0, armedEdgeCount);
    TEST_ASSERT_EQUAL(0, disarmEdgeCount);
    TEST_ASSERT_EQUAL(0, killedEdgeCount);
}

void test_same_state_killed_fires_nothing() {
    State prev = State::Killed;
    armingFsmTick(State::Killed, prev, makeCallbacks());
    TEST_ASSERT_EQUAL(0, armedEdgeCount);
    TEST_ASSERT_EQUAL(0, disarmEdgeCount);
    TEST_ASSERT_EQUAL(0, killedEdgeCount);
}

// Killed->Disarmed via clearKill: the FSM should NOT fire the Armed-Disarm
// edge (prev was Killed, not Armed). No PID-reset side-effect is appropriate
// on this transition. This guard locks in that exact contract.
void test_killed_to_disarmed_fires_no_callback() {
    State prev = State::Killed;
    armingFsmTick(State::Disarmed, prev, makeCallbacks());
    TEST_ASSERT_EQUAL(0, armedEdgeCount);
    TEST_ASSERT_EQUAL(0, disarmEdgeCount);
    TEST_ASSERT_EQUAL(0, killedEdgeCount);
    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed), static_cast<int>(prev));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_disarmed_to_armed_fires_armed_edge_once);
    RUN_TEST(test_armed_to_disarmed_fires_disarm_edge_once);
    RUN_TEST(test_any_to_killed_fires_killed_edge_once_from_armed);
    RUN_TEST(test_disarmed_to_killed_fires_killed_edge_once);
    RUN_TEST(test_same_state_disarmed_fires_nothing);
    RUN_TEST(test_same_state_armed_fires_nothing);
    RUN_TEST(test_same_state_killed_fires_nothing);
    RUN_TEST(test_killed_to_disarmed_fires_no_callback);
    return UNITY_END();
}
