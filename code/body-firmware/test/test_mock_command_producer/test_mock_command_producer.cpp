/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include "MockCommandProducer.h"
#include "sync/CommandLatch.h"
#include "BodyVelocity.h"

static CommandLatch<BodyVelocity>* latch = nullptr;
static MockCommandProducer<BodyVelocity>* producer = nullptr;

void setUp() {
    latch = new CommandLatch<BodyVelocity>();
    producer = new MockCommandProducer<BodyVelocity>(*latch);
}

void tearDown() {
    delete producer;
    delete latch;
    producer = nullptr; latch = nullptr;
}

void test_default_state_not_started_default_connected_true() {
    TEST_ASSERT_FALSE(producer->isStarted());
    TEST_ASSERT_TRUE(producer->connected());
}

void test_start_then_stop_toggles_state() {
    producer->start();
    TEST_ASSERT_TRUE(producer->isStarted());
    producer->stop();
    TEST_ASSERT_FALSE(producer->isStarted());
}

void test_inject_when_started_writes_to_latch() {
    producer->start();
    BodyVelocity cmd{1.0f, 2.0f, 3.0f};
    producer->inject(cmd);
    auto got = latch->read();
    TEST_ASSERT_TRUE(got.has_value());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, got->vx);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, got->vy);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, got->omega);
}

void test_inject_when_stopped_does_not_write() {
    // producer never start()ed
    BodyVelocity cmd{1.0f, 2.0f, 3.0f};
    producer->inject(cmd);
    auto got = latch->read();
    TEST_ASSERT_FALSE(got.has_value());
}

void test_set_connected_reflected_by_connected() {
    producer->setConnected(false);
    TEST_ASSERT_FALSE(producer->connected());
    producer->setConnected(true);
    TEST_ASSERT_TRUE(producer->connected());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_default_state_not_started_default_connected_true);
    RUN_TEST(test_start_then_stop_toggles_state);
    RUN_TEST(test_inject_when_started_writes_to_latch);
    RUN_TEST(test_inject_when_stopped_does_not_write);
    RUN_TEST(test_set_connected_reflected_by_connected);
    return UNITY_END();
}
