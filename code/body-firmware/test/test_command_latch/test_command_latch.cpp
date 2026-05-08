/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include "sync/CommandLatch.h"
#include "BodyVelocity.h"

static CommandLatch<BodyVelocity>* latch = nullptr;

void setUp() {
    latch = new CommandLatch<BodyVelocity>();
}

void tearDown() {
    delete latch;
    latch = nullptr;
}

void test_empty_read_returns_nullopt() {
    auto v = latch->read();
    TEST_ASSERT_FALSE(v.has_value());
}

void test_write_then_read_returns_value() {
    BodyVelocity in{1.0f, 2.0f, 3.0f};
    latch->write(in);
    auto v = latch->read();
    TEST_ASSERT_TRUE(v.has_value());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, v->vx);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, v->vy);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, v->omega);
}

void test_second_read_returns_nullopt() {
    BodyVelocity in{1.0f, 2.0f, 3.0f};
    latch->write(in);
    auto first = latch->read();
    TEST_ASSERT_TRUE(first.has_value());
    auto second = latch->read();
    TEST_ASSERT_FALSE(second.has_value());
}

void test_overwrite_returns_latest() {
    latch->write({1.0f, 0.0f, 0.0f});
    latch->write({9.0f, 8.0f, 7.0f});
    auto v = latch->read();
    TEST_ASSERT_TRUE(v.has_value());
    TEST_ASSERT_EQUAL_FLOAT(9.0f, v->vx);
    TEST_ASSERT_EQUAL_FLOAT(8.0f, v->vy);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, v->omega);
}

void test_consumed_value_does_not_reappear_after_overwrite() {
    // Write A, read A, then overwrite-twice with B then C, read C.
    // Catches a regression where a consumed read could reappear.
    latch->write({1.0f, 1.0f, 1.0f});
    auto a = latch->read();
    TEST_ASSERT_TRUE(a.has_value());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, a->vx);

    latch->write({2.0f, 2.0f, 2.0f});
    latch->write({3.0f, 3.0f, 3.0f});
    auto c = latch->read();
    TEST_ASSERT_TRUE(c.has_value());
    TEST_ASSERT_EQUAL_FLOAT(3.0f, c->vx);

    auto empty = latch->read();
    TEST_ASSERT_FALSE(empty.has_value());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_empty_read_returns_nullopt);
    RUN_TEST(test_write_then_read_returns_value);
    RUN_TEST(test_second_read_returns_nullopt);
    RUN_TEST(test_overwrite_returns_latest);
    RUN_TEST(test_consumed_value_does_not_reappear_after_overwrite);
    return UNITY_END();
}
