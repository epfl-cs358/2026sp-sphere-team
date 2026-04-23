#include <unity.h>
#include "MovingAverage.h"

void test_empty_returns_zero() {
    MovingAverage<5> ma;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ma.average());
}

void test_single_value() {
    MovingAverage<5> ma;
    ma.push(10.0f);
    TEST_ASSERT_EQUAL_FLOAT(10.0f, ma.average());
}

void test_multiple_values() {
    MovingAverage<3> ma;
    ma.push(1.0f);
    ma.push(2.0f);
    ma.push(3.0f);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, ma.average());
}

void test_wraparound() {
    MovingAverage<3> ma;
    ma.push(1.0f);
    ma.push(2.0f);
    ma.push(3.0f);
    ma.push(10.0f);  // overwrites 1.0f
    TEST_ASSERT_EQUAL_FLOAT(5.0f, ma.average());  // (2+3+10)/3
}

void test_reset() {
    MovingAverage<3> ma;
    ma.push(5.0f);
    ma.push(5.0f);
    ma.reset();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ma.average());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_empty_returns_zero);
    RUN_TEST(test_single_value);
    RUN_TEST(test_multiple_values);
    RUN_TEST(test_wraparound);
    RUN_TEST(test_reset);
    return UNITY_END();
}
