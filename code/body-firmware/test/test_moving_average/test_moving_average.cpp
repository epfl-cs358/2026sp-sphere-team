#include <unity.h>
#include <cmath>
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

void test_window_size_one() {
    MovingAverage<1> ma;
    ma.push(42.0f);
    TEST_ASSERT_EQUAL_FLOAT(42.0f, ma.average());
    ma.push(99.0f);
    TEST_ASSERT_EQUAL_FLOAT(99.0f, ma.average());
}

void test_push_after_reset() {
    MovingAverage<3> ma;
    ma.push(100.0f);
    ma.push(100.0f);
    ma.push(100.0f);
    ma.push(100.0f);
    ma.reset();
    ma.push(7.0f);
    TEST_ASSERT_EQUAL_FLOAT(7.0f, ma.average());
    ma.push(8.0f);
    TEST_ASSERT_EQUAL_FLOAT(7.5f, ma.average());
}

void test_negative_values() {
    MovingAverage<3> ma;
    ma.push(-100.0f);
    ma.push(-200.0f);
    ma.push(-300.0f);
    TEST_ASSERT_EQUAL_FLOAT(-200.0f, ma.average());
}

void test_mixed_sign_values() {
    MovingAverage<4> ma;
    ma.push(100.0f);
    ma.push(-100.0f);
    ma.push(100.0f);
    ma.push(-100.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, ma.average());
}

void test_partial_fill() {
    MovingAverage<5> ma;
    ma.push(10.0f);
    ma.push(20.0f);
    // 2 values in size-5 window: average over 2, not 5
    TEST_ASSERT_EQUAL_FLOAT(15.0f, ma.average());
}

void test_nan_propagation() {
    MovingAverage<3> ma;
    ma.push(1.0f);
    ma.push(NAN);
    ma.push(3.0f);
    TEST_ASSERT_TRUE(std::isnan(ma.average()));
}

void test_accuracy_after_many_pushes() {
    MovingAverage<5> ma;
    for (int i = 0; i < 1000; ++i) {
        ma.push(1.0f);
    }
    TEST_ASSERT_EQUAL_FLOAT(1.0f, ma.average());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_empty_returns_zero);
    RUN_TEST(test_single_value);
    RUN_TEST(test_multiple_values);
    RUN_TEST(test_wraparound);
    RUN_TEST(test_reset);
    RUN_TEST(test_window_size_one);
    RUN_TEST(test_push_after_reset);
    RUN_TEST(test_negative_values);
    RUN_TEST(test_mixed_sign_values);
    RUN_TEST(test_partial_fill);
    RUN_TEST(test_nan_propagation);
    RUN_TEST(test_accuracy_after_many_pushes);
    return UNITY_END();
}
