#include <unity.h>

void test_true_is_true() {
    TEST_ASSERT_TRUE(true);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_true_is_true);
    return UNITY_END();
}
