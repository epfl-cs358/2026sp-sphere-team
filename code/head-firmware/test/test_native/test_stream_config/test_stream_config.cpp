#include <unity.h>
#include "StreamConfig.h"

void setUp() {}
void tearDown() {}

// --- Default construction ---

void test_default_resolution_is_vga() {
    StreamConfig cfg;
    TEST_ASSERT_EQUAL(Resolution::VGA, cfg.resolution);
}

void test_default_fps_is_20() {
    StreamConfig cfg;
    TEST_ASSERT_EQUAL_UINT8(20, cfg.fps);
}

void test_default_quality_is_80() {
    StreamConfig cfg;
    TEST_ASSERT_EQUAL_UINT8(80, cfg.quality);
}

// --- Resolution dimensions ---

void test_vga_width_640() {
    TEST_ASSERT_EQUAL_UINT16(640, StreamConfig::width(Resolution::VGA));
}

void test_vga_height_480() {
    TEST_ASSERT_EQUAL_UINT16(480, StreamConfig::height(Resolution::VGA));
}

void test_svga_width_800() {
    TEST_ASSERT_EQUAL_UINT16(800, StreamConfig::width(Resolution::SVGA));
}

void test_svga_height_600() {
    TEST_ASSERT_EQUAL_UINT16(600, StreamConfig::height(Resolution::SVGA));
}

void test_xga_width_1024() {
    TEST_ASSERT_EQUAL_UINT16(1024, StreamConfig::width(Resolution::XGA));
}

void test_xga_height_768() {
    TEST_ASSERT_EQUAL_UINT16(768, StreamConfig::height(Resolution::XGA));
}

void test_hd_width_1280() {
    TEST_ASSERT_EQUAL_UINT16(1280, StreamConfig::width(Resolution::HD));
}

void test_hd_height_720() {
    TEST_ASSERT_EQUAL_UINT16(720, StreamConfig::height(Resolution::HD));
}

void test_uxga_width_1600() {
    TEST_ASSERT_EQUAL_UINT16(1600, StreamConfig::width(Resolution::UXGA));
}

void test_uxga_height_1200() {
    TEST_ASSERT_EQUAL_UINT16(1200, StreamConfig::height(Resolution::UXGA));
}

// --- FPS clamping ---

void test_clamp_fps_zero_becomes_1() {
    TEST_ASSERT_EQUAL_UINT8(1, StreamConfig::clampFps(0));
}

void test_clamp_fps_50_becomes_30() {
    TEST_ASSERT_EQUAL_UINT8(30, StreamConfig::clampFps(50));
}

void test_clamp_fps_15_stays_15() {
    TEST_ASSERT_EQUAL_UINT8(15, StreamConfig::clampFps(15));
}

void test_clamp_fps_1_stays_1() {
    TEST_ASSERT_EQUAL_UINT8(1, StreamConfig::clampFps(1));
}

void test_clamp_fps_30_stays_30() {
    TEST_ASSERT_EQUAL_UINT8(30, StreamConfig::clampFps(30));
}

// --- Quality clamping ---

void test_clamp_quality_5_becomes_10() {
    TEST_ASSERT_EQUAL_UINT8(10, StreamConfig::clampQuality(5));
}

void test_clamp_quality_200_becomes_100() {
    TEST_ASSERT_EQUAL_UINT8(100, StreamConfig::clampQuality(200));
}

void test_clamp_quality_50_stays_50() {
    TEST_ASSERT_EQUAL_UINT8(50, StreamConfig::clampQuality(50));
}

void test_clamp_quality_10_stays_10() {
    TEST_ASSERT_EQUAL_UINT8(10, StreamConfig::clampQuality(10));
}

void test_clamp_quality_100_stays_100() {
    TEST_ASSERT_EQUAL_UINT8(100, StreamConfig::clampQuality(100));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_default_resolution_is_vga);
    RUN_TEST(test_default_fps_is_20);
    RUN_TEST(test_default_quality_is_80);

    RUN_TEST(test_vga_width_640);
    RUN_TEST(test_vga_height_480);
    RUN_TEST(test_svga_width_800);
    RUN_TEST(test_svga_height_600);
    RUN_TEST(test_xga_width_1024);
    RUN_TEST(test_xga_height_768);
    RUN_TEST(test_hd_width_1280);
    RUN_TEST(test_hd_height_720);
    RUN_TEST(test_uxga_width_1600);
    RUN_TEST(test_uxga_height_1200);

    RUN_TEST(test_clamp_fps_zero_becomes_1);
    RUN_TEST(test_clamp_fps_50_becomes_30);
    RUN_TEST(test_clamp_fps_15_stays_15);
    RUN_TEST(test_clamp_fps_1_stays_1);
    RUN_TEST(test_clamp_fps_30_stays_30);

    RUN_TEST(test_clamp_quality_5_becomes_10);
    RUN_TEST(test_clamp_quality_200_becomes_100);
    RUN_TEST(test_clamp_quality_50_stays_50);
    RUN_TEST(test_clamp_quality_10_stays_10);
    RUN_TEST(test_clamp_quality_100_stays_100);

    return UNITY_END();
}
