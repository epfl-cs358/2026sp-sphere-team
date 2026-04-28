#include <unity.h>
#include <cstring>
#include "StreamConfig.h"
#include "StatusJson.h"

void setUp() {}
void tearDown() {}

// --- Status JSON ---

void test_status_json_contains_resolution() {
    StreamConfig cfg;
    char buf[256];
    buildStatusJson(buf, sizeof(buf), cfg, -55, 12345, 2);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"resolution\":\"VGA\""));
}

void test_status_json_contains_fps() {
    StreamConfig cfg;
    char buf[256];
    buildStatusJson(buf, sizeof(buf), cfg, -55, 12345, 2);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"fps\":20"));
}

void test_status_json_contains_quality() {
    StreamConfig cfg;
    char buf[256];
    buildStatusJson(buf, sizeof(buf), cfg, -55, 12345, 2);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"quality\":80"));
}

void test_status_json_contains_rssi() {
    StreamConfig cfg;
    char buf[256];
    buildStatusJson(buf, sizeof(buf), cfg, -55, 12345, 2);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"rssi\":-55"));
}

void test_status_json_contains_uptime() {
    StreamConfig cfg;
    char buf[256];
    buildStatusJson(buf, sizeof(buf), cfg, -55, 12345, 2);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"uptime\":12345"));
}

void test_status_json_contains_clients() {
    StreamConfig cfg;
    char buf[256];
    buildStatusJson(buf, sizeof(buf), cfg, -55, 12345, 2);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"clients\":2"));
}

void test_status_json_svga_resolution() {
    StreamConfig cfg;
    cfg.resolution = Resolution::SVGA;
    char buf[256];
    buildStatusJson(buf, sizeof(buf), cfg, -30, 0, 0);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"resolution\":\"SVGA\""));
}

void test_status_json_uxga_resolution() {
    StreamConfig cfg;
    cfg.resolution = Resolution::UXGA;
    char buf[256];
    buildStatusJson(buf, sizeof(buf), cfg, -70, 99999, 1);
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"resolution\":\"UXGA\""));
}

// --- Config JSON parsing ---

void test_parse_full_config() {
    StreamConfig cfg;
    const char* json = R"({"resolution":"SVGA","fps":25,"quality":90})";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL(Resolution::SVGA, cfg.resolution);
    TEST_ASSERT_EQUAL_UINT8(25, cfg.fps);
    TEST_ASSERT_EQUAL_UINT8(90, cfg.quality);
}

void test_parse_fps_only() {
    StreamConfig cfg;
    const char* json = R"({"fps":15})";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL(Resolution::VGA, cfg.resolution);
    TEST_ASSERT_EQUAL_UINT8(15, cfg.fps);
    TEST_ASSERT_EQUAL_UINT8(80, cfg.quality);
}

void test_parse_quality_only() {
    StreamConfig cfg;
    const char* json = R"({"quality":50})";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(50, cfg.quality);
}

void test_parse_resolution_only() {
    StreamConfig cfg;
    const char* json = R"({"resolution":"HD"})";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL(Resolution::HD, cfg.resolution);
}

void test_parse_invalid_resolution_unchanged() {
    StreamConfig cfg;
    const char* json = R"({"resolution":"INVALID"})";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL(Resolution::VGA, cfg.resolution);
}

void test_parse_fps_clamped_to_min() {
    StreamConfig cfg;
    const char* json = R"({"fps":0})";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(1, cfg.fps);
}

void test_parse_quality_clamped_to_max() {
    StreamConfig cfg;
    const char* json = R"({"quality":200})";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT8(100, cfg.quality);
}

void test_parse_empty_object_no_changes() {
    StreamConfig cfg;
    const char* json = R"({})";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL(Resolution::VGA, cfg.resolution);
    TEST_ASSERT_EQUAL_UINT8(20, cfg.fps);
    TEST_ASSERT_EQUAL_UINT8(80, cfg.quality);
}

void test_parse_malformed_json_returns_false() {
    StreamConfig cfg;
    const char* json = "not json at all";
    bool ok = parseConfigJson(json, strlen(json), cfg);
    TEST_ASSERT_FALSE(ok);
}

void test_parse_all_resolutions() {
    const char* names[] = {"VGA", "SVGA", "XGA", "HD", "UXGA"};
    Resolution expected[] = {Resolution::VGA, Resolution::SVGA, Resolution::XGA,
                             Resolution::HD, Resolution::UXGA};

    for (int i = 0; i < 5; i++) {
        StreamConfig cfg;
        char json[64];
        snprintf(json, sizeof(json), R"({"resolution":"%s"})", names[i]);
        bool ok = parseConfigJson(json, strlen(json), cfg);
        TEST_ASSERT_TRUE(ok);
        TEST_ASSERT_EQUAL(expected[i], cfg.resolution);
    }
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_status_json_contains_resolution);
    RUN_TEST(test_status_json_contains_fps);
    RUN_TEST(test_status_json_contains_quality);
    RUN_TEST(test_status_json_contains_rssi);
    RUN_TEST(test_status_json_contains_uptime);
    RUN_TEST(test_status_json_contains_clients);
    RUN_TEST(test_status_json_svga_resolution);
    RUN_TEST(test_status_json_uxga_resolution);

    RUN_TEST(test_parse_full_config);
    RUN_TEST(test_parse_fps_only);
    RUN_TEST(test_parse_quality_only);
    RUN_TEST(test_parse_resolution_only);
    RUN_TEST(test_parse_invalid_resolution_unchanged);
    RUN_TEST(test_parse_fps_clamped_to_min);
    RUN_TEST(test_parse_quality_clamped_to_max);
    RUN_TEST(test_parse_empty_object_no_changes);
    RUN_TEST(test_parse_malformed_json_returns_false);
    RUN_TEST(test_parse_all_resolutions);

    return UNITY_END();
}
