#include <unity.h>
#include <cstring>
#include "FrameHeader.h"

void setUp() {}
void tearDown() {}

// --- Size constant ---

void test_header_size_is_8() {
    TEST_ASSERT_EQUAL(8, FrameHeader::SIZE);
}

// --- Encoding ---

void test_encode_sequence_and_timestamp() {
    uint8_t buf[FrameHeader::SIZE];
    FrameHeader::encode(buf, 42, 123456);

    // sequence = 42 = 0x0000002A in LE: 2A 00 00 00
    TEST_ASSERT_EQUAL_HEX8(0x2A, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[3]);

    // timestamp = 123456 = 0x0001E240 in LE: 40 E2 01 00
    TEST_ASSERT_EQUAL_HEX8(0x40, buf[4]);
    TEST_ASSERT_EQUAL_HEX8(0xE2, buf[5]);
    TEST_ASSERT_EQUAL_HEX8(0x01, buf[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00, buf[7]);
}

// --- Decoding ---

void test_decode_sequence_and_timestamp() {
    uint8_t buf[FrameHeader::SIZE] = {0x2A, 0x00, 0x00, 0x00,
                                       0x40, 0xE2, 0x01, 0x00};
    uint32_t seq, ts;
    FrameHeader::decode(buf, seq, ts);

    TEST_ASSERT_EQUAL_UINT32(42, seq);
    TEST_ASSERT_EQUAL_UINT32(123456, ts);
}

// --- Roundtrip ---

void test_roundtrip_typical_values() {
    uint8_t buf[FrameHeader::SIZE];
    FrameHeader::encode(buf, 1000, 5000000);

    uint32_t seq, ts;
    FrameHeader::decode(buf, seq, ts);

    TEST_ASSERT_EQUAL_UINT32(1000, seq);
    TEST_ASSERT_EQUAL_UINT32(5000000, ts);
}

// --- Edge cases ---

void test_roundtrip_zero_values() {
    uint8_t buf[FrameHeader::SIZE];
    FrameHeader::encode(buf, 0, 0);

    uint32_t seq, ts;
    FrameHeader::decode(buf, seq, ts);

    TEST_ASSERT_EQUAL_UINT32(0, seq);
    TEST_ASSERT_EQUAL_UINT32(0, ts);
}

void test_roundtrip_max_values() {
    uint8_t buf[FrameHeader::SIZE];
    FrameHeader::encode(buf, UINT32_MAX, UINT32_MAX);

    uint32_t seq, ts;
    FrameHeader::decode(buf, seq, ts);

    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, seq);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, ts);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_header_size_is_8);
    RUN_TEST(test_encode_sequence_and_timestamp);
    RUN_TEST(test_decode_sequence_and_timestamp);
    RUN_TEST(test_roundtrip_typical_values);
    RUN_TEST(test_roundtrip_zero_values);
    RUN_TEST(test_roundtrip_max_values);

    return UNITY_END();
}
