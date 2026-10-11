// Unit tests for the pure parts of ARC-Core (run on an ESP32-S3: `pio test`).
#include <Arduino.h>
#include <unity.h>
// Pure modules only (no RTC / WiFi): compiled directly into the test.
#include "../../../src/arc_timecode.cpp"
#include "../../../src/arc_timeline.cpp"
#include "../../../src/arc_userbits.cpp"

using namespace arc;

static void test_df_known_values() {
    Timecode t;
    t = {0, 1, 0, 2};
    TEST_ASSERT_EQUAL_UINT32(1800, tcToFrames(t, TcRate::R2997DF));
    t = {0, 10, 0, 0};
    TEST_ASSERT_EQUAL_UINT32(17982, tcToFrames(t, TcRate::R2997DF));
    const Timecode a = framesToTc(1799, TcRate::R2997DF);
    TEST_ASSERT_TRUE((a == Timecode{0, 0, 59, 29}));
    const Timecode b = framesToTc(1800, TcRate::R2997DF);
    TEST_ASSERT_TRUE((b == Timecode{0, 1, 0, 2}));
}

static void test_df_roundtrip_day() {
    for (uint32_t f = 0; f < framesPerDay(TcRate::R2997DF); f += 7) {
        const Timecode tc = framesToTc(f, TcRate::R2997DF);
        TEST_ASSERT_EQUAL_UINT32(f, tcToFrames(tc, TcRate::R2997DF));
        if (tc.m % 10 != 0 && tc.s == 0) TEST_ASSERT_TRUE(tc.f >= 2);
    }
}

static void test_frame_time_consistency() {
    const TcRate rates[] = {TcRate::R24, TcRate::R25, TcRate::R2997DF, TcRate::R30};
    for (TcRate r : rates) {
        for (uint32_t k = 0; k < framesPerDay(r); k += 997) {
            int64_t into = -1;
            TEST_ASSERT_EQUAL_UINT32(k, frameAtNs(frameStartNs(k, r), r, &into));
            TEST_ASSERT_EQUAL_INT64(0, into);
            TEST_ASSERT_EQUAL_UINT32(k, frameAtNs(frameStartNs(k + 1, r) - 1, r, nullptr));
        }
    }
}

static void test_timeline_freerun() {
    const Timeline tl = Timeline::freeRun(TcRate::R25);
    const int64_t day = 86400LL * 1000000000LL;
    const int64_t t = 20000 * day + (10LL * 3600 + 20 * 60 + 30) * 1000000000LL + 123000000LL;  // 10:20:30 + 123 ms
    uint32_t f; int64_t into;
    TEST_ASSERT_TRUE(tl.frameAt(t, f, into));
    const Timecode tc = framesToTc(f, TcRate::R25);
    TEST_ASSERT_TRUE((tc == Timecode{10, 20, 30, 3}));
    TEST_ASSERT_EQUAL_INT64(3000000, into);   // 123 ms - 3 x 40 ms
    TEST_ASSERT_EQUAL_INT64(t - into, tl.frameStartNs(f, t));
    // midnight: frame 0 of the next day is closest
    const int64_t beforeMidnight = 20001 * day - 1000000;
    TEST_ASSERT_EQUAL_INT64(20001 * day, tl.frameStartNs(0, beforeMidnight));
}

static void test_timeline_anchored() {
    const int64_t t0 = 1000000000000LL;
    Timeline tl = Timeline::anchored(TcRate::R25, 100, t0);
    uint32_t f; int64_t into;
    TEST_ASSERT_FALSE(tl.frameAt(t0 - 1, f, into));
    TEST_ASSERT_TRUE(tl.frameAt(t0 + 80000000LL + 5, f, into));
    TEST_ASSERT_EQUAL_UINT32(102, f);
    TEST_ASSERT_EQUAL_INT64(t0 + 80000000LL, tl.frameStartNs(102, t0));
    tl = tl.stoppingAt(t0 + 1000000000LL);
    TEST_ASSERT_FALSE(tl.frameAt(t0 + 2000000000LL, f, into));
    TEST_ASSERT_EQUAL_UINT32(124, f);   // frozen at the last frame before the stop
}

static void test_userbits() {
    // A001 / scene 1 / shot 1 / take 2 -> A0204082 (matches ARC-Server data)
    TEST_ASSERT_EQUAL_HEX32(0xA0204082, shotUserBits('A', 1, 1, 1, 2));
    TEST_ASSERT_EQUAL_HEX32(0x12345678, reverseNibbles(0x87654321));
    TEST_ASSERT_TRUE(isEuSummerTime(2026, 7, 1, 12));
    TEST_ASSERT_FALSE(isEuSummerTime(2026, 1, 1, 12));
    TEST_ASSERT_TRUE(isEuSummerTime(2026, 3, 29, 2));    // last Sunday of March 2026
    TEST_ASSERT_FALSE(isEuSummerTime(2026, 3, 29, 1));
    TEST_ASSERT_FALSE(isEuSummerTime(2026, 10, 25, 3));  // last Sunday of October 2026
}

static void test_parse_format() {
    Timecode t;
    TEST_ASSERT_TRUE(parseTc("01:02:03:04", t));
    TEST_ASSERT_TRUE((t == Timecode{1, 2, 3, 4}));
    TEST_ASSERT_TRUE(parseTc("01:02:03;04", t));
    TEST_ASSERT_FALSE(parseTc("25:00:00:00", t));
    char buf[12];
    formatTc(Timecode{9, 8, 7, 6}, buf);
    TEST_ASSERT_EQUAL_STRING("09:08:07:06", buf);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_df_known_values);
    RUN_TEST(test_df_roundtrip_day);
    RUN_TEST(test_frame_time_consistency);
    RUN_TEST(test_timeline_freerun);
    RUN_TEST(test_timeline_anchored);
    RUN_TEST(test_userbits);
    RUN_TEST(test_parse_format);
    UNITY_END();
}

void loop() {}
