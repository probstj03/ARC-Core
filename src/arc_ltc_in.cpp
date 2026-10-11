#include "arc_ltc_in.h"
#include <esp_timer.h>

namespace arc {
namespace ltcin {

namespace {

struct RawFrame {
    uint8_t bits[10];
    bool reverse;
    int64_t endEspUs;
};

QueueHandle_t s_queue = nullptr;
void (*s_onFrame)(const DecodedFrame&) = nullptr;
volatile uint32_t s_frames = 0;

uint8_t flip8(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
}

// States: 0 = init, 1 = measure the half-bit period, 2 = read bits.
void IRAM_ATTR isrLtc() {
    const int64_t nowEsp = esp_timer_get_time();
    const uint32_t now = (uint32_t)nowEsp;
    static uint32_t lastEdge = 0;
    const uint32_t cell = now - lastEdge;
    if (cell < 100) return;   // glitch filter
    lastEdge = now;

    static uint8_t shift[10];
    static uint8_t st = 0, counter = 0, zCount = 0;
    static bool eatEdge = false;
    static uint32_t accum = 0, cellOne = 0, cellMin = 0, cellDet = 0, cellMax = 0;

    if (st == 0) {
        for (int i = 0; i < 10; i++) shift[i] = 0;
        eatEdge = false; counter = 0; zCount = 0; accum = 0; cellOne = 0;
        st = 1;
    } else if (st == 1) {
        if (++counter > 40) {
            if (zCount > 1) cellOne = (accum / (zCount - 1)) / 2;
            else { st = 0; return; }
            cellMin = cellOne / 2; cellDet = cellMin * 3; cellMax = cellMin * 5;
            st = 2;
        } else if (cell >= cellOne) {
            if (++zCount > 1) accum += cell;
            else cellOne = (cell * 2) / 3;
        }
    } else {
        if (cell < cellMin || cell > cellMax) { st = 0; return; }
        uint8_t bit = 0;
        if (cell > cellDet) {
            if (eatEdge) { st = 0; return; }
        } else {
            if (!eatEdge) { eatEdge = true; return; }
            eatEdge = false; bit = 0x80;
        }
        for (int i = 9; i > 0; i--) shift[i] = (shift[i] >> 1) | ((shift[i - 1] & 1) << 7);
        shift[0] = (shift[0] >> 1) | bit;
        const uint16_t fwd = ((uint16_t)shift[0] << 8) | shift[1];
        const uint16_t rvs = ((uint16_t)shift[8] << 8) | shift[9];
        if (fwd == 0xBFFC || rvs == 0x3FFD) {
            RawFrame f;
            for (int i = 0; i < 10; i++) f.bits[i] = shift[i];
            f.reverse = fwd != 0xBFFC;
            f.endEspUs = nowEsp;
            BaseType_t woken = pdFALSE;
            xQueueSendFromISR(s_queue, &f, &woken);
            for (int i = 0; i < 10; i++) shift[i] = 0;
            if (woken) portYIELD_FROM_ISR();
        }
        eatEdge = false;
    }
}

void decodeTask(void*) {
    RawFrame raw;
    int64_t lastEnd = 0;
    uint8_t maxFrame = 0, est = 0;
    uint32_t window = 0;
    TcRate lastRate = TcRate::R25;
    bool rateKnown = false;
    for (;;) {
        if (!xQueueReceive(s_queue, &raw, portMAX_DELAY)) continue;
        uint8_t b[10];
        if (raw.reverse) { for (int i = 0; i < 10; i++) b[i] = flip8(raw.bits[9 - i]); }
        else             { for (int i = 0; i < 10; i++) b[i] = raw.bits[i]; }

        DecodedFrame f;
        f.tc.h = ((b[2] & 0x03) * 10) + (b[3] & 0x0F);
        f.tc.m = ((b[4] & 0x07) * 10) + (b[5] & 0x0F);
        f.tc.s = ((b[6] & 0x07) * 10) + (b[7] & 0x0F);
        f.tc.f = ((b[8] & 0x03) * 10) + (b[9] & 0x0F);
        f.dropFrame = (b[8] & 0x04) != 0;
        // Display order: user1 is the most significant nibble.
        f.userBits = ((uint32_t)(b[9] >> 4) & 0xF) << 28 | ((uint32_t)(b[8] >> 4) & 0xF) << 24 |
                     ((uint32_t)(b[7] >> 4) & 0xF) << 20 | ((uint32_t)(b[6] >> 4) & 0xF) << 16 |
                     ((uint32_t)(b[5] >> 4) & 0xF) << 12 | ((uint32_t)(b[4] >> 4) & 0xF) << 8 |
                     ((uint32_t)(b[3] >> 4) & 0xF) << 4 | ((uint32_t)(b[2] >> 4) & 0xF);
        f.endEspUs = raw.endEspUs;
        f.periodUs = lastEnd ? raw.endEspUs - lastEnd : 0;
        f.reverse = raw.reverse;
        lastEnd = raw.endEspUs;

        // Rate: highest frame number over a window, DF flag, frame period.
        if (f.tc.f > maxFrame) maxFrame = f.tc.f;
        if (++window >= 32) {
            est = maxFrame + 1;
            maxFrame = 0;
            window = 0;
            if (f.dropFrame) { lastRate = TcRate::R2997DF; rateKnown = true; }
            else if (est == 24) { lastRate = TcRate::R24; rateKnown = true; }
            else if (est == 25) { lastRate = TcRate::R25; rateKnown = true; }
            else if (est == 30) {
                // 30 vs 29.97 non-drop are not distinguishable by numbers; ARC uses 30 ND.
                lastRate = TcRate::R30; rateKnown = true;
            }
        }
        if (f.dropFrame) { lastRate = TcRate::R2997DF; rateKnown = true; }
        f.rate = lastRate;
        f.rateKnown = rateKnown;
        s_frames++;
        if (s_onFrame) s_onFrame(f);
    }
}

}  // namespace

void begin(int pin, void (*onFrame)(const DecodedFrame&)) {
    s_onFrame = onFrame;
    s_queue = xQueueCreate(24, sizeof(RawFrame));
    xTaskCreatePinnedToCore(decodeTask, "ARCLTCIN", 4096, nullptr, 4, nullptr, 1);
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), isrLtc, CHANGE);
}

uint32_t framesReceived() { return s_frames; }

}  // namespace ltcin
}  // namespace arc
