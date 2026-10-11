#pragma once
// LTC decoder: edge-timing biphase-mark ISR on the LTC input pin.
// Every decoded frame carries the esp_timer time of its sync-word end, which
// is the boundary to the next frame on the line (plus the input delay of the
// LM358/74HC14 stage, identical on all ARC_AP boards).
#include <Arduino.h>
#include "arc_timecode.h"

namespace arc {

struct DecodedFrame {
    Timecode tc;           // TC of the frame that just ended
    TcRate rate;           // detected rate (falls back to the last known)
    bool rateKnown;
    bool dropFrame;
    uint32_t userBits;     // display order
    int64_t endEspUs;      // esp_timer at the end of the frame (= start of the next)
    int64_t periodUs;      // time since the previous frame
    bool reverse;
};

namespace ltcin {

void begin(int pin, void (*onFrame)(const DecodedFrame&));
uint32_t framesReceived();

}  // namespace ltcin
}  // namespace arc
