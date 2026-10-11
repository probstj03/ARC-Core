#pragma once
// Timecode arithmetic (pure C++, no hardware): rates, drop-frame conversion,
// frame <-> real time. All "real time" values are wall-clock nanoseconds.
#include <stdint.h>

namespace arc {

enum class TcRate : uint8_t { R24, R25, R2997DF, R30 };

struct Timecode {
    uint8_t h = 0, m = 0, s = 0, f = 0;
    Timecode() = default;
    Timecode(uint8_t hh, uint8_t mm, uint8_t ss, uint8_t ff) : h(hh), m(mm), s(ss), f(ff) {}
    bool operator==(const Timecode& o) const { return h == o.h && m == o.m && s == o.s && f == o.f; }
    bool operator!=(const Timecode& o) const { return !(*this == o); }
};

uint8_t labelFps(TcRate r);            // frames per second in the label (30 for DF)
double realFps(TcRate r);              // 24, 25, 29.97002997, 30
int64_t frameDurationNs(TcRate r);     // real duration of one frame
uint32_t framesPerDay(TcRate r);

uint32_t tcToFrames(const Timecode& tc, TcRate r);
Timecode framesToTc(uint32_t frames, TcRate r);

// Number of whole frames in `ns` (no day wrap); `intoNs` = remainder.
int64_t framesIn(int64_t ns, TcRate r, int64_t* intoNs);
// Duration of `frames` frames in ns (rounded up, inverse of framesIn).
int64_t framesDurationNs(int64_t frames, TcRate r);

// Frame of the day running at time `nsOfDay` (0 .. 24 h); `intoNs` = time since its start.
uint32_t frameAtNs(int64_t nsOfDay, TcRate r, int64_t* intoNs);
// Start of a frame of the day in ns after midnight.
int64_t frameStartNs(uint32_t frame, TcRate r);

const char* rateKey(TcRate r);         // "24", "25", "29.97DF", "30"
bool rateFromKey(const char* key, TcRate& out);
bool rateFromFps(double fps, TcRate& out);

bool parseTc(const char* s, Timecode& out);   // "HH:MM:SS:FF" or "HH:MM:SS;FF"
void formatTc(const Timecode& tc, char out[12]);

}  // namespace arc
