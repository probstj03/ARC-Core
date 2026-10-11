#include "arc_timecode.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

namespace arc {

uint8_t labelFps(TcRate r) {
    switch (r) {
        case TcRate::R24: return 24;
        case TcRate::R25: return 25;
        default:          return 30;
    }
}

double realFps(TcRate r) { return r == TcRate::R2997DF ? 30000.0 / 1001.0 : (double)labelFps(r); }

int64_t frameDurationNs(TcRate r) {
    // Exact for 24/25/30; DF: 1001/30000 s = 33366666.67 ns (rounded, see frameStartNs).
    switch (r) {
        case TcRate::R24:     return 41666667LL;
        case TcRate::R2997DF: return 33366667LL;
        case TcRate::R30:     return 33333333LL;
        default:              return 40000000LL;
    }
}

uint32_t framesPerDay(TcRate r) {
    switch (r) {
        case TcRate::R24:     return 24u * 86400u;
        case TcRate::R2997DF: return 2589408u;   // 144 x 17982
        case TcRate::R30:     return 30u * 86400u;
        default:              return 25u * 86400u;
    }
}

uint32_t tcToFrames(const Timecode& tc, TcRate r) {
    const uint32_t base = labelFps(r);
    uint32_t frames = ((uint32_t)tc.h * 3600u + (uint32_t)tc.m * 60u + tc.s) * base + tc.f;
    if (r == TcRate::R2997DF) {
        const uint32_t totalMinutes = 60u * tc.h + tc.m;
        frames -= 2u * (totalMinutes - totalMinutes / 10u);
    }
    return frames;
}

Timecode framesToTc(uint32_t frames, TcRate r) {
    frames %= framesPerDay(r);
    const uint32_t base = labelFps(r);
    if (r == TcRate::R2997DF) {
        const uint32_t d = frames / 17982u;
        const uint32_t m = frames % 17982u;
        frames += 18u * d + (m >= 2u ? 2u * ((m - 2u) / 1798u) : 0u);
    }
    Timecode tc;
    tc.f = frames % base;
    tc.s = (frames / base) % 60u;
    tc.m = (frames / (base * 60u)) % 60u;
    tc.h = (frames / (base * 3600u)) % 24u;
    return tc;
}

// Frame k of a rate starts at k * 1e9 / fps ns. For DF fps = 30000/1001, so
// the start is k * 1001e9 / 30000 = k * 100100000 / 3 ns. Rounded up, so that
// framesIn(framesDurationNs(k)) == k holds exactly.
int64_t framesDurationNs(int64_t k, TcRate r) {
    switch (r) {
        case TcRate::R24:     return (k * 1000000000LL + 23) / 24;
        case TcRate::R2997DF: return (k * 100100000LL + 2) / 3;
        case TcRate::R30:     return (k * 1000000000LL + 29) / 30;
        default:              return k * 40000000LL;
    }
}

int64_t framesIn(int64_t ns, TcRate r, int64_t* intoNs) {
    if (ns < 0) ns = 0;
    int64_t k;
    switch (r) {
        case TcRate::R24:     k = (ns * 24) / 1000000000LL; break;
        case TcRate::R2997DF: k = (ns * 3) / 100100000LL; break;
        case TcRate::R30:     k = (ns * 30) / 1000000000LL; break;
        default:              k = ns / 40000000LL; break;
    }
    if (intoNs) *intoNs = ns - framesDurationNs(k, r);
    return k;
}

int64_t frameStartNs(uint32_t frame, TcRate r) { return framesDurationNs(frame, r); }

uint32_t frameAtNs(int64_t nsOfDay, TcRate r, int64_t* intoNs) {
    return (uint32_t)(framesIn(nsOfDay, r, intoNs) % framesPerDay(r));
}

const char* rateKey(TcRate r) {
    switch (r) {
        case TcRate::R24:     return "24";
        case TcRate::R2997DF: return "29.97DF";
        case TcRate::R30:     return "30";
        default:              return "25";
    }
}

bool rateFromKey(const char* key, TcRate& out) {
    if (!key) return false;
    if (!strcmp(key, "24")) out = TcRate::R24;
    else if (!strcmp(key, "25")) out = TcRate::R25;
    else if (!strcmp(key, "29.97DF") || !strcmp(key, "29.97")) out = TcRate::R2997DF;
    else if (!strcmp(key, "30")) out = TcRate::R30;
    else return false;
    return true;
}

bool rateFromFps(double fps, TcRate& out) {
    if (fabs(fps - 24.0) < 0.01) out = TcRate::R24;
    else if (fabs(fps - 25.0) < 0.01) out = TcRate::R25;
    else if (fabs(fps - 29.97) < 0.01) out = TcRate::R2997DF;
    else if (fabs(fps - 30.0) < 0.01) out = TcRate::R30;
    else return false;
    return true;
}

bool parseTc(const char* s, Timecode& out) {
    if (!s || strlen(s) < 11) return false;
    unsigned h, m, sec, f;
    if (sscanf(s, "%2u:%2u:%2u%*[:;]%2u", &h, &m, &sec, &f) != 4) return false;
    if (h > 23 || m > 59 || sec > 59 || f > 29) return false;
    out.h = (uint8_t)h; out.m = (uint8_t)m; out.s = (uint8_t)sec; out.f = (uint8_t)f;
    return true;
}

void formatTc(const Timecode& tc, char out[12]) {
    snprintf(out, 12, "%02u:%02u:%02u:%02u", tc.h, tc.m, tc.s, tc.f);
}

}  // namespace arc
