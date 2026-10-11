#include "arc_timeline.h"

namespace arc {

namespace {
constexpr int64_t NS_PER_DAY = 86400LL * 1000000000LL;

int64_t floorMod(int64_t a, int64_t b) {
    const int64_t m = a % b;
    return m < 0 ? m + b : m;
}
}  // namespace

Timeline Timeline::freeRun(TcRate r) {
    Timeline t;
    t.kind = Kind::FreeRun;
    t.rate = r;
    return t;
}

Timeline Timeline::anchored(TcRate r, uint32_t frame, int64_t atNs, double speed) {
    Timeline t;
    t.kind = Kind::Anchored;
    t.rate = r;
    t.anchorFrame = frame % framesPerDay(r);
    t.anchorNs = atNs;
    t.speed = speed > 0.0 ? speed : 1.0;
    return t;
}

Timeline Timeline::stopped(TcRate r, uint32_t frame) {
    Timeline t;
    t.kind = Kind::Stopped;
    t.rate = r;
    t.anchorFrame = frame % framesPerDay(r);
    return t;
}

bool Timeline::frameAt(int64_t wallNs, uint32_t& frame, int64_t& intoNs) const {
    intoNs = 0;
    switch (kind) {
        case Kind::FreeRun: {
            const int64_t nsOfDay = floorMod(wallNs, NS_PER_DAY);
            frame = frameAtNs(nsOfDay, rate, &intoNs);
            return true;
        }
        case Kind::Anchored: {
            if (wallNs < anchorNs) { frame = anchorFrame; return false; }
            const bool stoppedNow = wallNs >= stopAtNs;
            const int64_t t = stoppedNow ? stopAtNs - 1 : wallNs;
            const int64_t elapsed = (int64_t)((double)(t - anchorNs) * speed);
            int64_t into = 0;
            const int64_t k = framesIn(elapsed, rate, &into);
            frame = (uint32_t)((anchorFrame + k) % framesPerDay(rate));
            if (stoppedNow) return false;
            intoNs = (int64_t)((double)into / speed);
            return true;
        }
        default:
            frame = anchorFrame;
            return false;
    }
}

int64_t Timeline::frameStartNs(uint32_t frame, int64_t nearNs) const {
    const int64_t fpd = framesPerDay(rate);
    if (kind == Kind::FreeRun) {
        const int64_t dayStart = nearNs - floorMod(nearNs, NS_PER_DAY);
        int64_t best = dayStart + arc::frameStartNs(frame, rate);
        // The closest occurrence may be on the neighbouring day (midnight).
        const int64_t prev = best - NS_PER_DAY, next = best + NS_PER_DAY;
        auto dist = [nearNs](int64_t v) { return v > nearNs ? v - nearNs : nearNs - v; };
        if (dist(prev) < dist(best)) best = prev;
        if (dist(next) < dist(best)) best = next;
        return best;
    }
    // Anchored / stopped: relative frame index counted from the anchor.
    int64_t rel = floorMod((int64_t)frame - (int64_t)anchorFrame, fpd);
    // Pick the occurrence closest to nearNs (rel may wrap around the day).
    const int64_t elapsedNear = (int64_t)((double)(nearNs - anchorNs) * speed);
    const int64_t kNear = framesIn(elapsedNear > 0 ? elapsedNear : 0, rate, nullptr);
    const int64_t cycles = (kNear - rel + fpd / 2) / fpd;
    rel += cycles * fpd;
    return anchorNs + (int64_t)((double)framesDurationNs(rel, rate) / speed);
}

Timeline Timeline::stoppingAt(int64_t atNs) const {
    Timeline t = *this;
    t.stopAtNs = atNs;
    return t;
}

Timeline Timeline::frozenAt(int64_t atNs) const {
    uint32_t frame = 0;
    int64_t into = 0;
    frameAt(atNs, frame, into);
    return stopped(rate, frame);
}

}  // namespace arc
