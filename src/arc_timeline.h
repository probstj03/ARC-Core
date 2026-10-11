#pragma once
// TC as a function of wall-clock time (pure C++).
//
// Every device derives its timecode from the same deterministic formula, so
// two devices with jammed RTCs and the same timeline show the same TC without
// any connection. Times are wall-clock nanoseconds since 1970 (RTC local time).
//
//   FreeRun   TC = time of day
//   Anchored  frame `anchorFrame` starts at `anchorNs`, runs with `speed`,
//             optionally freezes at `stopAtNs` (Record Run, Playback)
//   Stopped   TC frozen at `anchorFrame`
#include <stdint.h>
#include "arc_timecode.h"

namespace arc {

struct Timeline {
    enum class Kind : uint8_t { Stopped, FreeRun, Anchored };

    Kind kind = Kind::Stopped;
    TcRate rate = TcRate::R25;
    double speed = 1.0;
    int64_t anchorNs = 0;
    uint32_t anchorFrame = 0;
    int64_t stopAtNs = INT64_MAX;

    static Timeline freeRun(TcRate r);
    static Timeline anchored(TcRate r, uint32_t frame, int64_t atNs, double speed = 1.0);
    static Timeline stopped(TcRate r, uint32_t frame);

    // Frame running at `wallNs`. Returns false if the TC does not run at that
    // moment (stopped, before the anchor, after stopAt); `frame` is then the
    // frozen frame to display and `intoNs` is 0.
    bool frameAt(int64_t wallNs, uint32_t& frame, int64_t& intoNs) const;

    // Wall-clock start of `frame`, the occurrence closest to `nearNs`.
    int64_t frameStartNs(uint32_t frame, int64_t nearNs) const;

    // Same timeline, but frozen from `atNs` on.
    Timeline stoppingAt(int64_t atNs) const;
    // Frozen copy at the frame running at `atNs`.
    Timeline frozenAt(int64_t atNs) const;
};

}  // namespace arc
