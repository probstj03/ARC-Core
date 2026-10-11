#pragma once
// Phase-locked LTC generator.
//
// The generator is time driven: before it starts a frame it asks the
// OutputClock when the first sample of that frame will reach the DAC and
// compares this with the frame start of the Timeline. Errors of 0.75 samples
// or more are corrected by dropping or repeating one sample (max. one per
// frame, 20.8 µs at 48 kHz); larger errors (start, jumps) realign directly
// inside the frame. The ESP32-S3 has no APLL for fine clock tuning, so this
// sample slipping is the phase loop.
#include <Arduino.h>
#include "arc_timeline.h"

namespace arc {

// Maps output sample indices to esp_timer time.
//
// After every blocking I2S write the writer reports (frames written, esp now).
// A write returns as soon as a DMA buffer is free, so the earliest returns per
// block are the most precise; a line fit through the per-block minima gives
// rate and offset. The constant queue + DAC delay is not part of this model,
// it is the calibrated output latency (see engine).
class OutputClock {
public:
    void reset(double sampleRate);
    void onWrite(uint64_t framesWritten, int64_t espUsAfterWrite);
    bool valid() const { return _valid; }
    double espUsOfSample(uint64_t n) const;   // without output latency
    double sampleRate() const { return _fs; }
    double measuredRate() const;              // samples per second (fit)

private:
    static constexpr int BLOCKS = 32;
    static constexpr uint64_t BLOCK_SAMPLES = 12000;  // 250 ms at 48 kHz
    double _fs = 48000.0;
    double _usPerSample = 1e6 / 48000.0;
    uint64_t _blockStart = 0;
    double _blockMin = 0;
    bool _blockHasData = false;
    double _bn[BLOCKS] = {0}, _bb[BLOCKS] = {0};
    int _count = 0, _head = 0;
    double _a = 0, _c = 0;   // esp(n) = _a + _c * n  (n relative to _n0)
    uint64_t _n0 = 0;
    bool _valid = false;
    void _fit();
};

class LtcGenerator {
public:
    void begin(double outRate);
    void setTimeline(const Timeline& tl);
    Timeline timeline() const;
    void setUserBits(uint32_t displayOrder);
    void setLatencyUs(double us) { _latencyUs = us; }
    double latencyUs() const { return _latencyUs; }

    // Renders `n` mono samples starting at absolute output sample `startIndex`.
    void render(int16_t* out, size_t n, uint64_t startIndex, const OutputClock& clk);

    // State of the frame currently being output (for displays / events).
    Timecode currentTc() const;
    bool running() const { return !_silent; }
    double lastPhaseErrorUs() const { return _lastErrUs; }
    uint32_t slips() const { return _slips; }
    uint32_t realigns() const { return _realigns; }
    // Output sample index at which the current frame started.
    uint64_t currentFrameStartIndex() const { return _frameStartIndex; }

private:
    void _startFrame(uint64_t index, const OutputClock& clk);
    void _ensureEncoder(TcRate rate, double speed);
    double _espOut(uint64_t index, const OutputClock& clk) const;

    mutable portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    Timeline _tl;
    bool _tlChanged = true;
    volatile uint32_t _userBits = 0;
    volatile bool _userBitsDirty = true;
    double _outRate = 48000.0;
    double _latencyUs = 21800.0;

    void* _enc = nullptr;          // LTCEncoder*
    TcRate _encRate = TcRate::R25;
    double _encSpeed = 0.0;
    uint8_t* _buf = nullptr;
    int _len = 0, _pos = 0, _hold = 0;
    bool _silent = true;
    bool _needRealign = true;
    uint32_t _frame = 0;
    Timecode _tc;
    uint64_t _frameStartIndex = 0;
    bool _clkWasValid = false;
    double _lastErrUs = 0;
    uint32_t _slips = 0, _realigns = 0;
    uint8_t _silence[64] = {0};
};

}  // namespace arc
