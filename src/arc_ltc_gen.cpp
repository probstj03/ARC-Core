#include "arc_ltc_gen.h"
#include <esp_timer.h>
#include "arc_clock.h"
#include "arc_libltc.h"
#include "arc_userbits.h"

namespace arc {

// ── OutputClock ─────────────────────────────────────────────────────────────
void OutputClock::reset(double sampleRate) {
    _fs = sampleRate;
    _usPerSample = 1e6 / sampleRate;
    _blockStart = 0;
    _blockHasData = false;
    _count = _head = 0;
    _valid = false;
}

void OutputClock::onWrite(uint64_t n, int64_t espUs) {
    // Offset of this observation against the nominal rate; the minimum per
    // block is the observation with the least scheduling delay.
    const double off = (double)espUs - (double)n * _usPerSample;
    if (!_blockHasData) {
        _blockStart = n;
        _blockMin = off;
        _blockHasData = true;
    } else if (off < _blockMin) {
        _blockMin = off;
    }
    if (n - _blockStart >= BLOCK_SAMPLES) {
        _bn[_head] = (double)(_blockStart + BLOCK_SAMPLES / 2);
        _bb[_head] = _blockMin;
        _head = (_head + 1) % BLOCKS;
        if (_count < BLOCKS) _count++;
        _blockHasData = false;
        _fit();
    }
    if (!_valid && _count == 0 && _blockHasData) {
        // Until the first block is complete: use the newest observation.
        _n0 = n;
        _a = (double)espUs;
        _c = _usPerSample;
    }
}

void OutputClock::_fit() {
    if (_count < 2) {
        const int i = (_head + BLOCKS - 1) % BLOCKS;
        _n0 = (uint64_t)_bn[i];
        _a = _bb[i] + _bn[i] * _usPerSample;
        _c = _usPerSample;
        _valid = true;
        return;
    }
    // Least squares b = p + q*n on the block minima (n relative to the newest).
    const int last = (_head + BLOCKS - 1) % BLOCKS;
    const double nRef = _bn[last];
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int k = 0; k < _count; k++) {
        const int i = (_head + BLOCKS - 1 - k) % BLOCKS;
        const double x = _bn[i] - nRef, y = _bb[i];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const double den = _count * sxx - sx * sx;
    const double q = den != 0 ? (_count * sxy - sx * sy) / den : 0.0;
    const double p = (sy - q * sx) / _count;
    _n0 = (uint64_t)nRef;
    _a = p + nRef * _usPerSample;     // esp at n0
    _c = _usPerSample + q;
    _valid = true;
}

double OutputClock::espUsOfSample(uint64_t n) const {
    return _a + ((double)(int64_t)(n - _n0)) * _c;
}

double OutputClock::measuredRate() const { return _c > 0 ? 1e6 / _c : _fs; }

// ── LtcGenerator ──────────────────────────────────────────────────────────
namespace {
LTC_TV_STANDARD toStd(TcRate r) {
    switch (r) {
        case TcRate::R24:     return LTC_TV_FILM_24;
        case TcRate::R2997DF: return LTC_TV_525_DF;
        case TcRate::R30:     return LTC_TV_525_60;
        default:              return LTC_TV_625_50;
    }
}
}  // namespace

void LtcGenerator::begin(double outRate) {
    _outRate = outRate;
    _needRealign = true;
}

void LtcGenerator::setTimeline(const Timeline& tl) {
    portENTER_CRITICAL(&_mux);
    _tl = tl;
    _tlChanged = true;
    portEXIT_CRITICAL(&_mux);
}

Timeline LtcGenerator::timeline() const {
    portENTER_CRITICAL(&_mux);
    const Timeline t = _tl;
    portEXIT_CRITICAL(&_mux);
    return t;
}

void LtcGenerator::setUserBits(uint32_t displayOrder) {
    _userBits = displayOrder;
    _userBitsDirty = true;
}

Timecode LtcGenerator::currentTc() const {
    portENTER_CRITICAL(&_mux);
    const Timecode t = _tc;
    portEXIT_CRITICAL(&_mux);
    return t;
}

void LtcGenerator::_ensureEncoder(TcRate rate, double speed) {
    if (_enc && rate == _encRate && speed == _encSpeed) return;
    if (_enc) ltc_encoder_free((LTCEncoder*)_enc);
    const LTC_TV_STANDARD std = toStd(rate);
    // One frame lasts outRate / (fps * speed) output samples.
    LTCEncoder* e = ltc_encoder_create(_outRate / speed, ltc_standard_fps(std), std, 0);
    ltc_encoder_set_filter(e, 25.0);    // SMPTE rise time 25 µs
    ltc_encoder_set_volume(e, -3.0);
    _enc = e;
    _encRate = rate;
    _encSpeed = speed;
    _userBitsDirty = true;
    _needRealign = true;
}

double LtcGenerator::_espOut(uint64_t index, const OutputClock& clk) const {
    if (clk.valid()) return clk.espUsOfSample(index) + _latencyUs;
    return (double)esp_timer_get_time() + _latencyUs;   // rough until the clock has data
}

void LtcGenerator::_startFrame(uint64_t index, const OutputClock& clk) {
    Timeline tl;
    portENTER_CRITICAL(&_mux);
    tl = _tl;
    if (_tlChanged) { _tlChanged = false; _needRealign = true; }
    portEXIT_CRITICAL(&_mux);

    _ensureEncoder(tl.rate, tl.speed);
    // Once the output clock model becomes valid, align precisely once.
    if (clk.valid() && !_clkWasValid) _needRealign = true;
    _clkWasValid = clk.valid();
    const int64_t wallNs = clock::wallNsAtEsp((int64_t)_espOut(index, clk));

    uint32_t frame = 0;
    int64_t intoNs = 0;
    const bool runs = tl.frameAt(wallNs, frame, intoNs);
    if (!runs) {
        // TC does not run: output silence in small chunks, show the frozen frame.
        _silent = true;
        _needRealign = true;
        _buf = _silence;
        _len = sizeof(_silence);
        _pos = 0;
        _hold = 0;
        portENTER_CRITICAL(&_mux);
        _tc = framesToTc(frame, tl.rate);
        portEXIT_CRITICAL(&_mux);
        return;
    }

    int skip = 0;
    const uint32_t expected = (_frame + 1) % framesPerDay(tl.rate);
    const int64_t halfFrameNs = frameDurationNs(tl.rate) / 2;
    if (!_silent && !_needRealign) {
        const int64_t errNs = wallNs - tl.frameStartNs(expected, wallNs);
        if (errNs > -halfFrameNs && errNs < halfFrameNs) {
            frame = expected;
            const double errSamples = (double)errNs * _outRate / 1e9;
            _lastErrUs = (double)errNs / 1000.0;
            if (errSamples >= 0.75) { skip = 1; _slips++; }         // late: drop one sample
            else if (errSamples <= -0.75) { _hold = 1; _slips++; }  // early: repeat one sample
        } else {
            _needRealign = true;
        }
    }
    if (_silent || _needRealign) {
        // Jump into the running frame at the right sample.
        skip = (int)((double)intoNs * _outRate / 1e9 + 0.5);
        _lastErrUs = 0;
        _realigns++;
        _needRealign = false;
    }

    LTCEncoder* e = (LTCEncoder*)_enc;
    if (_userBitsDirty) {
        _userBitsDirty = false;
        ltc_encoder_set_user_bits(e, reverseNibbles(_userBits));
    }
    const Timecode tc = framesToTc(frame, tl.rate);
    SMPTETimecode st;
    ltc_encoder_get_timecode(e, &st);
    st.hours = tc.h; st.mins = tc.m; st.secs = tc.s; st.frame = tc.f;
    ltc_encoder_set_timecode(e, &st);
    ltc_encoder_encode_frame(e);
    ltcsnd_sample_t* buf = nullptr;
    _len = ltc_encoder_get_bufferptr(e, &buf, 1);
    _buf = buf;
    _pos = skip < _len ? skip : _len - 1;
    _silent = false;
    _frame = frame;
    _frameStartIndex = index;
    portENTER_CRITICAL(&_mux);
    _tc = tc;
    portEXIT_CRITICAL(&_mux);
}

void LtcGenerator::render(int16_t* out, size_t n, uint64_t startIndex, const OutputClock& clk) {
    for (size_t i = 0; i < n; i++) {
        if (_pos >= _len) _startFrame(startIndex + i, clk);
        if (_silent) {
            out[i] = 0;
            _pos++;
            continue;
        }
        out[i] = (int16_t)(((int)_buf[_pos] - 128) << 8);
        if (_hold > 0) _hold--;
        else _pos++;
    }
}

}  // namespace arc
