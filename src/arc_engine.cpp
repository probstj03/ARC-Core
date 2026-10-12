#include "arc_engine.h"
#include <Preferences.h>
#include <driver/i2s.h>
#include <esp_timer.h>
#include <algorithm>
#include "arc_clock.h"

namespace arc {
namespace engine {

namespace {

Config s_cfg;
volatile Mode s_mode = Mode::FreeRun;
volatile bool s_modePending = false;
volatile Mode s_reqMode = Mode::FreeRun;
volatile TcRate s_rate = TcRate::R25;

LtcGenerator s_gen;
OutputClock s_clk;
volatile bool s_external = false;
volatile bool s_userMute = false;
volatile uint32_t s_txBits = 0;
volatile uint32_t s_rxBits = 0;

void (*s_statusCb)() = nullptr;
void (*s_jamCb)(const JamStatus&) = nullptr;
volatile bool s_statusDirty = false;

// ── latency (loopback calibration) ───────────────────────────────────────
constexpr float DEFAULT_LATENCY_US = 21800.0f;   // 4 x 256 DMA frames @ 48 kHz + DAC
volatile float s_latencyUs = DEFAULT_LATENCY_US;
volatile bool s_latCalibrated = false;
float s_savedLatencyUs = 0;
int64_t s_lastLatSaveMs = 0;
int32_t s_loopSamples[25];
int s_loopCount = 0;
int s_loopGoodRounds = 0;

// ── decoder / jam state ─────────────────────────────────────────────────
portMUX_TYPE s_decMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t s_lastRxFrame = 0;        // frame that started at s_lastRxEsp
int64_t s_lastRxEsp = 0;
TcRate s_lastRxRate = TcRate::R25;
bool s_haveRx = false;

JamStatus s_jam;
uint32_t s_prevFrames = 0;
bool s_prevValid = false;
int64_t s_offsets[32];
int s_offCount = 0;
// verification samples: (seconds since start, residual µs)
constexpr int VERIFY_S = 30;
float s_vx[VERIFY_S * 30 + 8], s_vy[VERIFY_S * 30 + 8];
int s_vCount = 0;
int64_t s_verifyStartNs = 0;

void markStatus() { s_statusDirty = true; }

bool isDecodeMode(Mode m) { return m == Mode::Slave || m == Mode::JamFree || m == Mode::JamRec; }
bool isGenMode(Mode m) { return m == Mode::FreeRun || m == Mode::RecordRun || m == Mode::Playback; }

void applyMute() {
    const bool mute = isDecodeMode(s_mode) || s_jam.muteHeld || s_userMute;
    digitalWrite(s_cfg.dacMute, mute ? LOW : HIGH);
}

int64_t medianOf(int64_t* v, int n) {
    std::sort(v, v + n);
    return v[n / 2];
}

void loadLatency() {
    Preferences p;
    p.begin("arccore", true);
    const float v = p.getFloat("latUs", 0.0f);
    p.end();
    if (v > 1000.0f && v < 100000.0f) {
        s_latencyUs = v;
        s_savedLatencyUs = v;
        s_latCalibrated = true;
    }
    s_gen.setLatencyUs(s_latencyUs);
}

void saveLatencyIfNeeded() {
    if (!s_latCalibrated) return;
    if (fabsf(s_latencyUs - s_savedLatencyUs) < 20.0f) return;
    if (millis() - s_lastLatSaveMs < 60000) return;
    s_lastLatSaveMs = millis();
    Preferences p;
    p.begin("arccore", false);
    p.putFloat("latUs", s_latencyUs);
    p.end();
    s_savedLatencyUs = s_latencyUs;
}

// Own output heard on J2: compare the observed frame boundary with the timeline.
void loopbackSample(const DecodedFrame& f) {
    if (!isGenMode(s_mode) || s_jam.muteHeld || s_userMute) return;
    const Timeline tl = s_gen.timeline();
    if (tl.rate != f.rate) return;
    const int64_t observed = clock::wallNsAtEsp(f.endEspUs);
    uint32_t frame;
    int64_t into;
    if (!tl.frameAt(observed - 1000000, frame, into)) return;   // TC must be running
    const uint32_t next = (tcToFrames(f.tc, f.rate) + 1) % framesPerDay(f.rate);
    const int64_t expected = tl.frameStartNs(next, observed);
    const int64_t deltaNs = observed - expected;
    if (deltaNs > 3000000 || deltaNs < -3000000) return;          // not our signal / not locked
    s_loopSamples[s_loopCount++] = (int32_t)(deltaNs / 1000);
    if (s_loopCount < 25) return;
    int64_t tmp[25];
    for (int i = 0; i < 25; i++) tmp[i] = s_loopSamples[i];
    const int64_t med = medianOf(tmp, 25);
    s_loopCount = 0;
    // Output is late by `med` µs -> the generator has to start earlier.
    float lat = (float)s_gen.latencyUs() + 0.5f * (float)med;
    if (lat < 2000.0f) lat = 2000.0f;
    if (lat > 120000.0f) lat = 120000.0f;
    s_gen.setLatencyUs(lat);
    // Only the internal 48 kHz path is stored; an external output (ARC-Play
    // playback) has its own buffers and is tracked live only.
    if (s_external) return;
    s_latencyUs = lat;
    if (llabs(med) < 60) { if (++s_loopGoodRounds >= 3) s_latCalibrated = true; }
    else s_loopGoodRounds = 0;
}

void finishJam(bool ok) {
    s_jam.state = ok ? JamStatus::State::Done : JamStatus::State::Failed;
    if (ok) {
        s_jam.lastJamNs = clock::nowNs();
        s_jam.muteHeld = true;
    }
    if (s_jamCb) s_jamCb(s_jam);
    markStatus();
}

void verifyFinish() {
    // Residual: median; drift: least squares slope of residual over time.
    static int64_t tmp[VERIFY_S * 30 + 8];   // static: too large for the decoder task stack
    for (int i = 0; i < s_vCount; i++) tmp[i] = (int64_t)s_vy[i];
    const int64_t med = s_vCount ? medianOf(tmp, s_vCount) : 0;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < s_vCount; i++) {
        sx += s_vx[i]; sy += s_vy[i]; sxx += (double)s_vx[i] * s_vx[i]; sxy += (double)s_vx[i] * s_vy[i];
    }
    const double den = s_vCount * sxx - sx * sx;
    const double slopeUsPerS = (s_vCount > 10 && den != 0) ? (s_vCount * sxy - sx * sy) / den : 0.0;
    s_jam.ppm = (float)slopeUsPerS;               // µs per s == ppm
    s_jam.residualUs = (int32_t)med;
    clock::adjustTrimNs(-med * 1000LL);           // remove the remaining offset
    int8_t aging = clock::agingOffset();
    if (fabs(slopeUsPerS) > 0.1 && fabs(slopeUsPerS) < 12.0) {
        // Positive slope: our RTC runs fast -> larger aging value slows it down.
        int v = aging + (int)lround(slopeUsPerS / 0.1);
        v = v > 127 ? 127 : (v < -128 ? -128 : v);
        aging = (int8_t)v;
        clock::setAgingOffset(aging);
    }
    s_jam.aging = aging;
    Serial.printf("[ARC] Jam geprüft: Rest %ld us, Drift %.2f ppm, Aging %d\n", (long)s_jam.residualUs,
                  s_jam.ppm, (int)aging);
    finishJam(true);
    s_reqMode = Mode::FreeRun;
    s_modePending = true;
}

void onFrame(const DecodedFrame& f) {
    s_rxBits = f.userBits;
    const uint32_t frames = tcToFrames(f.tc, f.rate);
    const uint32_t next = (frames + 1) % framesPerDay(f.rate);
    portENTER_CRITICAL(&s_decMux);
    s_lastRxFrame = next;
    s_lastRxEsp = f.endEspUs;
    s_lastRxRate = f.rate;
    s_haveRx = true;
    portEXIT_CRITICAL(&s_decMux);

    loopbackSample(f);

    if (s_mode != Mode::JamFree && s_mode != Mode::JamRec) return;

    const int64_t nominalUs = frameDurationNs(f.rate) / 1000;
    const bool periodOk = f.periodUs > nominalUs * 9 / 10 && f.periodUs < nominalUs * 11 / 10;
    const bool continuous = s_prevValid && frames == (s_prevFrames + 1) % framesPerDay(f.rate);
    s_prevFrames = frames;
    s_prevValid = true;
    const int64_t localAtEdge = clock::wallNsAtEsp(f.endEspUs);
    const int64_t masterAtEdge = Timeline::freeRun(f.rate).frameStartNs(next, localAtEdge);

    if (s_jam.state == JamStatus::State::Verifying) {
        if (!continuous || !periodOk) return;
        const float x = (float)((localAtEdge - s_verifyStartNs) / 1000000000.0);
        if (s_vCount < (int)(sizeof(s_vx) / sizeof(s_vx[0]))) {
            s_vx[s_vCount] = x;
            s_vy[s_vCount] = (float)((localAtEdge - masterAtEdge) / 1000);
            s_vCount++;
        }
        const int64_t elapsedS = (localAtEdge - s_verifyStartNs) / 1000000000LL;
        s_jam.verifyRemainingS = elapsedS < VERIFY_S ? (uint32_t)(VERIFY_S - elapsedS) : 0;
        if (elapsedS >= VERIFY_S) verifyFinish();
        return;
    }

    if (s_jam.state != JamStatus::State::Listening) return;
    if (continuous && periodOk && f.rateKnown) s_jam.okFrames++;
    else { s_jam.okFrames = 0; s_offCount = 0; }
    s_jam.rate = f.rate;
    s_jam.rateKnown = f.rateKnown;
    if (s_jam.okFrames == 0) return;
    s_offsets[s_offCount % 32] = masterAtEdge - localAtEdge;
    s_offCount++;
    if (s_jam.okFrames < 75) return;

    if (f.rate != s_rate) { s_rate = f.rate; markStatus(); }
    if (s_mode == Mode::JamFree) {
        // The master runs Free Run: its TC is its time of day.
        const int n = s_offCount < 32 ? s_offCount : 32;
        int64_t tmp[32];
        for (int i = 0; i < n; i++) tmp[i] = s_offsets[i];
        const int64_t off = medianOf(tmp, n);
        Serial.printf("[ARC] Jam-Lock: Abweichung %lld us, stelle RTC\n", (long long)(off / 1000));
        clock::setTimeAt(localAtEdge + off, f.endEspUs);
        s_jam.state = JamStatus::State::Verifying;
        s_jam.verifyRemainingS = VERIFY_S;
        s_verifyStartNs = clock::nowNs();
        s_vCount = 0;
        s_prevValid = false;
        markStatus();
    } else {
        // Jam -> Record Run: continue the master's TC from this frame boundary.
        s_gen.setTimeline(Timeline::anchored(f.rate, next, localAtEdge));
        finishJam(true);
        s_reqMode = Mode::RecordRun;
        s_modePending = true;
    }
}

// ── I2S output task ─────────────────────────────────────────────────────
bool s_driverInstalled = false;

void installDriver() {
    i2s_config_t c = {};
    c.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    c.sample_rate = s_cfg.sampleRate;
    c.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    c.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    c.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    c.dma_buf_count = 4;
    c.dma_buf_len = 256;
    c.use_apll = false;
    c.tx_desc_auto_clear = true;
    i2s_driver_install(I2S_NUM_0, &c, 0, nullptr);
    i2s_pin_config_t p = {};
    p.mck_io_num = I2S_PIN_NO_CHANGE;
    p.bck_io_num = s_cfg.dacBck;
    p.ws_io_num = s_cfg.dacLrck;
    p.data_out_num = s_cfg.dacDin;
    p.data_in_num = I2S_PIN_NO_CHANGE;
    i2s_set_pin(I2S_NUM_0, &p);
    i2s_zero_dma_buffer(I2S_NUM_0);
    s_driverInstalled = true;
}

void outTask(void*) {
    static int16_t mono[256];
    static int16_t stereo[512];
    uint64_t written = 0;
    for (;;) {
        if (s_external) {
            if (s_driverInstalled) { i2s_driver_uninstall(I2S_NUM_0); s_driverInstalled = false; }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (!s_driverInstalled) {
            installDriver();
            written = 0;
            s_clk.reset(s_cfg.sampleRate);
            s_gen.begin(s_cfg.sampleRate);
            s_gen.setLatencyUs(s_latencyUs);
        }
        s_gen.render(mono, 256, written, s_clk);
        // Channel mask from the device config (slate: both, play: right = J2).
        const bool left = s_cfg.ltcChannels & 1, right = s_cfg.ltcChannels & 2;
        for (int i = 0; i < 256; i++) {
            stereo[2 * i] = left ? mono[i] : 0;
            stereo[2 * i + 1] = right ? mono[i] : 0;
        }
        size_t bytes = 0;
        i2s_write(I2S_NUM_0, stereo, sizeof(stereo), &bytes, portMAX_DELAY);
        written += bytes / 4;
        s_clk.onWrite(written, esp_timer_get_time());
    }
}

// Converts the timeline to another rate, keeping the TC that is on air.
Timeline retimed(const Timeline& tl, TcRate r) {
    const int64_t now = clock::nowNs();
    uint32_t frame;
    int64_t into;
    const bool runs = tl.frameAt(now, frame, into);
    const Timecode tc = framesToTc(frame, tl.rate);
    switch (tl.kind) {
        case Timeline::Kind::FreeRun: return Timeline::freeRun(r);
        case Timeline::Kind::Anchored:
            if (runs) return Timeline::anchored(r, tcToFrames(tc, r), now, tl.speed);
            return Timeline::stopped(r, tcToFrames(tc, r));
        default: return Timeline::stopped(r, tcToFrames(tc, r));
    }
}

void applyMode(Mode m) {
    const Mode old = s_mode;
    s_mode = m;
    const Timeline cur = s_gen.timeline();
    switch (m) {
        case Mode::FreeRun:
            s_gen.setTimeline(Timeline::freeRun(s_rate));
            break;
        case Mode::RecordRun:
            // From a jam the anchored timeline is already set; otherwise freeze the current TC.
            if (old == Mode::JamRec && cur.kind == Timeline::Kind::Anchored) break;
            s_gen.setTimeline(cur.frozenAt(clock::nowNs()));
            break;
        case Mode::Playback:
            s_gen.setTimeline(cur.frozenAt(clock::nowNs()));
            break;
        case Mode::Slave:
        case Mode::JamFree:
        case Mode::JamRec:
            s_gen.setTimeline(cur.frozenAt(clock::nowNs()));
            if (m != Mode::Slave) {
                s_jam.state = JamStatus::State::Listening;
                s_jam.okFrames = 0;
                s_offCount = 0;
                s_prevValid = false;
            }
            break;
    }
    applyMute();
    markStatus();
}

}  // namespace

// ── public API ───────────────────────────────────────────────────────────
void begin(const Config& cfg, TcRate initialRate) {
    s_cfg = cfg;
    s_rate = initialRate;
    pinMode(cfg.dacMute, OUTPUT);
    digitalWrite(cfg.dacMute, LOW);
    loadLatency();
    s_clk.reset(cfg.sampleRate);
    s_gen.begin(cfg.sampleRate);
    s_gen.setTimeline(Timeline::freeRun(initialRate));
    s_mode = Mode::FreeRun;
    xTaskCreatePinnedToCore(outTask, "ARCLTCOUT", 6144, nullptr, configMAX_PRIORITIES - 3, nullptr, 1);
    ltcin::begin(cfg.ltcIn, onFrame);
    applyMute();
}

void loop() {
    if (s_modePending) {
        s_modePending = false;
        applyMode(s_reqMode);
    }
    // A scheduled stop has passed: turn it into a plain frozen timeline.
    const Timeline tl = s_gen.timeline();
    static bool lastRunning = false;
    const int64_t now = clock::nowNs();
    if (tl.kind == Timeline::Kind::Anchored && tl.stopAtNs != INT64_MAX && now >= tl.stopAtNs) {
        s_gen.setTimeline(tl.frozenAt(tl.stopAtNs - 1));
    }
    const bool r = running();
    if (r != lastRunning) { lastRunning = r; markStatus(); }
    saveLatencyIfNeeded();
    if (s_statusDirty) {
        s_statusDirty = false;
        if (s_statusCb) s_statusCb();
    }
}

void requestMode(Mode m) {
    s_reqMode = m;
    s_modePending = true;
}

Mode mode() { return s_mode; }

const char* modeKey(Mode m) {
    switch (m) {
        case Mode::FreeRun:   return "gen_free";
        case Mode::RecordRun: return "gen_rec";
        case Mode::Playback:  return "playback";
        case Mode::Slave:     return "slave";
        case Mode::JamFree:   return "jam_free";
        case Mode::JamRec:    return "jam_rec";
    }
    return "gen_free";
}

bool modeFromKey(const char* key, Mode& out) {
    if (!key) return false;
    const String k(key);
    if (k == "gen_free") out = Mode::FreeRun;
    else if (k == "gen_rec") out = Mode::RecordRun;
    else if (k == "playback") out = Mode::Playback;
    else if (k == "slave") out = Mode::Slave;
    else if (k == "jam_free") out = Mode::JamFree;
    else if (k == "jam_rec") out = Mode::JamRec;
    else return false;
    return true;
}

void setRate(TcRate r) {
    if (r == s_rate) return;
    s_rate = r;
    s_gen.setTimeline(retimed(s_gen.timeline(), r));
    markStatus();
}

TcRate rate() { return s_rate; }

void start(int64_t atNs) {
    if (s_mode != Mode::RecordRun && s_mode != Mode::Playback) return;
    if (!atNs) atNs = clock::nowNs();
    const Timeline tl = s_gen.timeline();
    uint32_t frame;
    int64_t into;
    if (tl.kind == Timeline::Kind::Anchored && tl.stopAtNs == INT64_MAX && tl.frameAt(atNs, frame, into)) return;
    tl.frameAt(atNs, frame, into);   // frozen frame (or the frame at a pending stop)
    s_gen.setTimeline(Timeline::anchored(s_rate, frame, atNs, tl.speed > 0 ? tl.speed : 1.0));
    markStatus();
}

void startWith(int64_t atNs, const Timecode& tc, double speed) {
    if (s_mode != Mode::RecordRun && s_mode != Mode::Playback) return;
    if (!atNs) atNs = clock::nowNs();
    s_gen.setTimeline(Timeline::anchored(s_rate, tcToFrames(tc, s_rate), atNs, speed > 0 ? speed : 1.0));
    markStatus();
}

void stop(int64_t atNs) {
    if (s_mode != Mode::RecordRun && s_mode != Mode::Playback) return;
    if (!atNs) atNs = clock::nowNs();
    const Timeline tl = s_gen.timeline();
    if (tl.kind != Timeline::Kind::Anchored) return;
    s_gen.setTimeline(tl.stoppingAt(atNs));
    markStatus();
}

bool running() {
    const Timeline tl = s_gen.timeline();
    uint32_t f;
    int64_t into;
    return isGenMode(s_mode) && tl.frameAt(clock::nowNs(), f, into);
}

void setTimecode(const Timecode& tc) {
    const Timeline tl = s_gen.timeline();
    if (tl.kind == Timeline::Kind::FreeRun) return;   // Free Run follows the clock
    const uint32_t frame = tcToFrames(tc, s_rate);
    if (running()) s_gen.setTimeline(Timeline::anchored(s_rate, frame, clock::nowNs(), tl.speed));
    else s_gen.setTimeline(Timeline::stopped(s_rate, frame));
    markStatus();
}

void setTimeline(const Timeline& tl) {
    s_rate = tl.rate;
    s_gen.setTimeline(tl);
    markStatus();
}

Timeline timeline() { return s_gen.timeline(); }

Timecode timecodeAt(int64_t wallNs, int64_t& intoNs) {
    if (isDecodeMode(s_mode)) {
        portENTER_CRITICAL(&s_decMux);
        const uint32_t frame = s_lastRxFrame;
        const int64_t esp = s_lastRxEsp;
        const TcRate r = s_lastRxRate;
        const bool have = s_haveRx;
        portEXIT_CRITICAL(&s_decMux);
        intoNs = 0;
        if (!have) return Timecode();
        const int64_t startNs = clock::wallNsAtEsp(esp);
        int64_t elapsed = wallNs - startNs;
        if (elapsed < 0 || elapsed > 500000000LL) elapsed = 0;   // signal lost: hold
        const int64_t k = framesIn(elapsed, r, &intoNs);
        return framesToTc((uint32_t)((frame + k) % framesPerDay(r)), r);
    }
    uint32_t frame;
    const Timeline tl = s_gen.timeline();
    tl.frameAt(wallNs, frame, intoNs);
    return framesToTc(frame, tl.rate);
}

Timecode current() {
    int64_t into;
    return timecodeAt(clock::nowNs(), into);
}

void setUserBits(uint32_t displayOrder) {
    s_txBits = displayOrder;
    s_gen.setUserBits(displayOrder);
}

uint32_t txUserBits() { return s_txBits; }
uint32_t rxUserBits() { return s_rxBits; }

JamStatus jam() { return s_jam; }

void confirmUnplug() {
    s_jam.muteHeld = false;
    applyMute();
    markStatus();
}

float outputLatencyUs() { return s_latencyUs; }
bool latencyCalibrated() { return s_latCalibrated; }
float phaseErrorUs() { return (float)s_gen.lastPhaseErrorUs(); }

void setExternalOutput(bool external) { s_external = external; }
bool internalOutputStopped() { return s_external && !s_driverInstalled; }

double sampleIndexAtWall(int64_t wallNs) {
    const double espOut = (double)clock::espUsAtWall(wallNs);
    return s_clk.sampleAtEspUs(espOut - s_gen.latencyUs());
}
LtcGenerator& generator() { return s_gen; }
OutputClock& outputClock() { return s_clk; }

void setMuted(bool muted) {
    s_userMute = muted;
    applyMute();
}

void onStatusChange(void (*cb)()) { s_statusCb = cb; }
void onJamEvent(void (*cb)(const JamStatus&)) { s_jamCb = cb; }

}  // namespace engine
}  // namespace arc
