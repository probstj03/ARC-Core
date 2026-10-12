#pragma once
// Timecode engine shared by ARC-Play and ARC-Slate.
//
//   * Timeline (arc_timeline) defines the TC as a function of wall-clock time.
//   * The phase-locked LtcGenerator renders it to the PCM5102 via I2S.
//   * The decoder reads J2. In generator modes the device hears its own
//     output (J2 in/out share one node on ARC_AP) and uses it to calibrate
//     the output latency continuously (loopback).
//   * Jam Sync: lock onto incoming LTC, set the RTC to µs precision, verify
//     for 30 s (residual + drift, DS3231 aging trim), then Free Run.
//
// All "at" parameters are wall-clock ns (arc::clock); 0 = now.
#include <Arduino.h>
#include "arc_ltc_gen.h"
#include "arc_ltc_in.h"
#include "arc_timeline.h"

namespace arc {
namespace engine {

enum class Mode : uint8_t { FreeRun, RecordRun, Playback, Slave, JamFree, JamRec };

struct Config {
    int dacBck, dacLrck, dacDin, dacMute;   // PCM5102 (XSMT: HIGH = on)
    int ltcIn;
    uint32_t sampleRate = 48000;
    uint8_t ltcChannels = 3;                // bit 0 = left, bit 1 = right (internal output)
};

struct JamStatus {
    enum class State : uint8_t { Idle, Listening, Verifying, Done, Failed };
    State state = State::Idle;
    uint32_t okFrames = 0;      // consecutive good frames while listening
    TcRate rate = TcRate::R25;
    bool rateKnown = false;
    int32_t residualUs = 0;     // after verification
    float ppm = 0;              // local RTC vs master (before aging trim)
    int8_t aging = 0;           // DS3231 aging register after trim
    bool muteHeld = false;      // TC output stays muted until the cable is unplugged
    int64_t lastJamNs = 0;      // wall clock of the last successful jam
    uint32_t verifyRemainingS = 0;
};

void begin(const Config& cfg, TcRate initialRate);
void loop();                    // call from the Arduino loop

void requestMode(Mode m);
Mode mode();
const char* modeKey(Mode m);    // "gen_free", "gen_rec", "playback", "slave", "jam_free", "jam_rec"
bool modeFromKey(const char* key, Mode& out);

void setRate(TcRate r);
TcRate rate();

// Record Run (and Playback) transport. `atNs` schedules exactly on the wall clock.
void start(int64_t atNs = 0);
// Start at `atNs` with TC `tc` (frame running at that moment) and `speed`
// (e.g. a slate following ARC-Play's song TC).
void startWith(int64_t atNs, const Timecode& tc, double speed);
void stop(int64_t atNs = 0);
bool running();
void setTimecode(const Timecode& tc);          // while stopped: frozen TC; running: re-anchor now
void setTimeline(const Timeline& tl);          // full control (ARC-Play playback)
Timeline timeline();

// TC on air at a wall-clock time (exact, from the timeline); `intoNs` = position in the frame.
Timecode timecodeAt(int64_t wallNs, int64_t& intoNs);
Timecode current();

void setUserBits(uint32_t displayOrder);
uint32_t txUserBits();
uint32_t rxUserBits();

JamStatus jam();
void confirmUnplug();

// Output latency (queue + DAC), calibrated via loopback; persisted in NVS.
float outputLatencyUs();
bool latencyCalibrated();
float phaseErrorUs();            // last generator phase error

// External output (ARC-Play playback mixer): the engine stops its own I2S
// task; the device renders generator() into its stream and reports writes to
// outputClock().
void setExternalOutput(bool external);
bool internalOutputStopped();    // the internal I2S driver is uninstalled (external may start)
// Output sample index that reaches the DAC at wall-clock `wallNs` (uses the calibrated latency).
double sampleIndexAtWall(int64_t wallNs);
LtcGenerator& generator();
OutputClock& outputClock();
void setMuted(bool muted);       // device-level mute request (e.g. upload)

void onStatusChange(void (*cb)());
void onJamEvent(void (*cb)(const JamStatus&));

}  // namespace engine
}  // namespace arc
