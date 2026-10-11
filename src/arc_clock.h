#pragma once
// RTC-disciplined wall clock with microsecond resolution.
//
// The DS3231 1 Hz SQW output marks every second change with one edge. An ISR
// timestamps that edge with esp_timer; between edges the time is interpolated
// with the ESP32 timer, scaled by the measured ESP-µs per RTC second. Result:
// wall-clock time (RTC local time) without any I2C access on the hot path.
//
// After the RTC was set, the next second edge shows how precisely the DS3231
// took over the new time; the remaining error is kept as a software trim, so
// the clock is correct to a few µs immediately after setting.
#include <Arduino.h>
#include <RTClib.h>

namespace arc {
namespace clock {

struct Pins { int sda; int scl; int sqw; };

bool begin(const Pins& pins, bool sqwFalling);
bool rtcPresent();
bool sqwActive();                     // second edges arriving?
bool rtcLostPower();

int64_t nowNs();                      // wall-clock ns since 1970 (RTC local time)
int64_t nowUs();
int64_t wallNsAtEsp(int64_t espUs);   // wall clock at an esp_timer instant
int64_t espUsAtWall(int64_t wallNs);  // inverse
DateTime now();
uint32_t usInSecond();

// Sets the RTC so that the wall clock equals `wallNs` at esp_timer `espUs`
// (may be in the past). Blocks up to ~2.5 s.
void setTimeAt(int64_t wallNs, int64_t espUs);
void setTimeNs(int64_t wallNsNow);
// Coarse time from NTP / ARC-Studio. If the clock is only off by whole days
// (RTC lost power, then jammed), only the date is corrected and the jammed
// phase is kept. Returns true if the clock was changed.
bool applyCoarseTime(int64_t wallNsNow, int64_t toleranceNs, bool force);
void setTime(const DateTime& dt);

// Shifts the software trim (fine correction without touching the RTC).
void adjustTrimNs(int64_t deltaNs);
int64_t trimNs();

// DS3231 aging offset register (≈0.1 ppm per LSB at 25 °C, positive = slower).
int8_t agingOffset();
void setAgingOffset(int8_t value);
float temperature();

// Measures which SQW edge coincides with the second change (~2.5 s).
bool detectSqwEdge(bool& falling, int32_t& offsetMs);
void useEdge(bool falling);

double espUsPerRtcSecond();           // measured rate of the ESP timer vs the RTC

void lockI2C();
void unlockI2C();
RTC_DS3231& rtc();

}  // namespace clock
}  // namespace arc
