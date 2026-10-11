#include "arc_clock.h"
#include <Wire.h>
#include <esp_timer.h>
#include <driver/gpio.h>

namespace arc {
namespace clock {

namespace {

RTC_DS3231 s_rtc;
SemaphoreHandle_t s_i2cMtx = nullptr;
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
Pins s_pins{};
bool s_rtcOk = false;
bool s_lostPower = false;

volatile bool    s_falling    = true;
volatile int64_t s_edgeUs     = 0;     // esp_timer at the last second edge
volatile int64_t s_epochSec   = 0;     // RTC second that started at s_edgeUs
volatile bool    s_epochValid = false;
volatile uint32_t s_edgeCount = 0;
volatile int64_t s_lastRiseUs = 0;
volatile int64_t s_lastFallUs = 0;
volatile double  s_usPerSec   = 1000000.0;
volatile int64_t s_trimNs     = 0;

int64_t s_fallbackBaseSec = 0;
int64_t s_fallbackBaseUs  = 0;

void IRAM_ATTR isrSqw() {
    const int64_t t = esp_timer_get_time();
    const bool high = gpio_get_level((gpio_num_t)s_pins.sqw);
    if (high) s_lastRiseUs = t; else s_lastFallUs = t;
    if (high == s_falling) return;
    portENTER_CRITICAL_ISR(&s_mux);
    const int64_t d = t - s_edgeUs;
    // Track the ESP timer rate per RTC second (ignores missed / spurious edges).
    if (s_edgeUs && d > 999000 && d < 1001000) s_usPerSec = s_usPerSec + ((double)d - s_usPerSec) / 16.0;
    s_edgeUs = t;
    if (s_epochValid) s_epochSec = s_epochSec + 1;
    s_edgeCount = s_edgeCount + 1;
    portEXIT_CRITICAL_ISR(&s_mux);
}

bool waitForEdge(uint32_t timeoutMs) {
    const uint32_t c0 = s_edgeCount;
    const uint32_t t0 = millis();
    while (s_edgeCount == c0) {
        if (millis() - t0 > timeoutMs) return false;
        delay(1);
    }
    return true;
}

void resync() {
    if (!s_rtcOk) return;
    if (waitForEdge(1500)) {
        lockI2C();
        const DateTime dt = s_rtc.now();
        unlockI2C();
        portENTER_CRITICAL(&s_mux);
        s_epochSec = (int64_t)dt.unixtime();
        s_epochValid = true;
        portEXIT_CRITICAL(&s_mux);
    } else {
        lockI2C();
        const DateTime dt = s_rtc.now();
        unlockI2C();
        s_fallbackBaseSec = (int64_t)dt.unixtime();
        s_fallbackBaseUs = esp_timer_get_time();
        s_epochValid = false;
        Serial.println("[ARC] Keine SQW-Flanken – Zeitstempel nur sekundengenau");
    }
}

int64_t rawWallNsAtEsp(int64_t espUs) {
    if (!s_epochValid) {
        return s_fallbackBaseSec * 1000000000LL + (espUs - s_fallbackBaseUs) * 1000LL;
    }
    portENTER_CRITICAL(&s_mux);
    const int64_t e = s_edgeUs;
    const int64_t sec = s_epochSec;
    const double rate = s_usPerSec;
    portEXIT_CRITICAL(&s_mux);
    const double d = (double)(espUs - e);
    return sec * 1000000000LL + (int64_t)(d * (1.0e9 / rate));
}

}  // namespace

void lockI2C() { xSemaphoreTake(s_i2cMtx, portMAX_DELAY); }
void unlockI2C() { xSemaphoreGive(s_i2cMtx); }
RTC_DS3231& rtc() { return s_rtc; }
bool rtcPresent() { return s_rtcOk; }
bool rtcLostPower() { return s_lostPower; }
double espUsPerRtcSecond() { return s_usPerSec; }
int64_t trimNs() { return s_trimNs; }
void adjustTrimNs(int64_t deltaNs) { s_trimNs = s_trimNs + deltaNs; }

bool sqwActive() { return s_epochValid && (esp_timer_get_time() - s_edgeUs) < 1500000; }

bool begin(const Pins& pins, bool sqwFalling) {
    s_pins = pins;
    s_i2cMtx = xSemaphoreCreateMutex();
    s_falling = sqwFalling;
    Wire.begin(pins.sda, pins.scl, 400000);
    s_rtcOk = s_rtc.begin(&Wire);
    if (!s_rtcOk) {
        Serial.println("[ARC] DS3231 nicht gefunden");
        s_fallbackBaseUs = esp_timer_get_time();
        return false;
    }
    s_lostPower = s_rtc.lostPower();
    s_rtc.writeSqwPinMode(DS3231_SquareWave1Hz);
    pinMode(pins.sqw, INPUT_PULLUP);   // open drain
    attachInterrupt(digitalPinToInterrupt(pins.sqw), isrSqw, CHANGE);
    resync();
    return true;
}

void useEdge(bool falling) {
    s_falling = falling;
    s_edgeUs = 0;
    resync();
}

int64_t wallNsAtEsp(int64_t espUs) { return rawWallNsAtEsp(espUs) + s_trimNs; }
int64_t nowNs() { return wallNsAtEsp(esp_timer_get_time()); }
int64_t nowUs() { return nowNs() / 1000; }

int64_t espUsAtWall(int64_t wallNs) {
    const int64_t nowEsp = esp_timer_get_time();
    const int64_t nowWall = wallNsAtEsp(nowEsp);
    return nowEsp + (int64_t)((double)(wallNs - nowWall) * (s_usPerSec / 1.0e9));
}

DateTime now() { return DateTime((uint32_t)(nowNs() / 1000000000LL)); }
uint32_t usInSecond() { return (uint32_t)((nowNs() / 1000) % 1000000LL); }

void setTimeAt(int64_t wallNs, int64_t espUs) {
    if (!s_rtcOk) return;
    // Target clock: wall(t) = wallNs + (t - espUs). Writing the seconds
    // register restarts the DS3231 second countdown, so the write must happen
    // exactly at a whole second of the target clock.
    constexpr int64_t I2C_WRITE_US = 75;   // start + address + register + seconds byte at 400 kHz
    const int64_t offsetNs = wallNs - espUs * 1000LL;          // wall = esp*1000 + offset
    const int64_t nowEsp = esp_timer_get_time();
    int64_t nextSecNs = ((nowEsp * 1000LL + offsetNs) / 1000000000LL + 1) * 1000000000LL;
    int64_t writeAtEsp = (nextSecNs - offsetNs) / 1000LL - I2C_WRITE_US;
    if (writeAtEsp - nowEsp < 3000) {                          // too close: next second
        nextSecNs += 1000000000LL;
        writeAtEsp += 1000000;
    }
    while (esp_timer_get_time() < writeAtEsp - 2000) delay(1);
    lockI2C();
    while (esp_timer_get_time() < writeAtEsp - 200) { }
    portDISABLE_INTERRUPTS();   // only the last 200 µs
    while (esp_timer_get_time() < writeAtEsp) { }
    portENABLE_INTERRUPTS();
    s_rtc.adjust(DateTime((uint32_t)(nextSecNs / 1000000000LL)));
    unlockI2C();
    s_trimNs = 0;
    resync();
    // Fine trim: compare the observed second edge with the target clock.
    if (s_epochValid) {
        const int64_t edgeEsp = s_edgeUs;
        const int64_t targetAtEdge = edgeEsp * 1000LL + offsetNs;
        const int64_t rtcAtEdge = s_epochSec * 1000000000LL;
        s_trimNs = targetAtEdge - rtcAtEdge;
        Serial.printf("[ARC] RTC gestellt, Rest-Trim %lld us\n", (long long)(s_trimNs / 1000));
    }
    s_lostPower = false;
}

void setTimeNs(int64_t wallNsNow) { setTimeAt(wallNsNow, esp_timer_get_time()); }

bool applyCoarseTime(int64_t wallNsNow, int64_t toleranceNs, bool force) {
    const int64_t esp = esp_timer_get_time();
    const int64_t local = wallNsAtEsp(esp);
    const int64_t diff = wallNsNow - local;
    constexpr int64_t DAY = 86400LL * 1000000000LL;
    const int64_t days = (diff >= 0 ? diff + DAY / 2 : diff - DAY / 2) / DAY;
    const int64_t rest = diff - days * DAY;
    if (!force && days == 0 && llabs(rest) <= toleranceNs) return false;
    if (!force && days != 0 && llabs(rest) <= toleranceNs) {
        setTimeAt(local + days * DAY, esp);   // date only, phase unchanged
        return true;
    }
    setTimeAt(wallNsNow, esp);
    return true;
}
void setTime(const DateTime& dt) { setTimeNs((int64_t)dt.unixtime() * 1000000000LL); }

int8_t agingOffset() {
    if (!s_rtcOk) return 0;
    lockI2C();
    Wire.beginTransmission(0x68);
    Wire.write(0x10);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 1);
    const int8_t v = Wire.available() ? (int8_t)Wire.read() : 0;
    unlockI2C();
    return v;
}

void setAgingOffset(int8_t value) {
    if (!s_rtcOk) return;
    lockI2C();
    Wire.beginTransmission(0x68);
    Wire.write(0x10);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
    // Force a temperature conversion so the new offset is applied right away.
    Wire.beginTransmission(0x68);
    Wire.write(0x0E);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 1);
    const uint8_t ctrl = Wire.available() ? Wire.read() : 0;
    Wire.beginTransmission(0x68);
    Wire.write(0x0E);
    Wire.write(ctrl | 0x20);   // CONV
    Wire.endTransmission();
    unlockI2C();
}

float temperature() {
    if (!s_rtcOk) return NAN;
    lockI2C();
    const float t = s_rtc.getTemperature();
    unlockI2C();
    return t;
}

bool detectSqwEdge(bool& falling, int32_t& offsetMs) {
    if (!s_rtcOk) return false;
    lockI2C();
    uint8_t lastSec = s_rtc.now().second();
    unlockI2C();
    int64_t prevPollUs = esp_timer_get_time();
    const uint32_t t0 = millis();
    while (millis() - t0 < 2500) {
        lockI2C();
        const uint8_t sec = s_rtc.now().second();
        unlockI2C();
        const int64_t pollUs = esp_timer_get_time();
        if (sec != lastSec) {
            const int64_t changeUs = (prevPollUs + pollUs) / 2;
            delay(5);
            if (s_lastRiseUs == 0 && s_lastFallUs == 0) return false;
            const int64_t dRise = llabs(changeUs - s_lastRiseUs) % 1000000LL;
            const int64_t dFall = llabs(changeUs - s_lastFallUs) % 1000000LL;
            const int64_t eRise = dRise > 500000 ? 1000000 - dRise : dRise;
            const int64_t eFall = dFall > 500000 ? 1000000 - dFall : dFall;
            falling = eFall < eRise;
            offsetMs = (int32_t)((falling ? eFall : eRise) / 1000);
            return true;
        }
        lastSec = sec;
        prevPollUs = pollUs;
        delayMicroseconds(500);
    }
    return false;
}

}  // namespace clock
}  // namespace arc
