#include "arc_events.h"
#include <LittleFS.h>
#include <Preferences.h>

namespace arc {
namespace events {

namespace {
constexpr const char* LOG = "/events.jsonl";
constexpr const char* OLD = "/events.old.jsonl";
constexpr size_t MAX_BYTES = 256 * 1024;
SemaphoreHandle_t s_mtx = nullptr;
bool s_ok = false;
uint32_t s_nextId = 1;
}  // namespace

bool begin() {
    s_mtx = xSemaphoreCreateMutex();
    s_ok = LittleFS.begin(true);
    Preferences p;
    p.begin("arcevents", true);
    s_nextId = p.getUInt("next", 1);
    p.end();
    return s_ok;
}

uint32_t nextId() {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    const uint32_t id = s_nextId++;
    Preferences p;
    p.begin("arcevents", false);
    p.putUInt("next", s_nextId);
    p.end();
    xSemaphoreGive(s_mtx);
    return id;
}

uint32_t lastId() { return s_nextId > 0 ? s_nextId - 1 : 0; }

void forEachAfter(uint32_t sinceId, bool (*fn)(JsonDocument& ev, void* ctx), void* ctx) {
    if (!s_ok) return;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    bool go = true;
    for (const char* name : {OLD, LOG}) {
        if (!go) break;
        File f = LittleFS.open(name, "r");
        if (!f) continue;
        while (go && f.available()) {
            const String line = f.readStringUntil('\n');
            if (line.length() < 2) continue;
            JsonDocument ev;
            if (deserializeJson(ev, line) != DeserializationError::Ok) continue;
            if ((uint32_t)(ev["id"] | 0) <= sinceId) continue;
            go = fn(ev, ctx);
        }
        f.close();
    }
    xSemaphoreGive(s_mtx);
}

void append(JsonDocument& event) {
    if (!s_ok) return;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    File f = LittleFS.open(LOG, "r");
    const size_t sz = f ? f.size() : 0;
    if (f) f.close();
    if (sz > MAX_BYTES) {
        LittleFS.remove(OLD);
        LittleFS.rename(LOG, OLD);
    }
    f = LittleFS.open(LOG, "a");
    if (f) {
        serializeJson(event, f);
        f.print('\n');
        f.close();
    }
    xSemaphoreGive(s_mtx);
}

void print(Stream& out) {
    if (!s_ok) return;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (const char* name : {OLD, LOG}) {
        File f = LittleFS.open(name, "r");
        if (!f) continue;
        while (f.available()) out.write(f.read());
        f.close();
    }
    xSemaphoreGive(s_mtx);
}

void clear() {
    if (!s_ok) return;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    LittleFS.remove(LOG);
    LittleFS.remove(OLD);
    xSemaphoreGive(s_mtx);
}

size_t size() {
    if (!s_ok) return 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    File f = LittleFS.open(LOG, "r");
    const size_t sz = f ? f.size() : 0;
    if (f) f.close();
    xSemaphoreGive(s_mtx);
    return sz;
}

}  // namespace events
}  // namespace arc
