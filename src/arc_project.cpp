#include "arc_project.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include "arc_events.h"

namespace arc {
namespace project {

namespace {

constexpr const char* PATH = "/project.json";
constexpr int EVENTS_PER_CHUNK = 12;

SemaphoreHandle_t s_mtx = nullptr;
JsonDocument s_doc;            // { code, production, director, camera, songtitle, ..., shotlist: [...] }
String s_rx;                   // chunks of an incoming project_put
int s_rxNext = 0;
String s_rxCode;

void lock() { xSemaphoreTake(s_mtx, portMAX_DELAY); }
void unlock() { xSemaphoreGive(s_mtx); }

bool loadFile() {
    File f = LittleFS.open(PATH, "r");
    if (!f) return false;
    JsonDocument doc;
    const bool ok = deserializeJson(doc, f) == DeserializationError::Ok;
    f.close();
    if (ok) s_doc = doc;
    return ok;
}

void sendText(AsyncWebSocketClient* client, JsonDocument& doc) {
    String out;
    serializeJson(doc, out);
    client->text(out);
}

bool isNumber(const String& s) {
    if (s.isEmpty()) return false;
    for (char c : s) if (!isdigit((unsigned char)c)) return false;
    return true;
}

// "01" == "1", but "6B" != "6".
bool same(JsonVariantConst v, const String& s) {
    const String a = v.is<const char*>() ? String(v.as<const char*>()) : String(v.as<long>());
    if (a == s) return true;
    return isNumber(a) && isNumber(s) && a.toInt() == s.toInt();
}

struct PullCtx {
    AsyncWebSocketClient* client;
    JsonDocument batch;
    int n = 0;
    uint32_t last = 0;
};

void flushBatch(PullCtx& c, bool done) {
    c.batch["type"] = "log_chunk";
    c.batch["lastId"] = c.last;
    c.batch["done"] = done;
    if (!c.batch["events"].is<JsonArray>()) c.batch["events"].to<JsonArray>();
    sendText(c.client, c.batch);
    c.batch.clear();
    c.n = 0;
}

bool collect(JsonDocument& ev, void* ctx) {
    PullCtx& c = *(PullCtx*)ctx;
    if (!c.batch["events"].is<JsonArray>()) c.batch["events"].to<JsonArray>();
    c.batch["events"].as<JsonArray>().add(ev.as<JsonObjectConst>());
    c.last = ev["id"] | c.last;
    if (++c.n >= EVENTS_PER_CHUNK) flushBatch(c, false);
    return true;
}

}  // namespace

bool begin() {
    s_mtx = xSemaphoreCreateMutex();
    lock();
    const bool ok = loadFile();
    unlock();
    if (ok) Serial.printf("[ARC] Offline-Projekt '%s' mit %d Shots geladen\n", code().c_str(), shotCount());
    return ok;
}

String code() {
    lock();
    const String c = s_doc["code"] | "";
    unlock();
    return c;
}

int shotCount() {
    lock();
    const int n = s_doc["shotlist"].is<JsonArray>() ? (int)s_doc["shotlist"].size() : 0;
    unlock();
    return n;
}

bool shotMessage(int index, JsonDocument& out) {
    lock();
    JsonArrayConst list = s_doc["shotlist"].as<JsonArrayConst>();
    if (list.isNull() || index < 0 || index >= (int)list.size()) { unlock(); return false; }
    out.clear();
    out["type"] = "shot";
    for (const char* k : {"code", "production", "director", "camera", "songtitle"}) {
        if (!s_doc[k].isNull()) out[k] = s_doc[k];
    }
    for (JsonPairConst kv : list[index].as<JsonObjectConst>()) out[kv.key()] = kv.value();
    out["offlineIndex"] = index;
    unlock();
    return true;
}

int indexOf(const String& scene, const String& shot) {
    lock();
    int found = -1;
    JsonArrayConst list = s_doc["shotlist"].as<JsonArrayConst>();
    for (size_t i = 0; i < list.size() && found < 0; i++) {
        if (same(list[i]["scene"], scene) && same(list[i]["shot"], shot)) found = (int)i;
    }
    unlock();
    return found;
}

bool handleMessage(const String& type, JsonDocument& doc, AsyncWebSocketClient* client) {
    if (type == "project_put") {
        const int index = doc["index"] | 0;
        const int total = doc["total"] | 1;
        if (index == 0) { s_rx = ""; s_rxNext = 0; s_rxCode = doc["code"] | ""; }
        if (index != s_rxNext) { s_rx = ""; s_rxNext = 0; return true; }   // out of order: drop
        s_rx += doc["data"] | "";
        s_rxNext++;
        if (s_rxNext < total) return true;
        JsonDocument parsed;
        JsonDocument ack;
        ack["type"] = "project_ack";
        ack["code"] = s_rxCode;
        if (deserializeJson(parsed, s_rx) != DeserializationError::Ok) {
            ack["ok"] = false;
        } else {
            lock();
            File f = LittleFS.open(PATH, "w");
            if (f) { f.print(s_rx); f.close(); }
            s_doc = parsed;
            unlock();
            ack["ok"] = (bool)f;
            ack["shots"] = shotCount();
            Serial.printf("[ARC] Offline-Projekt '%s' gespeichert (%d Shots)\n", s_rxCode.c_str(), shotCount());
        }
        s_rx = "";
        s_rxNext = 0;
        sendText(client, ack);
        return true;
    }
    if (type == "log_pull") {
        PullCtx ctx;
        ctx.client = client;
        ctx.last = doc["since"] | 0;
        events::forEachAfter(doc["since"] | 0, collect, &ctx);
        flushBatch(ctx, true);
        return true;
    }
    if (type == "log_ack") return true;   // log is kept (bounded, rotated)
    return false;
}

}  // namespace project
}  // namespace arc
