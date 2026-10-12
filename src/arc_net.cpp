#include "arc_net.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <sys/time.h>
#include <algorithm>
#include <vector>
#include "arc_clock.h"
#include "arc_engine.h"
#include "arc_events.h"
#include "arc_project.h"

namespace arc {
namespace net {

namespace {

constexpr const char* ARC_NET_SSID = "ARC-Net";
constexpr const char* TZ_EUROPE_BERLIN = "CET-1CEST,M3.5.0,M10.5.0/3";

struct KnownNet { String ssid; String pass; int priority; };

Config s_cfg;
AsyncWebServer s_server(80);
AsyncWebSocket s_ws("/ws");
SemaphoreHandle_t s_mtx = nullptr;
std::vector<KnownNet> s_known;
String s_arcPass;
String s_deviceId;
String s_ssid;
volatile Role s_role = Role::Offline;
volatile bool s_serverStarted = false;
volatile bool s_statusDirty = false;
volatile bool s_ntpRequested = false;
volatile int64_t s_setTimeUs = 0;

// ── known networks ───────────────────────────────────────────────────────
void loadKnown() {
    Preferences p;
    p.begin("wifi", true);
    const String json = p.getString("list", "");
    s_arcPass = p.getString("arcPass", s_cfg.arcNetPassword);
    p.end();
    s_known.clear();
    if (json.length()) {
        JsonDocument doc;
        if (deserializeJson(doc, json) == DeserializationError::Ok) {
            for (JsonObject n : doc.as<JsonArray>()) s_known.push_back({n["s"] | "", n["p"] | "", n["r"] | 0});
        }
        return;
    }
    for (int i = 0; i < s_cfg.defaultCount; i++) {
        s_known.push_back({s_cfg.defaults[i].ssid, s_cfg.defaults[i].pass, s_cfg.defaults[i].priority});
    }
}

void saveKnown() {
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (const auto& n : s_known) {
        JsonObject o = arr.add<JsonObject>();
        o["s"] = n.ssid; o["p"] = n.pass; o["r"] = n.priority;
    }
    String json;
    serializeJson(doc, json);
    Preferences p;
    p.begin("wifi", false);
    p.putString("list", json);
    p.putString("arcPass", s_arcPass);
    p.end();
}

String knownHash() {
    uint32_t h = 2166136261u;
    auto mix = [&h](const String& s) { for (char c : s) { h ^= (uint8_t)c; h *= 16777619u; } h ^= 0xFF; h *= 16777619u; };
    for (const auto& n : s_known) { mix(n.ssid); mix(n.pass); mix(String(n.priority)); }
    mix(s_arcPass);
    char buf[9];
    snprintf(buf, sizeof(buf), "%08lx", (unsigned long)h);
    return String(buf);
}

// ── messages ─────────────────────────────────────────────────────────────
void sendAll(JsonDocument& doc) {
    if (!s_serverStarted || s_ws.count() == 0) return;
    String out;
    serializeJson(doc, out);
    s_ws.textAll(out);
}

String isoTimestamp(int64_t wallNs) {
    const DateTime d((uint32_t)(wallNs / 1000000000LL));
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03d", d.year(), d.month(), d.day(), d.hour(),
             d.minute(), d.second(), (int)((wallNs / 1000000LL) % 1000));
    return String(buf);
}

void fillStatus(JsonDocument& doc, const char* type) {
    const int64_t now = clock::nowNs();
    doc["type"] = type;
    doc["device"] = s_cfg.device;
    doc["id"] = s_deviceId;
    doc["proto"] = 2;
    doc["fw"] = s_cfg.firmware;
    if (!s_cfg.ltcEngine) {
        doc["timestamp"] = isoTimestamp(now);
        doc["rtcUs"] = now / 1000;
        doc["sqw"] = clock::sqwActive();
        if (s_cfg.fillStatus) s_cfg.fillStatus(doc);
        return;
    }
    int64_t into = 0;
    const Timecode tc = engine::timecodeAt(now, into);
    char tcs[12];
    formatTc(tc, tcs);
    doc["gen_mode"] = engine::modeKey(engine::mode());
    doc["tc_mode"] = rateKey(engine::rate());
    doc["running"] = engine::running();
    doc["timecode"] = tcs;
    doc["subframeUs"] = (int32_t)(into / 1000);
    doc["timestamp"] = isoTimestamp(now);      // legacy (ms)
    doc["rtcUs"] = now / 1000;
    doc["sqw"] = clock::sqwActive();
    const engine::JamStatus j = engine::jam();
    JsonObject jam = doc["jam"].to<JsonObject>();
    static const char* const states[] = {"idle", "listening", "verifying", "done", "failed"};
    jam["state"] = states[(int)j.state];
    jam["okFrames"] = j.okFrames;
    jam["rate"] = rateKey(j.rate);
    jam["residualUs"] = j.residualUs;
    jam["ppm"] = j.ppm;
    jam["aging"] = j.aging;
    jam["muteHeld"] = j.muteHeld;
    jam["lastJamUs"] = j.lastJamNs / 1000;
    jam["verifyRemainingS"] = j.verifyRemainingS;
    JsonObject out = doc["output"].to<JsonObject>();
    out["latencyUs"] = engine::outputLatencyUs();
    out["calibrated"] = engine::latencyCalibrated();
    out["phaseErrUs"] = engine::phaseErrorUs();
    if (s_cfg.fillStatus) s_cfg.fillStatus(doc);
}

void sendHello(AsyncWebSocketClient* client) {
    JsonDocument doc;
    doc["type"] = "hello";
    doc["device"] = s_cfg.device;
    doc["id"] = s_deviceId;
    doc["fw"] = s_cfg.firmware;
    doc["proto"] = 2;
    JsonArray caps = doc["caps"].to<JsonArray>();
    for (int i = 0; i < s_cfg.capCount; i++) caps.add(s_cfg.caps[i]);
    doc["wifiHash"] = knownHash();
    doc["projectCode"] = project::code();
    doc["projectShots"] = project::shotCount();
    doc["lastEventId"] = events::lastId();
    String out;
    serializeJson(doc, out);
    client->text(out);
}

int64_t atNs(JsonDocument& doc) {
    // "at" = wall-clock µs (rtcUs of the device time base); 0 / missing = now.
    if (doc["at"].isNull()) return 0;
    return (int64_t)doc["at"].as<double>() * 1000LL;
}

void handleCommand(JsonDocument& doc) {
    const String cmd = doc["cmd"] | "";
    if (!s_cfg.ltcEngine) {
        if (cmd == "sync_rtc") s_ntpRequested = true;
        s_statusDirty = true;
        return;
    }
    if (cmd == "start" || cmd == "play") {
        Timecode tc;
        if (!doc["tc"].isNull() && parseTc(doc["tc"] | "", tc)) {
            engine::startWith(atNs(doc), tc, doc["speed"] | 1.0);
        } else {
            engine::start(atNs(doc));
        }
    }
    else if (cmd == "stop" || cmd == "pause") engine::stop(atNs(doc));
    else if (cmd == "reset") engine::setTimecode(Timecode());
    else if (cmd == "set_tc_mode") {
        TcRate r;
        if (rateFromKey(doc["val"] | "", r)) engine::setRate(r);
    } else if (cmd == "set_gen_mode") {
        engine::Mode m;
        if (engine::modeFromKey(doc["val"] | "", m)) engine::requestMode(m);
    } else if (cmd == "jam_confirm_unplug") engine::confirmUnplug();
    else if (cmd == "sync_rtc") s_ntpRequested = true;
    s_statusDirty = true;
}

void handleWifiConfig(JsonDocument& doc) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    if (doc["networks"].is<JsonArray>()) {
        s_known.clear();
        for (JsonObject n : doc["networks"].as<JsonArray>()) {
            s_known.push_back({n["ssid"] | "", n["password"] | "", n["priority"] | 0});
        }
    }
    if (!doc["arcNet"]["password"].isNull()) s_arcPass = doc["arcNet"]["password"].as<String>();
    saveKnown();
    const String hash = knownHash();
    xSemaphoreGive(s_mtx);
    JsonDocument ack;
    ack["type"] = "wifi_config_ack";
    ack["device"] = s_cfg.device;
    ack["id"] = s_deviceId;
    ack["wifiHash"] = hash;
    sendAll(ack);
}

void onWsEvent(AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[ARC] ARC-Studio verbunden: %s\n", client->remoteIP().toString().c_str());
        sendHello(client);
        JsonDocument init;
        fillStatus(init, "init_sync");
        init.remove("timecode");   // legacy: init_sync carries no TC
        String out;
        serializeJson(init, out);
        client->text(out);
        s_statusDirty = true;
        if (s_cfg.onClientChange) s_cfg.onClientChange(true);
        return;
    }
    if (type == WS_EVT_DISCONNECT) {
        Serial.println("[ARC] ARC-Studio getrennt");
        if (s_cfg.onClientChange) s_cfg.onClientChange(false);
        return;
    }
    if (type != WS_EVT_DATA) return;
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    const bool binary = info->opcode == WS_BINARY ||
                        (info->opcode == WS_CONTINUATION && info->message_opcode == WS_BINARY);
    if (binary) {
        if (s_cfg.onBinary) s_cfg.onBinary(client, data, len);
        return;
    }
    if (info->opcode != WS_TEXT || !info->final || info->index != 0 || info->len != len) return;

    JsonDocument doc;
    if (deserializeJson(doc, (const char*)data, len) != DeserializationError::Ok) return;
    const String type_ = doc["type"] | "";
    if (s_cfg.onMessage && s_cfg.onMessage(type_, doc, client)) return;
    if (project::handleMessage(type_, doc, client)) return;

    if (type_ == "command") handleCommand(doc);
    else if (type_ == "wifi_config") handleWifiConfig(doc);
    else if (type_ == "ping") {
        JsonDocument pong;
        pong["type"] = "pong";
        pong["device"] = s_cfg.device;
        pong["id"] = s_deviceId;
        pong["t0"] = doc["t0"];
        pong["rtcUs"] = clock::nowUs();
        sendTo(client, pong);
    } else if (type_ == "set_time") {
        s_setTimeUs = (int64_t)doc["epochMs"].as<double>() * 1000LL;   // coarse, local time
    }
}

// ── connection ───────────────────────────────────────────────────────────
bool connectTo(const String& ssid, const String& pass, uint32_t timeoutMs) {
    Serial.printf("[ARC] WLAN '%s' ...", ssid.c_str());
    WiFi.begin(ssid.c_str(), pass.c_str());
    const uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) delay(100);
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf(" ok (%s)\n", WiFi.localIP().toString().c_str());
        s_ssid = ssid;
        return true;
    }
    Serial.println(" fehlgeschlagen");
    WiFi.disconnect(true);
    delay(100);
    return false;
}

bool scanContains(int n, const char* ssid) {
    for (int i = 0; i < n; i++) if (WiFi.SSID(i) == ssid) return true;
    return false;
}

void startServices() {
    MDNS.end();
    if (MDNS.begin(s_cfg.hostname)) {
        MDNS.addService("http", "tcp", 80);
        MDNS.addServiceTxt("http", "tcp", "device_type", s_cfg.deviceType);
        MDNS.addServiceTxt("http", "tcp", "id", s_deviceId.c_str());
        MDNS.addServiceTxt("http", "tcp", "proto", "2");
    }
    if (!s_serverStarted) {
        s_ws.onEvent(onWsEvent);
        s_server.addHandler(&s_ws);
        s_server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
            r->send(200, "text/plain", String("ARC ") + s_cfg.device + " " + s_cfg.firmware + " – WebSocket: /ws");
        });
        s_server.begin();
        s_serverStarted = true;
    }
}

// NTP sets the RTC only if it is clearly wrong: after a jam the RTC is far
// more precise than NTP and must not be overwritten.
void syncNtp(bool force) {
    configTzTime(TZ_EUROPE_BERLIN, "pool.ntp.org", "time.nist.gov");
    struct tm ti;
    if (!getLocalTime(&ti, 5000)) { Serial.println("[ARC] NTP: keine Antwort"); return; }
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    const int64_t esp = esp_timer_get_time();
    struct tm lt;
    localtime_r(&tv.tv_sec, &lt);
    const DateTime local(lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec);
    const int64_t localNs = (int64_t)local.unixtime() * 1000000000LL + (int64_t)tv.tv_usec * 1000LL;
    const int64_t diffMs = (localNs - clock::wallNsAtEsp(esp)) / 1000000LL;
    const int64_t nowNs = localNs + (esp_timer_get_time() - esp) * 1000LL;
    if (clock::applyCoarseTime(nowNs, 2000000000LL, force || clock::rtcLostPower())) {
        Serial.printf("[ARC] NTP: RTC korrigiert (Abweichung %lld ms)\n", (long long)diffMs);
    } else {
        Serial.printf("[ARC] NTP: RTC ok (Abweichung %lld ms, unverändert)\n", (long long)diffMs);
    }
}

void connectTask(void*) {
    WiFi.persistent(false);
    WiFi.setHostname(s_cfg.hostname);
    for (;;) {
        if (s_role == Role::Station || s_role == Role::ArcNetClient) {
            if (WiFi.status() == WL_CONNECTED) { vTaskDelay(pdMS_TO_TICKS(2000)); continue; }
            Serial.println("[ARC] WLAN verloren");
            s_role = Role::Offline;
            if (s_cfg.onClientChange) s_cfg.onClientChange(false);
        }
        if (s_role == Role::ArcNetAp) {
            // Own network: once a minute look for known networks while nobody uses ours.
            vTaskDelay(pdMS_TO_TICKS(60000));
            if (WiFi.softAPgetStationNum() > 0 || s_ws.count() > 0) continue;
            WiFi.mode(WIFI_AP_STA);
            const int n = WiFi.scanNetworks();
            bool found = false;
            xSemaphoreTake(s_mtx, portMAX_DELAY);
            for (const auto& k : s_known) if (scanContains(n, k.ssid.c_str())) found = true;
            xSemaphoreGive(s_mtx);
            if (!found) continue;
            WiFi.softAPdisconnect(true);
            s_role = Role::Offline;
        }

        WiFi.mode(WIFI_STA);
        WiFi.setSleep(!s_cfg.lowLatency);
        const int n = WiFi.scanNetworks();

        xSemaphoreTake(s_mtx, portMAX_DELAY);
        std::vector<KnownNet> known = s_known;
        xSemaphoreGive(s_mtx);
        std::vector<std::pair<int, int>> cand;
        for (size_t k = 0; k < known.size(); k++)
            for (int i = 0; i < n; i++)
                if (WiFi.SSID(i) == known[k].ssid) cand.push_back({(int)k, WiFi.RSSI(i)});
        std::sort(cand.begin(), cand.end(), [&](const std::pair<int, int>& a, const std::pair<int, int>& b) {
            if (known[a.first].priority != known[b.first].priority) return known[a.first].priority > known[b.first].priority;
            return a.second > b.second;
        });
        bool ok = false;
        for (const auto& c : cand) {
            if (connectTo(known[c.first].ssid, known[c.first].pass, 8000)) { s_role = Role::Station; ok = true; break; }
        }
        if (!ok && scanContains(n, ARC_NET_SSID) && connectTo(ARC_NET_SSID, s_arcPass, 8000)) {
            s_role = Role::ArcNetClient;
            ok = true;
        }
        if (!ok) {
            // Random wait so that two devices do not both open an AP.
            vTaskDelay(pdMS_TO_TICKS(esp_random() % 3000));
            const int n2 = WiFi.scanNetworks();
            if (scanContains(n2, ARC_NET_SSID) && connectTo(ARC_NET_SSID, s_arcPass, 8000)) {
                s_role = Role::ArcNetClient;
                ok = true;
            }
        }
        if (!ok) {
            WiFi.mode(WIFI_AP);
            WiFi.softAP(ARC_NET_SSID, s_arcPass.c_str());
            s_ssid = ARC_NET_SSID;
            s_role = Role::ArcNetAp;
            Serial.printf("[ARC] Eigenes Netz '%s' (%s)\n", ARC_NET_SSID, WiFi.softAPIP().toString().c_str());
        }
        startServices();
        if (s_role == Role::Station) syncNtp(false);
        s_statusDirty = true;
        if (s_cfg.onClientChange) s_cfg.onClientChange(studioConnected());
    }
}

}  // namespace

void begin(const Config& cfg) {
    s_cfg = cfg;
    s_mtx = xSemaphoreCreateMutex();
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char id[13];
    snprintf(id, sizeof(id), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    s_deviceId = id;
    loadKnown();
    xTaskCreatePinnedToCore(connectTask, "ARCNET", 8192, nullptr, 1, nullptr, 0);
}

void loop() {
    static uint32_t lastHb = 0;
    if (s_serverStarted) s_ws.cleanupClients();
    if (s_statusDirty || (millis() - lastHb > 1000 && s_ws.count() > 0)) {
        s_statusDirty = false;
        lastHb = millis();
        broadcastStatus();
    }
    if (s_ntpRequested) {
        s_ntpRequested = false;
        if (s_role == Role::Station) syncNtp(true);
    }
    if (s_setTimeUs) {
        const int64_t t = s_setTimeUs;
        s_setTimeUs = 0;
        clock::applyCoarseTime(t * 1000LL, 2000000000LL, false);
    }
}

Role role() { return s_role; }
String ssid() { return s_ssid; }
bool studioConnected() { return s_serverStarted && s_ws.count() > 0; }
const String& deviceId() { return s_deviceId; }

String ipString() {
    if (s_role == Role::ArcNetAp) return WiFi.softAPIP().toString();
    if (WiFi.status() == WL_CONNECTED) return WiFi.localIP().toString();
    return "-";
}

String statusLine() {
    String line;
    switch (s_role) {
        case Role::Station:      line = "WLAN " + s_ssid; break;
        case Role::ArcNetClient: line = "ARC-Net"; break;
        case Role::ArcNetAp:     line = "ARC-Net (eigenes Netz)"; break;
        default:                 line = "kein Netz"; break;
    }
    line += " " + ipString();
    line += studioConnected() ? "  |  Studio verbunden" : "  |  Studio getrennt";
    return line;
}

void broadcastStatus() {
    JsonDocument doc;
    fillStatus(doc, "status");
    sendAll(doc);
}

void send(JsonDocument& doc) { sendAll(doc); }

void sendTo(AsyncWebSocketClient* client, JsonDocument& doc) {
    String out;
    serializeJson(doc, out);
    client->text(out);
}

void requestNtpSync() { s_ntpRequested = true; }

void addNetwork(const String& ssid, const String& pass, int priority) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    for (auto it = s_known.begin(); it != s_known.end(); ++it) {
        if (it->ssid == ssid) { s_known.erase(it); break; }
    }
    s_known.push_back({ssid, pass, priority});
    saveKnown();
    xSemaphoreGive(s_mtx);
}

void clearNetworks() {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_known.clear();
    saveKnown();
    xSemaphoreGive(s_mtx);
}

void printNetworks(Stream& out) {
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    out.printf("Bekannte Netze (%u), Hash %s:\n", (unsigned)s_known.size(), knownHash().c_str());
    for (const auto& n : s_known) out.printf("  %-24s Prio %d\n", n.ssid.c_str(), n.priority);
    out.printf("ARC-Net-Passwort: %s\n", s_arcPass.length() >= 8 ? "gesetzt" : "ZU KURZ (min. 8 Zeichen)");
    xSemaphoreGive(s_mtx);
}

}  // namespace net
}  // namespace arc
