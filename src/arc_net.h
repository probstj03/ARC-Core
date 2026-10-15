#pragma once
// Networking shared by all ARC devices.
//
// Connection order: known networks (NVS, managed by ARC-Server) -> existing
// "ARC-Net" -> random wait -> open "ARC-Net" as access point ourselves.
// Services: mDNS (<hostname>.local, _http._tcp, TXT device_type/id/proto),
// WebSocket /ws for ARC-Studio (protocol v2, legacy fields kept).
//
// Generic messages handled here: ping, wifi_config, set_time and the commands
// start/play, stop/pause (with optional "at" in wall-clock µs), reset,
// set_tc_mode, set_gen_mode, jam_confirm_unplug, sync_rtc. Everything else
// (and every command first) goes to the device handler.
#include <Arduino.h>
#include <ArduinoJson.h>

class AsyncWebSocketClient;

namespace arc {
namespace net {

struct DefaultNetwork { const char* ssid; const char* pass; int priority; };

struct Config {
    const char* hostname = "arc-device";      // mDNS name
    const char* deviceType = "unknown";       // mDNS TXT device_type ("slate" / "playback")
    const char* device = "unknown";           // protocol "device" field ("slate" / "play")
    const char* firmware = "dev";
    const char* const* caps = nullptr;
    int capCount = 0;
    const DefaultNetwork* defaults = nullptr;
    int defaultCount = 0;
    const char* arcNetPassword = "arc-net-2026";
    // false for devices without LTC engine (e.g. ARC-Clock): status carries only the wall
    // clock, engine commands are ignored, nothing touches arc::engine.
    bool ltcEngine = true;
    // false: WiFi modem sleep is allowed (saves power, adds latency). Timing devices keep true.
    bool lowLatency = true;
    // Device hooks (all optional). onMessage returns true if it handled the message.
    bool (*onMessage)(const String& type, JsonDocument& doc, AsyncWebSocketClient* client) = nullptr;
    void (*fillStatus)(JsonDocument& doc) = nullptr;
    void (*onClientChange)(bool connected) = nullptr;
    void (*onBinary)(AsyncWebSocketClient* client, const uint8_t* data, size_t len) = nullptr;
};

enum class Role : uint8_t { Offline, Station, ArcNetClient, ArcNetAp };

void begin(const Config& cfg);
void loop();

Role role();
String ssid();
String ipString();
bool studioConnected();
const String& deviceId();
String statusLine();                  // "WiFi x 192.168.. | Studio connected"

void broadcastStatus();
void send(JsonDocument& doc);
void sendTo(AsyncWebSocketClient* client, JsonDocument& doc);
void requestNtpSync();

void addNetwork(const String& ssid, const String& pass, int priority);
void clearNetworks();
void printNetworks(Stream& out);

}  // namespace net
}  // namespace arc
