#pragma once
// Offline project: the shot list sent by ARC-Studio ("project_put", in
// chunks) is stored in LittleFS (/project.json) so the device can step
// through the shots without ARC-Studio. Takes and claps recorded meanwhile
// stay in the event log and are pulled by ARC-Studio later ("log_pull").
#include <Arduino.h>
#include <ArduinoJson.h>

class AsyncWebSocketClient;

namespace arc {
namespace project {

bool begin();                         // loads /project.json (LittleFS must be mounted)
String code();                        // project code ("" = none)
int shotCount();
// Copies shot `index` (scene, shot, framerate, starttime, endtime, speed, …) into `out`
// together with the project meta data (code, production, director, camera, songtitle).
bool shotMessage(int index, JsonDocument& out);
int indexOf(const String& scene, const String& shot);   // -1 = not found

// Handles project_put / log_pull / log_ack. Returns true if the message was consumed.
bool handleMessage(const String& type, JsonDocument& doc, AsyncWebSocketClient* client);

}  // namespace project
}  // namespace arc
