#pragma once
// Event log in LittleFS (/events.jsonl, one JSON object per line, rotated at
// 256 KB). Survives power loss; ARC-Studio pulls it after offline work.
#include <Arduino.h>
#include <ArduinoJson.h>

namespace arc {
namespace events {

bool begin();                        // mounts LittleFS (formats on first use)
uint32_t nextId();                   // monotonically increasing per device (persisted)
void append(JsonDocument& event);
void print(Stream& out);
void clear();
size_t size();

}  // namespace events
}  // namespace arc
