#pragma once
// ARC-Core – shared firmware building blocks of ARC-Play and ARC-Slate.
//
//   arc_timecode   rates, drop frame, frame <-> real time (pure)
//   arc_timeline   TC as a function of wall-clock time (pure)
//   arc_userbits   shot / date user bits (pure)
//   arc_clock      RTC-disciplined wall clock (DS3231 + SQW)
//   arc_ltc_gen    phase-locked LTC generator + output clock model
//   arc_ltc_in     LTC decoder with µs edge timestamps
//   arc_engine     modes, transport, Jam Sync, loopback latency calibration
//   arc_net        WiFi / ARC-Net, mDNS, WebSocket protocol v2
//   arc_events     event log (LittleFS)
//   arc_project    offline project (shot list in LittleFS) + log pull
#include "arc_clock.h"
#include "arc_engine.h"
#include "arc_events.h"
#include "arc_ltc_gen.h"
#include "arc_ltc_in.h"
#include "arc_net.h"
#include "arc_project.h"
#include "arc_timecode.h"
#include "arc_timeline.h"
#include "arc_userbits.h"
