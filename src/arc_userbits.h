#pragma once
// LTC user bits (pure C++). All values are in "display order": the first user
// bit group (user1) is the most significant nibble, as shown by recorders and
// decoded by ARC-Server (videoProcessor.js).
#include <stdint.h>

namespace arc {

// Shot identification:
//   [31:28] camera (A=0xA .. F=0xF) · [27:21] roll · [20:14] scene · [13:7] shot · [6:0] take
uint32_t shotUserBits(char cam, int roll, int scene, int shot, int take);

// Date + time zone (SMPTE 309M): DD MM YY + zone code (+0100 / +0200).
uint32_t dateUserBits(int day, int month, int year, bool summerTime);

// Converts between display order and the order expected by libltc's
// ltc_encoder_set_user_bits() (user1 = lowest nibble).
uint32_t reverseNibbles(uint32_t v);

// EU summer time (last Sunday of March 02:00 – last Sunday of October 03:00), local time.
bool isEuSummerTime(int year, int month, int day, int hour);

}  // namespace arc
