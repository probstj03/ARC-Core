#include "arc_userbits.h"

namespace arc {

uint32_t reverseNibbles(uint32_t v) {
    return ((v & 0xF) << 28) | (((v >> 4) & 0xF) << 24) | (((v >> 8) & 0xF) << 20) |
           (((v >> 12) & 0xF) << 16) | (((v >> 16) & 0xF) << 12) | (((v >> 20) & 0xF) << 8) |
           (((v >> 24) & 0xF) << 4) | ((v >> 28) & 0xF);
}

uint32_t shotUserBits(char cam, int roll, int scene, int shot, int take) {
    if (cam >= 'a' && cam <= 'z') cam = (char)(cam - 'a' + 'A');
    if (cam < 'A' || cam > 'F') cam = 'A';
    const uint32_t camId = (uint32_t)(cam - 'A' + 10) & 0xF;
    return (camId << 28) | (((uint32_t)roll & 0x7F) << 21) | (((uint32_t)scene & 0x7F) << 14) |
           (((uint32_t)shot & 0x7F) << 7) | ((uint32_t)take & 0x7F);
}

uint32_t dateUserBits(int day, int month, int year, bool summerTime) {
    const uint32_t yy = (uint32_t)(year % 100);
    const uint8_t tz = summerTime ? 0x24 : 0x25;   // libltc codes for +0200 / +0100
    const uint32_t raw = (uint32_t)(day % 10) | ((uint32_t)(day / 10) << 4) | ((uint32_t)(month % 10) << 8) |
                         ((uint32_t)(month / 10) << 12) | ((yy % 10) << 16) | ((yy / 10) << 20) |
                         ((uint32_t)(tz & 0x0F) << 24) | ((uint32_t)((tz >> 4) & 0x0F) << 28);
    return reverseNibbles(raw);
}

namespace {
// 0 = Sunday (Sakamoto)
int dayOfWeek(int y, int m, int d) {
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) y -= 1;
    return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}
int lastSunday(int y, int m) { return 31 - dayOfWeek(y, m, 31); }
}  // namespace

bool isEuSummerTime(int year, int month, int day, int hour) {
    if (month < 3 || month > 10) return false;
    if (month > 3 && month < 10) return true;
    const int ls = lastSunday(year, month);
    if (month == 3) return day > ls || (day == ls && hour >= 2);
    return day < ls || (day == ls && hour < 3);
}

}  // namespace arc
