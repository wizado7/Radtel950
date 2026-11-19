// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
namespace rt950 {
constexpr uint8_t encoder_interval(bool screen_awake) {return screen_awake?2u:25u;}
constexpr uint8_t keypad_interval(bool screen_awake) {return screen_awake?10u:25u;}
}
