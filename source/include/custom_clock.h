// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdbool.h>
#include <stdint.h>

enum { ClockCrystalMHz=8, ClockLowMult=3, ClockNormalMult=15 };
extern bool custom_clock_low;
static inline unsigned clock_mhz(unsigned mult) {return ClockCrystalMHz*mult;}
static inline unsigned clock_apb(unsigned mult) {return clock_mhz(mult)*1000000u/(mult==ClockNormalMult?2u:1u);}
static inline unsigned clock_brr(unsigned mult,unsigned baud) {return (clock_apb(mult)+baud/2u)/baud;}
static inline unsigned clock_mult(void) {return custom_clock_low?ClockLowMult:ClockNormalMult;}
static inline bool clock_due(uint32_t now,uint32_t at) {return (int32_t)(now-at)>=0;}
