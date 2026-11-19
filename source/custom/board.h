// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
#include "../diagnostic/nmea_status.h"
void board_init(void);
void board_display_init(uint32_t now);
void board_display_brightness(uint16_t permille);
bool board_display_poll(bool requested,uint32_t now);
void board_display_shutdown(void);
void board_radio_led(uint8_t rx,uint8_t tx);
enum { BOARD_BATTERY_FAST_MS=20, BOARD_BATTERY_CONVERSION_TIMEOUT_MS=20 };
void board_battery_init(uint32_t now);
bool board_battery_idle(void);
uint32_t board_battery_deadline(uint32_t now);
bool board_idle_clock(bool low);
#include "custom_clock.h"
void board_idle_wait(uint8_t timeout_ms,uint8_t ptt);
void board_display_retime(void);
void board_display_keypad(bool enabled);
void board_battery_fast(uint8_t enabled);
void board_battery_poll(uint32_t now);
uint16_t board_battery_mv(void);
uint8_t board_battery_sample_fresh(uint32_t now);
uint16_t board_flash_kib(void);
uint8_t board_storage_probe(void); /* bit 0/1: nonblank slot, 255: unsupported */
uint8_t board_battery_percent_from_mv(uint16_t mv);
uint8_t board_battery_raw(void);
uint16_t board_battery_raw12(void);
uint32_t board_battery_sample_age(uint32_t now);
uint32_t board_battery_errors(void);
uint8_t board_scan_key(void);
void board_keypad_sleep(bool enabled);
bool board_keypad_pending(void); // OEM-compatible raw codes 0..21, 255 = none
uint8_t board_power_switch_off(void);
void board_power_cut(void);
uint8_t board_ptt_mask(void); /* bit 0 PE3, bit 1 PE2, bit 2 PE5, pressed */
void board_gps_enable(uint8_t on);
void board_gps_poll(uint32_t now);
uint8_t board_gps_powered(void);
void board_gps_sleeping_set(bool sleeping);
bool board_gps_sleeping(void);
bool board_gps_transmit(bool inhibit,uint32_t now);
void board_gps_send(const uint8_t *data,unsigned size,uint32_t now);
uint32_t board_gps_bytes(void);
uint32_t board_gps_errors(void);
uint32_t board_gps_dropped(void);
extern volatile uint32_t custom_heartbeat;
extern volatile uint8_t custom_watchdog_armed;
#ifdef __cplusplus
}
#endif
