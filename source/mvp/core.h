#ifndef RT950_MVP_CORE_H
#define RT950_MVP_CORE_H
#include <stdbool.h>
enum { RT_GPS_OFF, RT_GPS_AUTO, RT_GPS_ON };
#include <stddef.h>
#include <stdint.h>
#define RT_KISS_MAX 1024u
#define RT_WATCH_SINGLE 0u
#define RT_WATCH_ABC 1u
#define RT_WATCH_MASK 2u
typedef struct {
    uint32_t rx_hz, tx_hz;
    uint16_t memory_channel;
    bool channel_mode;
} rt_vfo;
typedef struct {
    rt_vfo vfo[3];
    uint8_t active, watch_mode, watch_mask;
    uint16_t dwell_ms, hold_ms;
    uint8_t gps_on; /* RT_GPS_OFF, RT_GPS_AUTO, RT_GPS_ON */
    bool aprs_rx, beacon_on, ble_on, ble_export;
    uint16_t aprs_scan_seconds; /* 0 disables background scan */
    uint32_t aprs_rx_hz, aprs_hz, beacon_seconds; /* aprs_hz is the TX frequency */
} rt_config;
typedef struct { uint8_t tuned, last_active; uint32_t changed, hold_until; bool holding, initialized; } rt_watch;
static inline bool rt_low_fm_rx_frequency(uint32_t hz) {
    return hz>=18000000u && hz<30000000u && hz%10u==0;
}
void rt_defaults(rt_config *cfg);
bool rt_config_valid(const rt_config *cfg);
uint8_t rt_watch_tick(rt_watch *s, const rt_config *cfg, uint32_t now,
                      bool squelch_open, bool voice_ptt);
bool rt_beacon_due(const rt_config *cfg, bool valid_fix, bool voice_busy,
                   uint32_t now_seconds, uint32_t last_tx_seconds);
/* Encoder/decoder are stream based: a BLE notification is NOT a frame boundary. */
typedef void (*rt_frame_cb)(void *ctx, uint8_t type, const uint8_t *data, size_t n);
typedef struct {
    uint8_t buf[RT_KISS_MAX+1u]; size_t n;
    bool started, escaped, invalid;
    uint32_t rejected;
} rt_kiss;
void rt_kiss_feed(rt_kiss *s, uint8_t byte, rt_frame_cb cb, void *ctx);
size_t rt_kiss_encode(uint8_t type, const uint8_t *data, size_t n,
                      uint8_t *out, size_t cap);
#endif
