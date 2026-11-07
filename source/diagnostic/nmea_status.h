#ifndef NMEA_STATUS_H
#define NMEA_STATUS_H
#include <stdint.h>
typedef struct {
    uint32_t good, bad, last_gga_ms, last_fix_ms;
    uint8_t quality, used, seen_gp, seen_bd, max_gp, max_bd, ever_fix;
    char utc[16], lat[20], lon[20], ns[2], ew[2];
    uint32_t last_rmc_ms,last_time_ms,last_byte_ms;
    uint32_t speed_centi_knots;
    int32_t altitude_decimetres;
    uint8_t altitude_valid;
    uint16_t course_deci;
    uint8_t position_valid,time_valid,course_valid,rmc_valid,speed_valid;
    char date[7];
    uint32_t rmc_count;
} nav_status_t;
uint8_t nav_fix_fresh(uint32_t now);
void nav_feed(uint8_t byte, uint32_t now);
void nav_reset(void);
const nav_status_t *nav_get(void);
#endif
