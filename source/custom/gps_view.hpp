// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "board.h"
#include "application.hpp"
namespace rt950 {
void gps_coordinate(const char *nmea,unsigned degree_digits,bool decimal,char *out,bool dms=false);
bool gps_altitude_fresh(const nav_status_t &,uint32_t now);
int32_t gps_altitude_display(int32_t decimetres,bool feet);
unsigned gps_speed_tenths(uint32_t centi_knots,SpeedUnit unit);
const char *gps_speed_suffix(SpeedUnit unit);
bool gps_local_time(const nav_status_t &,int,uint32_t,char *,char *);
}
