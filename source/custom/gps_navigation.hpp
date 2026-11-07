// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
namespace rt950 {
enum class DistanceUnit : uint8_t { Kilometres, Miles, NauticalMiles };
struct GeoPoint {int32_t latitude=0,longitude=0;bool valid=false;}; // microdegrees
struct GeoRange {uint32_t metres=0;uint16_t bearing_deci=0;bool valid=false,bearing_valid=false;};
bool geo_valid(const GeoPoint &point);
bool gps_current_point(uint32_t now,GeoPoint &point);
GeoRange geo_range(const GeoPoint &from,const GeoPoint &to);
uint32_t distance_thousandths(uint32_t metres,DistanceUnit unit);
}
