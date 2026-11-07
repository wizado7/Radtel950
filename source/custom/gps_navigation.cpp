// SPDX-License-Identifier: GPL-3.0-or-later
#include "gps_navigation.hpp"
extern "C" {
#include "../diagnostic/nmea_status.h"
}
namespace rt950 {
namespace {
// Angles: unsigned full turn = 2^32. Vectors: Q30. All CORDIC loops
// have 31 iterations; no heap, libm or unbounded peripheral waits.
constexpr uint32_t angles[]={536870912u,316933406u,167458907u,85004756u,42667331u,21354465u,10679838u,5340245u,2670163u,1335087u,667544u,333772u,166886u,83443u,41722u,20861u,10430u,5215u,2608u,1304u,652u,326u,163u,81u,41u,20u,10u,5u,3u,1u,1u};
constexpr int64_t One=int64_t(1)<<30;
struct Pair {int32_t x,y;};
uint32_t turn(int32_t microdegrees) {
    const uint32_t magnitude=uint32_t(microdegrees<0?-microdegrees:microdegrees);
    const uint32_t result=uint32_t((uint64_t(magnitude)*1601279868u+(uint64_t(1)<<26))>>27);
    return microdegrees<0?0u-result:result;
}
Pair circle(uint32_t angle) {
    int64_t z=int32_t(angle),sign=1,x=652032874,y=0;
    if(z>One){z-=int64_t(1)<<31;sign=-1;}
    if(z< -One){z+=int64_t(1)<<31;sign=-1;}
    for(unsigned i=0;i<31;++i) {
        const int64_t old=x;
        if(z>=0){x-=y>>i;y+=old>>i;z-=angles[i];}
        else{x+=y>>i;y-=old>>i;z+=angles[i];}
    }
    x*=sign;y*=sign;
    if(x>One)x=One;
    if(x< -One)x=-One;
    if(y>One)y=One;
    if(y< -One)y=-One;
    return {int32_t(x),int32_t(y)};
}
int32_t product(int32_t a,int32_t b){return int32_t((int64_t(a)*b)>>30);}
uint32_t angle_of(int64_t x,int64_t y) {
    int64_t z=0;
    if(x<0){x=-x;y=-y;z=int64_t(1)<<31;}
    for(unsigned i=0;i<31;++i) {
        const int64_t old=x;
        if(y>0){x+=y>>i;y-=old>>i;z+=angles[i];}
        else if(y<0){x-=y>>i;y+=old>>i;z-=angles[i];}
    }
    return uint32_t(z);
}
uint32_t root(uint64_t value) {
    uint64_t bit=uint64_t(1)<<62,result=0;
    while(bit>value)bit>>=2;
    for(;bit;bit>>=2) {
        if(value>=result+bit){value-=result+bit;result=(result>>1)+bit;}
        else result>>=1;
    }
    return uint32_t(result);
}
bool coordinate(const char *text,unsigned degrees,char hemisphere,int32_t &value) {
    unsigned whole=0;
    for(unsigned i=0;i<degrees;++i){if(text[i]<'0'||text[i]>'9')return false;whole=whole*10u+unsigned(text[i]-'0');}
    text+=degrees;
    if(text[0]<'0'||text[0]>'5'||text[1]<'0'||text[1]>'9')return false;
    uint32_t minutes=unsigned(text[0]-'0')*10000000u+unsigned(text[1]-'0')*1000000u;
    unsigned pos=2,scale=100000;
    if(text[pos]=='.') {
        ++pos;unsigned count=0;
        for(;pos<16 && text[pos];++pos) {
            if(text[pos]<'0'||text[pos]>'9')return false;
            if(scale){minutes+=unsigned(text[pos]-'0')*scale;scale/=10;}
            ++count;
        }
        if(!count || pos==16)return false;
    }
    if(text[pos])return false;
    const unsigned maximum=degrees==2?90u:180u;
    if(whole>maximum || (whole==maximum && minutes))return false;
    const bool negative=hemisphere==(degrees==2?'S':'W');
    if(!negative && hemisphere!=(degrees==2?'N':'E'))return false;
    value=int32_t(whole*1000000u+minutes/60u);
    if(negative)value=-value;
    return true;
}
}
bool geo_valid(const GeoPoint &point) {
    return point.valid?(point.latitude>=-90000000 && point.latitude<=90000000 &&
        point.longitude>=-180000000 && point.longitude<=180000000):(!point.latitude && !point.longitude);
}
bool gps_current_point(uint32_t now,GeoPoint &point) {
    point={};if(!nav_fix_fresh(now))return false;
    const auto *n=nav_get();GeoPoint result{};
    if(!coordinate(n->lat,2,n->ns[0],result.latitude) ||
       !coordinate(n->lon,3,n->ew[0],result.longitude))return false;
    result.valid=true;point=result;return true;
}
GeoRange geo_range(const GeoPoint &from,const GeoPoint &to) {
    GeoRange result{};
    if(!from.valid || !to.valid || !geo_valid(from) || !geo_valid(to))return result;
    const Pair a=circle(turn(from.latitude)),b=circle(turn(to.latitude));
    const Pair al=circle(turn(from.longitude)),bl=circle(turn(to.longitude));
    const int32_t u[]={product(a.x,al.x),product(a.x,al.y),a.y};
    const int32_t v[]={product(b.x,bl.x),product(b.x,bl.y),b.y};
    const int64_t dot=int64_t(product(u[0],v[0]))+product(u[1],v[1])+product(u[2],v[2]);
    const int64_t x=int64_t(product(u[1],v[2]))-product(u[2],v[1]);
    const int64_t y=int64_t(product(u[2],v[0]))-product(u[0],v[2]);
    const int64_t z=int64_t(product(u[0],v[1]))-product(u[1],v[0]);
    const uint32_t cross=root(uint64_t(x*x)+uint64_t(y*y)+uint64_t(z*z));
    uint32_t angle=angle_of(dot,cross);
    if(angle>0x80000000u)angle=0x80000000u;
    // Chosen spherical model: R=6,371,000 m, circumference rounded to metre.
    result.metres=uint32_t((uint64_t(angle)*40030174u+0x80000000u)>>32);
    const Pair delta=circle(turn(to.longitude)-turn(from.longitude));
    const int64_t east=product(delta.y,b.x);
    const int64_t north=int64_t(product(a.x,b.y))-product(product(a.y,b.x),delta.x);
    result.valid=true;
    result.bearing_valid=result.metres>=3 && cross>128 && from.latitude>-90000000 && from.latitude<90000000;
    if(result.bearing_valid) {
        unsigned bearing=unsigned((uint64_t(angle_of(north,east))*3600u+0x80000000u)>>32);
        result.bearing_deci=uint16_t(bearing==3600?0:bearing);
    }
    return result;
}
uint32_t distance_thousandths(uint32_t metres,DistanceUnit unit) {
    if(unit==DistanceUnit::Kilometres)return metres;
    const uint32_t factor=unit==DistanceUnit::Miles?667192237u:579774203u;
    return uint32_t((uint64_t(metres)*factor+(uint64_t(1)<<29))>>30);
}
}
