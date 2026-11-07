// SPDX-License-Identifier: GPL-3.0-or-later
#include "gps_view.hpp"
namespace rt950 {
bool gps_altitude_fresh(const nav_status_t &nav,uint32_t now) {
    return nav.altitude_valid && nav_fix_fresh(now) && uint32_t(now-nav.last_gga_ms)<3000u;
}
int32_t gps_altitude_display(int32_t dm,bool feet) {
    // One foot is exactly 0.3048 m. The parser bounds magnitude to 99999 dm.
    const unsigned magnitude=unsigned(dm<0?-dm:dm);
    const int32_t rounded=int32_t(feet?(magnitude*1000u+1524u)/3048u:(magnitude+5u)/10u);
    return dm<0?-rounded:rounded;
}
unsigned gps_speed_tenths(uint32_t centi_knots,SpeedUnit unit) {
    // RMC parser limits this input to 100000 hundredths of a knot.
    switch(unit) {
    case SpeedUnit::Kmh:return (centi_knots*1852u+5000u)/10000u;
    case SpeedUnit::Mph:return (centi_knots*11508u+50000u)/100000u;
    case SpeedUnit::Knots:return (centi_knots+5u)/10u;
    }
    return 0;
}
const char *gps_speed_suffix(SpeedUnit unit) {
    switch(unit) {
    case SpeedUnit::Kmh:return "km/h";
    case SpeedUnit::Mph:return "mph";
    case SpeedUnit::Knots:return "kn";
    }
    return "";
}
void gps_coordinate(const char *nmea,unsigned degree_digits,bool decimal,char *out,bool dms) {
    if(!decimal && !dms) {
        while((*out++=*nmea++)) {}
        return;
    }
    // The NMEA parser already validated this coordinate.
    for(unsigned i=0;i<degree_digits;++i)*out++=*nmea++;
    const unsigned minutes=unsigned(nmea[0]-'0')*10u+unsigned(nmea[1]-'0');
    nmea+=2;
    unsigned micro_minutes=minutes*1000000u;
    if(*nmea=='.') {
        ++nmea;
        for(unsigned scale=100000;scale && *nmea;scale/=10)micro_minutes+=unsigned(*nmea++-'0')*scale;
    }
    if(dms) {
        // Integer hundredths of a second. Truncate so 59.999999 minutes
        // cannot display 60 seconds or escape the validated degree/pole.
        const unsigned seconds=(micro_minutes%1000000u)*6u/1000u;
        *out++='d';*out++=char('0'+minutes/10u);*out++=char('0'+minutes%10u);
        *out++=char(39);*out++=char('0'+seconds/1000u);*out++=char('0'+seconds/100u%10u);
        *out++='.';*out++=char('0'+seconds/10u%10u);*out++=char('0'+seconds%10u);
        *out++=char(34);*out=0;return;
    }
    // Truncation keeps 59.999999 minutes within the original degree.
    unsigned fraction=micro_minutes/60u;
    *out++='.';
    for(unsigned scale=100000;scale;scale/=10)*out++=char('0'+fraction/scale%10u);
    *out=0;
}

static void two(char *p,unsigned n){p[0]=char('0'+n/10);p[1]=char('0'+n%10);}
static unsigned pair(const char *p){return unsigned(p[0]-'0')*10u+unsigned(p[1]-'0');}
static unsigned days(unsigned m,unsigned y){static const uint8_t d[12]={31,28,31,30,31,30,31,31,30,31,30,31};return d[m-1]+(m==2 && y%4==0 && (y%100!=0||y%400==0)?1u:0u);}
bool gps_local_time(const nav_status_t &n,int zone,uint32_t now,char *time,char *date) {
 if(!n.time_valid || uint32_t(now-n.last_time_ms)>=3000u || zone< -720 || zone>840)return false;
 unsigned day=pair(n.date),month=pair(n.date+2),year=2000+pair(n.date+4);
 if(month<1||month>12||day<1||day>days(month,year))return false;
 int minutes=int(pair(n.utc)*60u+pair(n.utc+2))+zone;
 if(minutes<0){minutes+=1440;if(--day==0){if(--month==0){month=12;--year;}day=days(month,year);}}
 if(minutes>=1440){minutes-=1440;if(++day>days(month,year)){day=1;if(++month>12){month=1;++year;}}}
 two(time,unsigned(minutes)/60u);time[2]=':';two(time+3,unsigned(minutes)%60u);time[5]=':';two(time+6,pair(n.utc+4));time[8]=0;
 two(date,year/100);two(date+2,year%100);date[4]='-';two(date+5,month);date[7]='-';two(date+8,day);date[10]=0;return true;
}
}
