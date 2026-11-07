#include "nmea_status.h"
static nav_status_t nav;
static char line[192];
static uint32_t count;
static uint8_t receiving;
static int hex(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
static void field(uint32_t n, char *out, uint32_t capacity)
{
    const char *p = line;
    while (n && *p && *p != '*') if (*p++ == ',') --n;
    uint32_t i = 0;
    while (!n && *p && *p != ',' && *p != '*' && i + 1 < capacity)
        out[i++] = *p++;
    out[i] = 0;
}
static uint8_t integer(const char *s)
{
    uint32_t n = 0;
    while (*s >= '0' && *s <= '9') {
        n = n * 10 + (uint32_t)(*s++ - '0');
        if (n > 255) return 255;
    }
    return (uint8_t)n;
}
static uint8_t coordinate(const char *s, uint32_t degree_digits, uint32_t limit)
{
    uint32_t degrees = 0, i = 0;
    for (; i < degree_digits; ++i) {
        if (s[i] < '0' || s[i] > '9') return 0;
        degrees = degrees * 10 + (uint32_t)(s[i] - '0');
    }
    if (s[i] < '0' || s[i] > '5' || s[i+1] < '0' || s[i+1] > '9') return 0;
    uint32_t minutes = (uint32_t)(s[i]-'0') * 10 + (uint32_t)(s[i+1]-'0');
    i += 2;
    if (degrees > limit || s[i++] != '.') return 0;
    if (!s[i]) return 0;
    uint8_t fractional_nonzero = 0;
    unsigned fraction_digits=0;
    for (; s[i]; ++i) {
        if (s[i] < '0' || s[i] > '9' || ++fraction_digits>8) return 0;
        if (s[i] != '0') fractional_nonzero = 1;
    }
    return degrees < limit || (!minutes && !fractional_nonzero);
}
static uint8_t utc_valid(const char *s) {
    for(unsigned i=0;i<6;++i)if(s[i]<'0'||s[i]>'9')return 0;
    unsigned h=(unsigned)(s[0]-'0')*10u+(unsigned)(s[1]-'0');
    unsigned m=(unsigned)(s[2]-'0')*10u+(unsigned)(s[3]-'0');
    unsigned sec=(unsigned)(s[4]-'0')*10u+(unsigned)(s[5]-'0');
    if(h>23||m>59||sec>59)return 0;
    if(!s[6])return 1;
    if(s[6]!='.' || !s[7])return 0;
    for(unsigned i=7;s[i];++i)if(s[i]<'0'||s[i]>'9')return 0;
    return 1;
}
static uint8_t date_valid(const char *s) {
    for(unsigned i=0;i<6;++i)if(s[i]<'0'||s[i]>'9')return 0;
    if(s[6])return 0;
    unsigned d=(unsigned)(s[0]-'0')*10u+(unsigned)(s[1]-'0');
    unsigned m=(unsigned)(s[2]-'0')*10u+(unsigned)(s[3]-'0');
    unsigned y=(unsigned)(s[4]-'0')*10u+(unsigned)(s[5]-'0');
    static const uint8_t days[12]={31,28,31,30,31,30,31,31,30,31,30,31};
    if(m<1||m>12)return 0;
    unsigned max=days[m-1]+(m==2 && y%4==0?1u:0u);
    return d>=1 && d<=max;
}
static uint8_t decimal(const char *s,unsigned digits,uint32_t max,uint32_t *out) {
    if(*s<'0'||*s>'9')return 0;
    uint32_t n=0,frac=0,scale=1;unsigned len=0;
    while(*s>='0'&&*s<='9'){if(++len>4)return 0;n=n*10u+(unsigned)(*s++-'0');}
    if(*s=='.'){++s;if(!*s)return 0;unsigned f=0;while(*s>='0'&&*s<='9'){if(f++<digits){frac=frac*10u+(unsigned)(*s-'0');scale*=10u;}++s;}}
    if(*s)return 0;
    uint32_t mult=1;for(unsigned i=0;i<digits;++i)mult*=10u;
    n=n*mult+frac*mult/scale;if(n>max)return 0;*out=n;return 1;
}
static void parse(uint32_t now)
{
    uint32_t star = 1;
    uint8_t sum = 0;
    while (star < count && line[star] != '*') sum ^= (uint8_t)line[star++];
    if (star + 3 != count || hex(line[star+1]) < 0 || hex(line[star+2]) < 0 ||
        sum != (uint8_t)((hex(line[star+1]) << 4) | hex(line[star+2]))) {
        ++nav.bad; return;
    }
    ++nav.good;
    if (count < 9 || line[6] != ',') return;
    char value[20];
    if (line[3] == 'G' && line[4] == 'G' && line[5] == 'A') {
        nav.position_valid=0;
        nav.altitude_valid=0;
        field(6, value, sizeof value); nav.quality = integer(value);
        field(7, value, sizeof value); nav.used = integer(value);
        nav.last_gga_ms = now;
        /* Retain the last VALID location and UTC after reception is lost. */
        if (nav.quality && nav.quality <= 5 && nav.used) {
            char lat[20], lon[20], ns[2], ew[2];
            field(2, lat, sizeof lat); field(4, lon, sizeof lon);
            field(3, ns, sizeof ns); field(5, ew, sizeof ew);
            if (coordinate(lat, 2, 90) && coordinate(lon, 3, 180) &&
                (ns[0]=='N' || ns[0]=='S') &&
                (ew[0]=='E' || ew[0]=='W')) {

                field(2, nav.lat, sizeof nav.lat); field(4, nav.lon, sizeof nav.lon);
                field(3, nav.ns, sizeof nav.ns); field(5, nav.ew, sizeof nav.ew);
                nav.position_valid=1;nav.ever_fix = 1; nav.last_fix_ms = now;
                char unit[4];
                field(9,value,sizeof value);field(10,unit,sizeof unit);
                const uint8_t below=value[0]=='-';
                uint32_t altitude=0;
                if(unit[0]=='M' && !unit[1] && decimal(value+below,1,99999u,&altitude)) {
                    nav.altitude_decimetres=below?-(int32_t)altitude:(int32_t)altitude;
                    nav.altitude_valid=1;
                }
            }
        }
    }
    if(line[3]=='R' && line[4]=='M' && line[5]=='C') {
        char status[4],utc[16],date[8],lat[20],lon[20],ns[3],ew[3],mode[4];
        field(2,status,sizeof status);field(1,utc,sizeof utc);field(9,date,sizeof date);
        field(3,lat,sizeof lat);field(4,ns,sizeof ns);field(5,lon,sizeof lon);field(6,ew,sizeof ew);field(12,mode,sizeof mode);
        ++nav.rmc_count;nav.last_rmc_ms=now;nav.rmc_valid=0;nav.course_valid=0;nav.speed_valid=0;nav.position_valid=0;nav.time_valid=0;
        uint8_t real_mode=!mode[0] || mode[0]=='A' || mode[0]=='D' || mode[0]=='F' || mode[0]=='R';
        if(status[0]=='A' && !status[1] && real_mode && coordinate(lat,2,90) && coordinate(lon,3,180) &&
           (ns[0]=='N'||ns[0]=='S') && !ns[1] && (ew[0]=='E'||ew[0]=='W') && !ew[1]) {
            nav.rmc_valid=1;nav.position_valid=1;nav.ever_fix=1;nav.last_fix_ms=now;
            field(3,nav.lat,sizeof nav.lat);field(5,nav.lon,sizeof nav.lon);field(4,nav.ns,sizeof nav.ns);field(6,nav.ew,sizeof nav.ew);
            if(utc_valid(utc) && date_valid(date)){field(1,nav.utc,sizeof nav.utc);field(9,nav.date,sizeof nav.date);nav.time_valid=1;nav.last_time_ms=now;}
            uint32_t speed=0,course=0;
            field(7,value,sizeof value);uint8_t speed_ok=decimal(value,2,100000,&speed);nav.speed_centi_knots=speed;nav.speed_valid=speed_ok;
            field(8,value,sizeof value);
            if(speed_ok && speed>=100u && decimal(value,1,3599,&course)){nav.course_deci=(uint16_t)course;nav.course_valid=1;}
        }
    }
    if (line[3] == 'G' && line[4] == 'S' && line[5] == 'V') {
        field(3, value, sizeof value);
        uint8_t n = integer(value);
        if (line[1] == 'G' && line[2] == 'P') {
            nav.seen_gp = n; if (n > nav.max_gp) nav.max_gp = n;
        }
        if ((line[1] == 'B' && line[2] == 'D') ||
            (line[1] == 'G' && line[2] == 'B')) {
            nav.seen_bd = n; if (n > nav.max_bd) nav.max_bd = n;
        }
    }
}
/* Keep the streaming parser's branches explicit. This also avoids multi-
 * instruction IT blocks mishandled by the current Unicorn hook backend. */
#if defined(__GNUC__)
__attribute__((optimize("no-if-conversion,no-if-conversion2")))
#endif
void nav_feed(uint8_t byte, uint32_t now)
{
    nav.last_byte_ms=now;
    if (byte == '$') { count = 0; receiving = 1; }
    if (!receiving) return;
    if (byte == '\r' || byte == '\n') {
        line[count] = 0;
        if (count) parse(now);
        receiving = 0;
    } else if (count + 1 < sizeof line) line[count++] = (char)byte;
    else { receiving = 0; ++nav.bad; }
}
const nav_status_t *nav_get(void) { return &nav; }
void nav_reset(void) {nav=(nav_status_t){0};count=0;receiving=0;}

uint8_t nav_fix_fresh(uint32_t now) {return nav.position_valid && nav.ever_fix && (uint32_t)(now-nav.last_fix_ms)<3000u;}
