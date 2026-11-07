#include "core.h"
static bool frequency(uint32_t hz)
{
    return (hz>=136000000u && hz<=174000000u) ||
           (hz>=400000000u && hz<=470000000u);
}
void rt_defaults(rt_config *c)
{
    *c=(rt_config){0};
    c->watch_mode=RT_WATCH_ABC; c->watch_mask=7;
    c->dwell_ms=150; c->hold_ms=1000;
    c->gps_on=true; c->aprs_rx=true;
    c->aprs_rx_hz=144800000u; /* Initial RX preset, editable by region. */
    c->beacon_seconds=60;c->aprs_scan_seconds=10;
    /* APRS TX frequency and TX permissions require explicit user setup. */
}
bool rt_config_valid(const rt_config *c)
{
    if(c->active>2 || c->watch_mode>2 || (c->watch_mask & ~7u) ||
       c->dwell_ms<50 || c->dwell_ms>2000 || c->hold_ms>10000 ||
       c->beacon_seconds<10 || c->beacon_seconds>86400 ||
       (c->aprs_scan_seconds && (c->aprs_scan_seconds<5 || c->aprs_scan_seconds>600))) return false;
    for(unsigned i=0;i<3;i++)
        if(rt_low_fm_rx_frequency(c->vfo[i].rx_hz)) {
            if(c->vfo[i].tx_hz && (c->vfo[i].tx_hz!=c->vfo[i].rx_hz ||
               c->vfo[i].tx_hz<26000000u || c->vfo[i].tx_hz>=28000000u))return false;
        } else if((c->vfo[i].rx_hz && !frequency(c->vfo[i].rx_hz)) ||
                  (c->vfo[i].tx_hz && (!c->vfo[i].rx_hz || !frequency(c->vfo[i].tx_hz)))) return false;
    if(c->beacon_on && !frequency(c->aprs_hz)) return false;
    if(c->aprs_rx && !frequency(c->aprs_rx_hz)) return false;
    return true;
}
uint8_t rt_watch_tick(rt_watch *s,const rt_config *c,uint32_t now,
                      bool open,bool ptt)
{
    if(c->active>2) return 0;
    if(!s->initialized || s->last_active!=c->active) {
        s->initialized=true;s->last_active=c->active;s->tuned=c->active;
        s->changed=now;s->holding=false;return s->tuned;
    }
    if(s->tuned>2) s->tuned=c->active;
    if(ptt || c->watch_mode==RT_WATCH_SINGLE) {
        s->tuned=c->active;s->changed=now;s->holding=false;return s->tuned;
    }
    uint8_t mask=(c->watch_mode==RT_WATCH_ABC ? 7u:c->watch_mask) |
                 (uint8_t)(1u<<c->active);
    if(!(mask & (1u<<s->tuned))) {
        s->tuned=c->active;s->changed=now;s->holding=false;return s->tuned;
    }
    if(open) {s->holding=true;s->hold_until=now+c->hold_ms;return s->tuned;}
    if(s->holding) {
        if((int32_t)(now-s->hold_until)<0) return s->tuned;
        s->holding=false;
    }
    if((uint32_t)(now-s->changed)<c->dwell_ms) return s->tuned;
    for(unsigned i=1;i<=3;i++) {
        uint8_t next=(uint8_t)((s->tuned+i)%3);
        if(mask & (1u<<next)) {s->tuned=next;break;}
    }
    s->changed=now;return s->tuned;
}
bool rt_beacon_due(const rt_config *c,bool fix,bool voice_busy,
                   uint32_t now,uint32_t last)
{
    return c->beacon_on && frequency(c->aprs_hz) && c->gps_on && fix && !voice_busy &&
           c->beacon_seconds>=10 && (uint32_t)(now-last)>=c->beacon_seconds;
}
void rt_kiss_feed(rt_kiss *s,uint8_t b,rt_frame_cb cb,void *ctx)
{
    if(b==0xC0) {
        if(s->started && s->n && !s->invalid && !s->escaped && cb)
            cb(ctx,s->buf[0],s->buf+1,s->n-1);
        else if(s->invalid || s->escaped) ++s->rejected;
        s->started=true;s->n=0;s->invalid=false;s->escaped=false;return;
    }
    if(!s->started || s->invalid) return;
    if(s->escaped) {
        s->escaped=false;
        if(b==0xDC)b=0xC0;else if(b==0xDD)b=0xDB;
        else {s->invalid=true;return;}
    } else if(b==0xDB) {s->escaped=true;return;}
    if(s->n==sizeof s->buf) {s->invalid=true;return;}
    s->buf[s->n++]=b;
}
size_t rt_kiss_encode(uint8_t type,const uint8_t *p,size_t n,uint8_t *out,size_t cap)
{
    if(!out || (!p && n) || n>RT_KISS_MAX)return 0;
    size_t need=2+((type==0xC0 || type==0xDB)?2:1);
    for(size_t i=0;i<n;i++)need+=(p[i]==0xC0 || p[i]==0xDB)?2:1;
    if(cap<need)return 0;
    size_t j=0;out[j++]=0xC0;
    for(size_t i=0;i<=n;i++) {
        uint8_t b=i?p[i-1]:type;
        if(b==0xC0 || b==0xDB) {out[j++]=0xDB;out[j++]=b==0xC0?0xDC:0xDD;}
        else out[j++]=b;
    }
    out[j++]=0xC0;return j;
}
