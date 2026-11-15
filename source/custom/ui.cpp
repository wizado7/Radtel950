// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui.hpp"
#include "board.h"
#include "firmware_version.hpp"
#include "gps_view.hpp"
extern "C" {
#include "drivers/lcd.h"
}
namespace rt950 {
namespace {
constexpr const char *watch_names[]={"ACTIVE","A/B/C","A/B","A/C","B/C"};
static_assert(sizeof watch_names/sizeof watch_names[0]==sizeof watch_masks/sizeof watch_masks[0]);
constexpr uint16_t bg=0x0862,card=0x10C4,ink=0xFFFF,muted=0x9D55,accent=0x06DA,selected=0x1948;
void rect(unsigned x,unsigned y,unsigned w,unsigned h,uint16_t c) {lcd_fill_rect(uint16_t(x),uint16_t(y),uint16_t(w),uint16_t(h),c);}
Language draw_language=Language::English;
void label(unsigned x,unsigned y,const char *s,uint16_t c=ink,uint16_t b=card) {lcd_draw_string(uint16_t(x),uint16_t(y),translate(draw_language,s),c,b);}
void copy(char *d,const char *s) {unsigned n=0;for(;n<29 && s[n];++n)d[n]=s[n];d[n]=0;}
void num(char *d,uint32_t v) {char t[10];unsigned n=0;do{t[n++]=char('0'+v%10);v/=10;}while(v);unsigned i=0;while(n)d[i++]=t[--n];d[i]=0;}
// Calibration entry is integer milliwatts. Always show the unit and an
// exact fixed-point equivalent in watts; no floating point or guessed RF data.
void power_text(char *out,unsigned milliwatts,bool watts,bool known=true) {
 if(!known)copy(out,"--");
 else {
  num(out,watts?milliwatts/1000u:milliwatts);
  if(watts) {
   unsigned n=0;while(out[n])++n;
   out[n++]=draw_language==Language::Russian?',':'.';
   out[n++]=char('0'+milliwatts/100u%10);
   out[n++]=char('0'+milliwatts/10u%10);
   out[n++]=char('0'+milliwatts%10);out[n]=0;
  }
 }
 unsigned n=0;while(out[n])++n;
 copy(out+n,translate(draw_language,watts?" W":" mW"));
}
char *aprs_coordinate(char *out,int32_t value) {
 if(value<0){*out++='-';value=-value;}
 num(out,uint32_t(value)/100000);
 while(*out)++out;
 *out++='.';
 for(unsigned scale=10000;scale;scale/=10)*out++=char('0'+uint32_t(value)/scale%10);
 *out=0;
 return out;
}
void freq(char *d,uint32_t v) {
 if(!v){copy(d,"---.-----");return;}
 static const uint8_t positions[8]={0,1,2,4,5,6,7,8};
 d[3]='.';d[9]=0;uint32_t div=100000000;
 for(unsigned i=0;i<8;++i,div/=10)d[positions[i]]=char('0'+v/div%10);
}
bool eq(const char *a,const char *b) {while(*a && *a==*b){++a;++b;}return *a==*b;}
const char *gps_state(const Application &a,uint32_t now) {
 const auto *n=nav_get();
 if(!a.config().gps_on)return "OFF";
 if(!board_gps_bytes() && !n->good)return "NO DATA";
 if(uint32_t(now-n->last_byte_ms)>=3000u)return "STALE";
 return nav_fix_fresh(now)?"FIX":"NO FIX";
}
struct Cache { bool valid=false;Language language=Language::English;RadioSetting radio_setting=RadioSetting::Count;MenuGroup group=MenuGroup::Root;Page page=Page::Radio;char rows[16][30]{};uint8_t active=255,power_page=255; } cache;
bool changed(unsigned i,const char *s,bool force) {if(!force && eq(cache.rows[i],s))return false;copy(cache.rows[i],s);return true;}
const char *save_state_label(SaveState state) {
 switch(state) {
 case SaveState::Pending:return "SAVE PENDING";
 case SaveState::Deferred:return "SAVE DEFERRED";
 case SaveState::Failed:return "SAVE FAILED";
 case SaveState::Saved:return "SAVED";
 case SaveState::Ready:return "AUTO SAVE";
 }
 return "";
}
void panel(unsigned y,unsigned h) {rect(8,y,224,h,card);}
void stat(unsigned y,const char *name,const char *value) {panel(y,28);label(16,y+10,name,muted);label(104,y+10,value);}
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_power_status(const Application &app,const InputStatus &input,bool force) {
 const auto &p=input.power;const auto &trace=p.trace;const unsigned page=app.power_status_page();
 static constexpr const char *titles[]={"1/4 Battery readings","2/4 Last TX supply","3/4 Available sensors","4/4 Activity time"};
 if(force)label(10,52,titles[page],accent,bg);
 char value[20];
 if(page==0) {
  if(p.battery_mv)num(value,p.battery_mv);else copy(value,"--");
  if(changed(1,value,force))stat(78,"Battery mV",value);
  num(value,p.raw12);unsigned raw_length=0;while(value[raw_length])++raw_length;
  value[raw_length++]='/';num(value+raw_length,p.raw8);
  if(changed(2,value,force))stat(110,"ADC12/8",value);
  if(p.adc_age!=0xffffffffu)num(value,p.adc_age);else copy(value,"--");
  if(changed(3,value,force))stat(142,"Age ms",value);
  const char *fresh=!p.battery_mv || p.adc_age==0xffffffffu?"NO DATA":p.fresh?"FRESH":"STALE";
  if(changed(4,fresh,force))stat(174,"ADC state",translate(draw_language,fresh));
  if(p.low && p.high) {
   num(value,p.low);unsigned n=0;while(value[n])++n;value[n++]='-';num(value+n,p.high);
  } else copy(value,"--");
  if(changed(5,value,force))stat(206,"TX raw8",value);
  num(value,p.adc_errors);if(changed(6,value,force))stat(238,"ADC errors",value);
  const char *source=!p.low || !p.high?"NO LIMITS DATA":p.factory_limits?"FACTORY LIMITS":"FALLBACK LIMITS";
  if(changed(7,source,force)){rect(8,276,224,12,bg);label(10,278,source,muted,bg);}
 } else if(page==1) {
  if(trace.has_session())freq(value,trace.frequency());else copy(value,"--");
  if(changed(1,value,force))stat(78,"Last TX MHz",value);
  if(trace.before())num(value,trace.before());else copy(value,"--");
  if(changed(2,value,force))stat(110,"Before mV",value);
  if(trace.samples())num(value,trace.minimum());else copy(value,"--");
  if(changed(3,value,force))stat(142,"TX min mV",value);
  if(trace.samples())num(value,trace.maximum());else copy(value,"--");
  if(changed(4,value,force))stat(174,"TX max mV",value);
  if(trace.sag_valid()) {
   const int sag=trace.sag();value[0]=sag<0?'-':'+';num(value+1,unsigned(sag<0?-sag:sag));
  } else copy(value,"--");
  if(changed(5,value,force))stat(206,"Sag mV",value);
  num(value,trace.samples());if(changed(6,value,force))stat(238,"TX samples",value);
  const char *note=trace.fault()!=SupplyFault::None?supply_fault_text(trace.fault()):
      !trace.has_session()?"NO TX HISTORY":!trace.samples()?"NO SAMPLE DURING TX":
      trace.owner()==TxSupplyTrace::Owner::Aprs?"APRS TX SAMPLES":"VOICE TX SAMPLES";
  if(changed(7,note,force)){rect(8,276,224,12,bg);label(10,278,note,muted,bg);}
 } else if(page==2) {
  if(trace.has_session())num(value,trace.code());else copy(value,"--");
  if(changed(1,value,force))stat(78,"Last PA DAC",value);
  if(p.mcu_degrees==-32768)copy(value,translate(draw_language,"NO DATA"));
  else {
   value[0]='~';
   unsigned offset=1;
   if(p.mcu_degrees<0)value[offset++]='-';
   num(value+offset,unsigned(p.mcu_degrees<0?-p.mcu_degrees:p.mcu_degrees));
   while(value[offset])++offset;
   copy(value+offset," C");
  }
  if(changed(2,value,force))stat(110,"MCU temp",value);
  if(force) {
   stat(142,"PA temp",translate(draw_language,"NOT VERIFIED"));
   stat(174,"PA current",translate(draw_language,"NOT VERIFIED"));
   stat(206,"RF / SWR",translate(draw_language,"NOT VERIFIED"));
  }
  const char *check=p.profile_check?p.profile_check:"NO DATA";
  if(changed(6,check,force)) {
   panel(238,48);label(16,244,"Profile check",muted);
   label(16,266,check,ink);
  }
 } else {
  static constexpr const char *labels[]={"Single RX s","Watch RX s","GPS on s","BLE on s","APRS RX s","TX time s"};
  for(unsigned i=0;i<WorkloadTime::Count;++i) {
   num(value,p.workload.seconds(WorkloadTime::Activity(i)));
   if(changed(i+1,value,force))stat(78+32*i,labels[i],value);
  }
  if(force) {
   // Show all flags because the CRM may report multiple causes at once.
   constexpr const char *names[]={"PIN","POR","SW","WDT","WWDT","LP"};
   char flags[32]{};
   unsigned length=0;
   for(unsigned i=0;i<6;++i)if(p.reset_flags&(1u<<i)) {
    if(length)flags[length++]=' ';
    for(const char *name=names[i];*name;++name)flags[length++]=*name;
   }
   label(10,278,"Reset flags",muted,bg);
   label(94,278,length?flags:"--",ink,bg);
  }
 }
}
// Keep conditional Thumb blocks out of formatting used by the ARM preview hooks.
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void speed_text(char *out,const nav_status_t &nav,SpeedUnit unit,bool fresh) {
 if(!fresh){copy(out,"---");return;}
 const unsigned tenths=gps_speed_tenths(nav.speed_centi_knots,unit);
 num(out,tenths/10);
 unsigned n=0;
 while(out[n])++n;
 out[n++]='.';
 out[n++]=char('0'+tenths%10);
 out[n++]=' ';
 const char *suffix=gps_speed_suffix(unit);
 for(unsigned j=0;j<4 && suffix[j];++j)out[n++]=suffix[j];
 out[n]=0;
}
// Keep conditionals explicit in the list renderer for ARM preview hooks.
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void gps_options(const Application &a,bool force) {
  const bool altitude=a.page()==Page::AltitudeUnits, speed=a.page()==Page::SpeedUnits, distance=a.page()==Page::DistanceUnits;
  const char *title=altitude?"GPS ALTITUDE UNITS":speed?"GPS SPEED UNITS":distance?"Distance units":"GPS COORDINATE FORMAT";
  static constexpr const char *altitude_options[]={"Meters (m)","Feet (ft)"};
  static constexpr const char *speed_options[]={"Kilometers/hour (km/h)","Miles/hour (mph)","Knots (kn)"};
  static constexpr const char *coordinate_options[]={"Degrees + minutes","Decimal degrees","Degrees minutes seconds"};
  static constexpr const char *distance_options[]={"Kilometers (km)","Miles (mi)","Nautical miles (nm)"};
  const auto options=altitude?altitude_options:speed?speed_options:distance?distance_options:coordinate_options;
  const unsigned selection=altitude?unsigned(a.altitude_draft_feet()):speed?unsigned(a.speed_draft()):distance?unsigned(a.distance_draft()):a.coordinate_draft();
  const unsigned applied=altitude?unsigned(a.altitude_feet()):speed?unsigned(a.speed_unit()):distance?unsigned(a.distance_unit()):a.coordinate_selection();
  if(force)label(10,53,title,accent,bg);
  for(unsigned i=0;i<(altitude?2u:3u);++i) {
   char signature[5]={char('0'+i),char(i==selection?'1':'0'),char(i==applied?'1':'0'),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned y=76+i*36;
   const uint16_t fill=i==selection?selected:card;
   rect(8,y,224,30,fill);rect(8,y,3,30,i==selection?accent:card);
   label(17,y+11,options[i],ink,fill);
   if(i==applied)label(212,y+11,"*",accent,fill);
  }
  if(force)label(10,202,"* CURRENT SETTING",muted,bg);
}
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void radio_options(const Application &a,bool force) {
  const auto setting=a.radio_setting();
  const unsigned selection=a.radio_draft();
  const unsigned applied=radio_profile_option(a.active_radio_profile(),setting);
  const unsigned first=selection>=5?selection-4:0;
  if(force) {
   label(10,53,radio_setting_label(setting),accent,bg);
   label(10,266,"* CURRENT SETTING",muted,bg);
   label(10,282,"RF OFF",muted,bg);
  }
  for(unsigned row=0;row<5 && first+row<radio_option_count(setting);++row) {
   const unsigned index=first+row,y=76+row*36;
   char key[16],value[8];num(key,index);
   unsigned n=0;while(key[n])++n;
   key[n++]=index==selection?'S':'-';key[n++]=index==applied?'A':'-';key[n]=0;
   if(!changed(row+1,key,force))continue;
   const uint16_t fill=index==selection?selected:card;
   rect(8,y,224,30,fill);rect(8,y,3,30,index==selection?accent:card);
   label(17,y+11,radio_option_label(setting,index,value),ink,fill);
   if(index && (setting==RadioSetting::RxCtcss || setting==RadioSetting::TxCtcss))label(81,y+11,"Hz",muted,fill);
   if(index==applied)label(212,y+11,"*",accent,fill);
  }
}
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_fm_probe(const Application &app,const InputStatus &input,bool force) {
  using Phase=fm::Probe::Phase;
  using Failure=fm::Probe::Failure;
  const char *state="FM RESET";
  if(input.fm_phase==Phase::InitialCts)state="FM WAITING FOR CTS";
  if(input.fm_phase==Phase::PowerWait || input.fm_phase==Phase::PowerCts)state="FM POWER UP";
  if(input.fm_phase==Phase::Configure)state="FM CONFIGURE";
  if(input.fm_phase==Phase::Revision)state="FM READ REVISION";
  if(input.fm_phase==Phase::Receiver)state=input.fm_frequency?"FM TUNED":"FM TUNING";
  if(app.fm_audio() && app.fm_seek())state="FM SEEKING";
  if(input.fm_audio_error)state="FM AUDIO ERROR";
  if(input.fm_audio_enabled)state="FM LISTENING";
  if(input.fm_phase==Phase::Fault) {
   switch(input.fm_failure) {
   case Failure::Bus:state="FM BUS ERROR";break;
   case Failure::Chip:state="FM CHIP ERROR";break;
   case Failure::NoReply:state="FM NO I2C REPLY";break;
   case Failure::Timeout:state="FM CTS TIMEOUT";break;
   case Failure::TuneTimeout:state="FM TUNE TIMEOUT";break;
   case Failure::TuneFrequency:state="FM WRONG FREQUENCY";break;
   default:state="FM ERROR";break;
   }
  }
  if(app.sw_am() && input.fm_phase==Phase::Receiver)
   state=input.fm_audio_enabled?"AM LISTENING":input.fm_frequency?"AM TUNED":"AM TUNING";
  if(changed(1,state,force)){panel(52,28);label(16,62,state,accent);}
  char part[8]="--",hz[12],rssi[8]="--",snr[8]="--";
  if(input.fm_part)num(part,input.fm_part);
  if(!input.fm_frequency)copy(hz,"--");
  else if(app.sw_am())num(hz,input.fm_frequency);
  else {
   num(hz,input.fm_frequency/100);
   unsigned n=input.fm_frequency>=10000?3:2;
   hz[n++]='.';hz[n++]=char('0'+input.fm_frequency/10%10);
   hz[n++]=char('0'+input.fm_frequency%10);hz[n]=0;
  }
  if(input.fm_signal_ready){num(rssi,input.fm_signal.rssi);num(snr,input.fm_signal.snr);}
  if(changed(2,part,force))stat(88,"FM PART",part);
  if(changed(3,hz,force))stat(120,app.sw_am()?"AM kHz":"FM MHz",hz);
  if(changed(4,rssi,force)) {
   stat(152,"RSSI",rssi);
   if(input.fm_signal_ready) {
    const unsigned strength=input.fm_signal.rssi>60?60:input.fm_signal.rssi;
    rect(142,164,72,4,selected);
    if(strength)rect(142,164,strength*72/60+1,4,accent);
   }
  }
  if(changed(5,snr,force))stat(184,"SNR",snr);
  const char *quality="FM NO READING";
  if(input.fm_signal_ready) {
   switch(fm::quality(input.fm_signal)) {
   case fm::Quality::NoStation:quality="FM NO STATION";break;
   case fm::Quality::Weak:quality="FM WEAK";break;
   case fm::Quality::Fair:quality="FM FAIR";break;
   case fm::Quality::Good:quality="FM GOOD";break;
   }
  }
  if(changed(6,quality,force)){panel(216,28);label(16,226,quality,muted);}
  const char *flags=!input.fm_signal_ready?"":input.fm_signal.afc_railed?"AFC LIMIT":input.fm_signal.valid?"VALID":"INVALID";
  if(changed(7,flags,force)){panel(245,20);label(16,249,flags,muted);}
  if(app.fm_audio()) {
   const char *entry=*app.feedback()?app.feedback():*app.fm_entry()?app.fm_entry():app.sw_am()?"OK: enter frequency kHz":"OK: 10030 = 100.30";
   if(changed(8,entry,force)){rect(8,270,224,16,bg);label(10,271,entry,ink,bg);}
   if(force && !app.sw_am())label(10,293,"Right: seek   ABC: save",muted,bg);
  } else if(force)label(10,271,"FM TEST / AUDIO OFF",muted,bg);
}
void point_text(char *out,const GeoPoint &point,bool latitude) {
 if(!point.valid){copy(out,"--");return;}
 const int32_t coordinate=latitude?point.latitude:point.longitude;
 out[0]=coordinate<0?(latitude?'S':'W'):(latitude?'N':'E');out[1]=' ';
 const unsigned magnitude=unsigned(coordinate<0?-coordinate:coordinate),whole=magnitude/1000000u;
 num(out+2,whole);unsigned n=2u+(whole>=100u?3u:whole>=10u?2u:1u);
 out[n++]='.';for(unsigned scale=100000;scale;scale/=10)out[n++]=char('0'+magnitude/scale%10u);
 out[n]=0;
}
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_navigation(const Application &a,uint32_t now,bool force) {
 const auto &target=a.waypoint();const bool setup=a.page()==Page::Waypoint;
 if(force)label(10,52,setup?"Saved GPS point":"Distance to point",accent,bg);
 char latitude[30],longitude[30];point_text(latitude,target,true);point_text(longitude,target,false);
 if(setup) {
  const char *status=target.valid?"SAVED TARGET":"NO SAVED GPS POINT";
  if(changed(1,status,force)){rect(8,72,224,16,bg);label(10,76,status,accent,bg);}
  if(changed(2,latitude,force)){rect(8,96,224,16,bg);label(10,100,latitude,ink,bg);}
  if(changed(3,longitude,force)){rect(8,120,224,16,bg);label(10,124,longitude,ink,bg);}
  const char *mode=a.waypoint_confirm()?(a.waypoint_clear()?"CLEAR":"CAPTURE"):"LIST";
  const bool redraw=changed(4,mode,force);
  if(redraw)rect(8,156,224,100,bg);
  if(a.waypoint_confirm()) {
   if(redraw)label(10,178,a.waypoint_clear()?"CLEAR SAVED POINT?":"SAVE CURRENT GPS POINT?",ink,bg);
  } else {
   const char *actions[]={"Use current GPS fix","Clear saved point"};
   for(unsigned i=0;i<2;++i) {
    const bool selected_row=bool(i)==a.waypoint_clear();
    if(!changed(5+i,selected_row?"1":"0",force || redraw))continue;
    const unsigned y=164+40*i;const uint16_t fill=selected_row?selected:card;
    rect(8,y,224,30,fill);label(16,y+11,actions[i],ink,fill);
   }
  }
  if(changed(8,a.feedback(),force)){rect(8,268,224,20,bg);label(10,273,a.feedback(),muted,bg);}
  return;
 }
 GeoPoint current{};const bool fresh=a.config().gps_on && gps_current_point(now,current);
 static GeoPoint previous{},previous_target{};static GeoRange cached{};
 if(!fresh || !target.valid)cached={};
 else if(!cached.valid || current.latitude!=previous.latitude || current.longitude!=previous.longitude ||
         target.latitude!=previous_target.latitude || target.longitude!=previous_target.longitude) {
  cached=geo_range(current,target);previous=current;previous_target=target;
 }
 char distance[30],bearing[30];
 if(!cached.valid){copy(distance,"--");copy(bearing,"--");}
 else {
  const unsigned value=distance_thousandths(cached.metres,a.distance_unit()),whole=value/1000u;
  num(distance,whole);unsigned n=whole>=10000u?5u:whole>=1000u?4u:whole>=100u?3u:whole>=10u?2u:1u;
  distance[n++]='.';distance[n++]=char('0'+value/100u%10u);distance[n++]=char('0'+value/10u%10u);distance[n++]=char('0'+value%10u);
  copy(distance+n,a.distance_unit()==DistanceUnit::Kilometres?" km":a.distance_unit()==DistanceUnit::Miles?" mi":" nm");
  if(!cached.bearing_valid)copy(bearing,"--");
  else {const unsigned degrees=cached.bearing_deci/10u;num(bearing,degrees);unsigned k=degrees>=100u?3u:degrees>=10u?2u:1u;
   bearing[k++]='.';bearing[k++]=char('0'+cached.bearing_deci%10u);copy(bearing+k," deg");}
 }
 if(changed(1,distance,force))stat(80,"Distance",distance);
 if(changed(2,bearing,force))stat(120,"Azimuth",bearing);
 const char *status=!target.valid?"NO SAVED GPS POINT":!fresh?"FRESH GPS FIX REQUIRED":"SAVED TARGET";
 if(changed(3,status,force)){rect(8,160,224,16,bg);label(10,164,status,accent,bg);}
 if(changed(4,latitude,force)){rect(8,192,224,16,bg);label(10,196,latitude,ink,bg);}
 if(changed(5,longitude,force)){rect(8,224,224,16,bg);label(10,228,longitude,ink,bg);}
 if(force){label(10,260,"SPHERICAL DISTANCE",muted,bg);label(10,280,"TRUE NORTH / NO COMPASS",muted,bg);}
}
// Keep list selection branches explicit, as in gps_options, for the
// LCD-hook ARM model and independent rendering of each row.
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_tuner_steps(const Application &a,bool force) {
  const bool am=a.page()==Page::AmStep;
  if(force)label(10,52,am?"AM tuning step":"FM tuning step",accent,bg);
  for(unsigned i=0;i<(am?4u:3u);++i) {
   const bool highlight=i==a.tuner_step_draft();
   char signature[]={char('0'+i),char(highlight?'1':'0'),char(i==a.tuner_step_selection(am)?'1':'0'),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned khz=am?Application::am_steps[i]:Application::fm_steps[i]*10u;
   char value[16];num(value,khz);
   copy(value+(khz>=100u?3u:khz>=10u?2u:1u)," kHz");
   const unsigned y=76+36*i;const uint16_t fill=highlight?selected:card;
   rect(8,y,224,30,fill);label(17,y+11,value,ink,fill);
   if(i==a.tuner_step_selection(am))label(212,y+11,"*",accent,fill);
  }
  if(force){label(10,244,"MANUAL TUNING ONLY",muted,bg);label(10,266,"* CURRENT SETTING",muted,bg);}
}
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_broadcast_presets(const Application &a,bool force) {
  const bool am=a.broadcast_am();
  const char *mode=a.broadcast_confirm()?(a.broadcast_deleting()?"DELETE":"SAVE"):"LIST";
  const bool redraw=changed(9,mode,force);
  if(redraw)rect(8,48,224,244,bg);
  if(force || redraw)label(10,52,am?"AM presets":"FM presets",accent,bg);
  if(a.broadcast_confirm()) {
   if(redraw) {
    label(10,88,a.broadcast_deleting()?"DELETE THIS PRESET?":"SAVE FREQUENCY HERE?",ink,bg);
    char value[30];num(value,a.broadcast_selection()+1);
    label(10,112,"Preset",muted,bg);label(100,112,value,accent,bg);
    const unsigned hz=a.broadcast_deleting()?a.broadcast_frequency(a.broadcast_selection()):a.broadcast_source();
    if(am)num(value,hz);
    else {num(value,hz/100);unsigned n=0;while(value[n])++n;value[n++]='.';value[n++]=char('0'+hz/10%10);value[n++]=char('0'+hz%10);value[n]=0;}
    label(10,144,value,accent,bg);label(100,144,am?"kHz":"MHz",muted,bg);
   }
  } else {
   for(unsigned i=0;i<Application::BroadcastCount;++i) {
    char value[30];const unsigned hz=a.broadcast_frequency(i);
    if(!hz)copy(value,"EMPTY");
    else if(am)num(value,hz);
    else {num(value,hz/100);unsigned n=0;while(value[n])++n;value[n++]='.';value[n++]=char('0'+hz/10%10);value[n++]=char('0'+hz%10);value[n]=0;}
    char signature[30];signature[0]=i==a.broadcast_selection()?'1':'0';copy(signature+1,value);
    if(!changed(i+1,signature,force || redraw))continue;
    const unsigned y=70+24*i;const uint16_t fill=i==a.broadcast_selection()?selected:card;
    rect(8,y,224,22,fill);char ordinal[2]={char('1'+i),0};
    label(16,y+7,ordinal,muted,fill);label(44,y+7,value,ink,fill);
    if(hz)label(162,y+7,am?"kHz":"MHz",muted,fill);
   }
   const char *hint=*a.feedback()?a.feedback():"SAVE CURRENT FREQUENCY";
   // This line is independent of the eight row signatures.
   if(changed(12,hint,force || redraw)) {
    rect(8,270,224,18,bg);label(10,275,hint,muted,bg);
   }
  }
}
// Confirmation is opt-in and rendered with the same bounded list layout.
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_reset(const Application &a,bool force) {
 if(force) {
  label(10,52,a.reset_all()?"Reset all user data":"Reset settings",accent,bg);
  label(10,76,a.reset_all()?"64 DEFAULT CHANNELS":"CHANNELS / BANKS KEPT",muted,bg);
  label(10,98,a.reset_all()?"BANKS / GPS POINT CLEAR":"GPS POINT KEPT",muted,bg);
  label(10,122,"LANGUAGE / RF POINTS KEPT",muted,bg);
  label(10,144,"BOOT / FACTORY CAL KEPT",muted,bg);
 }
 for(unsigned i=0;i<2;++i) {
  const bool active=i==unsigned(a.reset_confirm());
  const char signature[]={char('0'+i),char('0'+active),0};
  if(!changed(i+1,signature,force))continue;
  const unsigned y=190+i*40;const uint16_t fill=active?selected:card;
  rect(8,y,224,32,fill);label(16,y+12,i?"Confirm reset":"Cancel",active?accent:ink,fill);
 }
}
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_import(const Application &a,const InputStatus &input,bool force) {
 if(force)label(10,52,"Import settings",accent,bg);
 if(changed(9,a.import_ready()?"CHECKED":"RECEIVE",force)){rect(8,74,224,218,bg);force=true;}
 if(a.import_ready()) {
  if(force){label(10,78,"REPLACE SETTINGS / CHANNELS",muted,bg);label(10,100,"RF KEPT / APRS TX OFF",muted,bg);}
  char count[12];num(count,input.import_channels);if(changed(3,count,force))stat(122,"Channels",count);
  for(unsigned i=0;i<2;++i) {
   const bool active=i==unsigned(a.import_confirm());
   char signature[]={char('0'+i),char('0'+active),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned y=182+40*i;const uint16_t fill=active?selected:card;
   rect(8,y,224,32,fill);label(16,y+12,i?"Apply import":"Cancel",active?accent:ink,fill);
  }
 } else {
  const char *state="Import not started";
  switch(input.import_phase) {
  case ImportPhase::Waiting:state="Waiting for computer";break;
  case ImportPhase::Receiving:state="Receiving settings";break;
  case ImportPhase::Error:state="Import rejected";break;
  case ImportPhase::Timeout:state="Import timed out";break;
  case ImportPhase::Cancelled:state="Import cancelled";break;
  default:break;
  }
  if(changed(1,state,force)){panel(82,36);label(16,96,state,accent);}
  char count[16];num(count,input.import_received);
  if(changed(2,count,force))stat(132,"Bytes",count);
  if(force){label(10,204,"USB CABLE / 115200",muted,bg);label(10,230,"CONFIRM ON RADIO",muted,bg);label(10,256,"PTT CANCELS IMPORT",muted,bg);}
 }
}
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_key_binding(const Application &a,bool force) {
 if(force)label(10,52,Application::key_binding_labels[a.binding_selection()],accent,bg);
 constexpr unsigned rows=8;
 const unsigned draft=a.binding_draft(),first=draft<rows?0:draft-rows+1;
 for(unsigned row=0;row<rows;++row) {
  const unsigned i=first+row;
  const bool active=i==a.binding_draft();
  const bool applied=i==unsigned(a.key_action(a.binding_selection()));
  const char signature[]={char('0'+i),char('0'+active),char('0'+applied),0};
  if(!changed(row+1,signature,force))continue;
  const unsigned y=72+row*24;const uint16_t fill=active?selected:card;
  rect(8,y,224,22,fill);label(16,y+7,Application::key_action_labels[i],active?accent:ink,fill);
  if(applied)label(214,y+7,"*",accent,fill);
 }
 if(force)label(10,278,a.binding_selection()<2 && a.side_ptt_enabled()?"PF1 USED BY PTT C":"HOLD 700 MS / RELEASE",muted,bg);
}
// Explicit branches match the other LCD-model-tested status pages.
__attribute__((noinline,optimize("no-if-conversion","no-if-conversion2")))
void draw_backup(const InputStatus &input,bool force) {
  if(force)label(10,52,"Export settings",accent,bg);
  const char *backup_state="Export not started";
  switch(input.backup_phase) {
  case BackupPhase::Waiting:backup_state="Waiting for computer";break;
  case BackupPhase::Sending:backup_state="Exporting settings";break;
  case BackupPhase::Complete:backup_state="Backup exported";break;
  case BackupPhase::Cancelled:backup_state="Export cancelled";break;
  case BackupPhase::Timeout:backup_state="Export timed out";break;
  case BackupPhase::Error:backup_state="Export error";break;
  default:break;
  }
  if(changed(1,backup_state,force)){panel(82,36);label(16,96,backup_state,accent);}
  char progress[20];num(progress,input.backup_sent);
  const unsigned n=input.backup_sent>=1000u?4u:input.backup_sent>=100u?3u:input.backup_sent>=10u?2u:1u;
  progress[n]='/';num(progress+n+1,input.backup_total);
  if(changed(2,progress,force))stat(132,"Bytes",progress);
  if(force) {
   panel(180,104);label(16,194,"USB CABLE / 115200",muted);
   label(16,222,"READ ONLY / NO FLASH WRITE",muted);
   label(16,250,"PTT CANCELS EXPORT",muted);
  }
}
}
void ui_invalidate(){cache.valid=false;}
void ui_boot_screen() {
 rect(0,0,240,320,bg);rect(32,86,176,3,accent);
 lcd_draw_string_2x(48,114,"RADTEL",ink,bg);
 label(72,153,"by wizado",accent,bg);
#ifdef RT950_BENCH_TX_HZ
#if defined(RT950_MAIN_TX)
 label(48,202,"CUSTOM " RT950_CUSTOM_VERSION,muted,bg);
#elif defined(RT950_BENCH_MATRIX)
 label(48,202,"RF ACCEPTANCE TX",muted,bg);
#else
 label(48,202,"BENCH TX ONLY",muted,bg);
#endif
#else
 label(48,202,"CUSTOM " RT950_CUSTOM_VERSION,muted,bg);
#endif
 label(72,224,"STARTING...",muted,bg);
}
void ui_draw(Application &a,const InputStatus &input,uint32_t now) {
 draw_language=a.language();
 const auto radio_setting=a.page()==Page::RadioOptions?a.radio_setting():RadioSetting::Count;
 const bool force=!cache.valid || cache.page!=a.page() || cache.language!=draw_language || cache.radio_setting!=radio_setting || (a.page()==Page::Settings && cache.group!=a.menu_group()) || (a.page()==Page::PowerStatus && cache.power_page!=a.power_status_page());
 cache.power_page=uint8_t(a.power_status_page());
 cache.group=a.menu_group();
 cache.radio_setting=radio_setting;
 cache.language=draw_language;
 if(a.page()==Page::LcdTest) {
  if(changed(0,a.lcd_test_gray()?"GRAY":"BLACK",force))rect(0,0,240,320,a.lcd_test_gray()?0x8410:0);
  cache.valid=true;cache.page=a.page();return;
 }
 if(force) {
  rect(0,0,240,320,bg);rect(0,0,240,26,card);
  label(10,9,"RADTEL",accent);
  rect(8,294,224,1,selected);

 }
 char battery[16];const uint16_t mv=board_battery_mv();
 if(mv) {num(battery,board_battery_percent_from_mv(mv));unsigned j=0;while(battery[j])++j;battery[j++]='%';battery[j]=0;}
 else copy(battery,"BAT ?");
 if(changed(11,battery,force)) {
  constexpr unsigned right=232,glyph_width=8;
  unsigned length=0;while(battery[length])++length;
  rect(right-5*glyph_width,5,5*glyph_width,16,card);
  label(right-length*glyph_width,9,battery,muted);
 }
 if(changed(12,a.keypad_locked()?"LOCK":"",force)){rect(88,5,88,16,card);if(a.keypad_locked())label(88,9,"LOCK",accent);}
 const bool presets=a.page()==Page::FmPresets || a.page()==Page::AmPresets;
 const char *footer=a.page()==Page::Timezone?"OK SAVE / BACK CANCEL":a.page()==Page::Weather?(a.weather().alerted()?"OK ACK / BACK EXIT":
     a.weather().scanning()?"OK LISTEN / BACK EXIT":"OK SCAN / BACK EXIT"):
     a.page()==Page::Import?"OK START / BACK CANCEL":a.page()==Page::Backup?"OK START / BACK CANCEL":a.page()==Page::Waypoint && a.waypoint_confirm()?"OK CONFIRM / BACK CANCEL":(a.page()==Page::Distance || a.page()==Page::FlashInfo)?"READ ONLY / BACK RETURN":presets?(a.broadcast_confirm()?"OK CONFIRM / BACK CANCEL":"LEFT SAVE / * DEL / OK PLAY"):a.page()==Page::About?"READ ONLY / BACK RETURN":a.page()==Page::PowerMeasured?"OK SAVE / * DEL / BACK EXIT":a.page()==Page::PowerCalibration?"RIGHT: RECORD / BACK EXIT":a.keypad_locked()?"HOLD # TO UNLOCK":a.page()==Page::VoiceRx?"READ ONLY / BACK RETURN":a.page()==Page::PowerStatus?"ARROWS PAGE / BACK RETURN":a.page()==Page::Debug?"BACK HOLD / LED 1 2 0":a.page()==Page::Gps?"HOLD VM GPS / BACK RETURN":a.page()==Page::Radio?(a.editing()?"HOLD OK MENU   * DELETE":a.config().vfo[a.config().active].channel_mode?"VM VFO   WHEEL CHANNEL":"OK MENU   ABC SLOT   VM CH"):"OK SELECT       BACK EXIT";
 if(changed(10,footer,force)){rect(8,300,224,16,bg);label(10,304,footer,muted,bg);}
 const char *state=gps_state(a,now);char line[30];
 if(a.page()==Page::Settings) {
  static constexpr const char *titles[]={"Settings","Radio","Channels & zones","GPS settings","APRS settings","Bluetooth settings","Display & power","System","Diagnostics","APRS RX Setup","Beacon Setup","Repeater","User keys"};
  static_assert(sizeof(titles)/sizeof(titles[0])==unsigned(MenuGroup::Count));
  copy(line,titles[unsigned(a.menu_group())]);
 } else if(a.page()==Page::PowerStatus)copy(line,"TX DISABLED ON THIS PAGE");
 else if(a.page()==Page::FmProbe && a.sw_am()) {
  const auto band=a.broadcast_band();
  copy(line,band==fm::Band::Longwave?"Longwave AM":
      band==fm::Band::Mediumwave?"Mediumwave AM":"Shortwave AM");
 } else {copy(line,"GPS ");copy(line+4,state);}
 if(changed(0,line,force)) {rect(8,32,224,12,bg);label(10,34,line,accent,bg);}
 if(a.page()==Page::Radio) {
  const auto &c=a.config();
  const int heard=a.recent_voice_slot();
  for(unsigned i=0;i<3;++i) {
   const bool sending=input.voice_tx_slot==i;
   char hz[12];freq(hz,sending?c.vfo[i].tx_hz:c.vfo[i].rx_hz);
   char key[30];copy(key,hz);key[10]=0; // append mode/channel to the cache signature
   const bool receiving=input.voice.ready && input.voice.receiving && input.voice.slot==i &&
       input.voice.frequency==c.vfo[i].rx_hz && !input.ptt_mask;
   const bool recent=heard==int(i);
   const bool split=c.vfo[i].tx_hz && c.vfo[i].rx_hz!=c.vfo[i].tx_hz;
   unsigned n=0;while(key[n])++n;key[n++]=c.vfo[i].channel_mode?'M':'V';key[n++]=split?'S':c.vfo[i].tx_hz?'T':'R';key[n++]=a.reverse_active(i)?'V':'-';key[n++]=sending?'T':receiving?'R':recent?'L':'-';num(key+n,c.vfo[i].memory_channel);
   const bool redraw=changed(i+1,key,force) || cache.active!=c.active;
   if(!redraw)continue;
   const unsigned y=52+i*70;const bool active=c.active==i;
   const uint16_t fill=active?selected:card;
   rect(8,y,224,64,fill);rect(8,y,4,64,active?accent:card);
   char slot[2]={char('A'+i),0};label(18,y+9,slot,active?accent:muted,fill);
   label(44,y+9,c.vfo[i].channel_mode?"CH":"VFO",muted,fill);
   if(c.vfo[i].channel_mode) {char channel[12];num(channel,c.vfo[i].memory_channel+1);label(76,y+9,channel,muted,fill);label(104,y+9,a.memory(c.vfo[i].memory_channel)->name,muted,fill);}
   label(174,y+9,sending?"TX":receiving?"RX":active?"ACTIVE":recent?"LAST RX":"STANDBY",(sending || receiving || active)?accent:muted,fill);
   lcd_draw_string_2x(18,uint16_t(y+29),hz,ink,fill);label(178,y+43,"MHz",muted,fill);
   if(c.vfo[i].rx_hz && !c.vfo[i].tx_hz)label(170,y+29,"NO TX",muted,fill);
   else if(a.reverse_active(i))label(170,y+29,"REV",accent,fill);
   else if(split && !sending)label(170,y+29,"SPLIT",muted,fill);
  }
  cache.active=c.active;
#ifdef RT950_BENCH_TX_HZ
  if(input.bench_ptt[0]) {
   if(eq(input.bench_ptt,"TX END LEVEL INVALID")) {
    constexpr char hex[]="0123456789ABCDEF";
    copy(line,"TX END LEVEL ");
    line[13]=hex[input.factory_ctcss_raw>>4];line[14]=hex[input.factory_ctcss_raw&15];
    line[15]='/';line[16]=hex[input.factory_ctcss_level>>4];line[17]=hex[input.factory_ctcss_level&15];
    line[18]=0;
   } else copy(line,input.bench_ptt);
  }
#else
  if(input.ptt_mask)copy(line,"RX ONLY");
#endif
  else
  if(input.monitor_active)copy(line,"MONITOR ACTIVE");
  else if(a.feedback()[0]) copy(line,a.feedback());
  else if(a.editing()) {copy(line,"Enter ");a.entry_text(line+6);copy(line+15," OK");}
  else if(input.voice.fault)copy(line,"RX BUS FAULT");
  else if(input.power_status[0])copy(line,input.power_status);
  else if(heard>=0 && heard!=c.active) {
   constexpr const char *select_rx[]={"Right: select RX A","Right: select RX B","Right: select RX C"};
   copy(line,select_rx[heard]);
  }
  else if(c.watch_mode==RT_WATCH_SINGLE)copy(line,"Watch: selected");
  else {
   const uint8_t mask=c.watch_mode==RT_WATCH_ABC?7u:uint8_t(c.watch_mask|(1u<<c.active));
   copy(line,mask==3?"Watch: A/B":mask==5?"Watch: A/C":mask==6?"Watch: B/C":"Watch: A/B/C");
  }
  if(changed(4,line,force)) {rect(8,266,224,20,bg);label(10,271,line,a.editing()?accent:muted,bg);}
  if(force)label(10,287,"Left: Watch slots",muted,bg);
 } else if(a.page()==Page::Settings) {
  const unsigned sel=a.menu_selection(),first=sel<6?0:sel-5;
  for(unsigned row=0;row<6;++row) {
   const unsigned idx=first+row,y=52+row*35;
   if(idx>=a.menu_count()) {
    if(changed(row+1,"",force))rect(8,y,224,30,bg);
    continue;
   }
   const auto &entry=a.menu_entry(idx);
   const char *status="OPEN";
   if(entry.toggle!=Option::Count) {
    status=a.enabled(entry.toggle)?"ON":"OFF";
   } else if(entry.action==Key::Mode)status=a.power_save()?"ON":"OFF";
   char key[30];copy(key,entry.name);unsigned n=0;while(key[n])++n;
   key[n++]=idx==sel?'>':' ';key[n++]=status[0];key[n++]=status[1];key[n++]=status[2];key[n]=0;
   if(!changed(row+1,key,force))continue;
   const uint16_t fill=idx==sel?selected:card;
   rect(8,y,224,30,fill);rect(8,y,3,30,idx==sel?accent:card);
   label(17,y+7,entry.name,ink,fill);
   label(17,y+19,status,muted,fill);
  }
  const bool repeater=a.menu_group()==MenuGroup::Repeater;
  char pair[30]="RX ";const auto &v=a.config().vfo[a.config().active];
  if(repeater) {
   freq(pair+3,v.rx_hz);copy(pair+12," TX ");freq(pair+16,v.tx_hz);
  } else pair[0]=0;
  if(repeater && changed(7,pair,force)){rect(8,230,224,18,bg);label(10,234,pair,accent,bg);}
  copy(line,a.feedback()[0]?a.feedback():repeater?a.tx_status():"ARROWS CHOOSE / OK SELECT");
  if(changed(8,line,force)){rect(8,266,224,20,bg);label(10,271,line,muted,bg);}
 } else if(a.page()==Page::AprsMessages) {
  const unsigned count=input.aprs_messages.count();
  const unsigned index=count?a.aprs_message_index()%count:0;
  const auto *message=input.aprs_messages.at(index);
  char from[30]{},to[30]{};
  if(message) {
   copy(from,message->source);
   unsigned end=0;while(from[end])++end;
   if(message->ssid){from[end++]='-';num(from+end,message->ssid);}
   copy(to,message->destination);
  }
  char page[]={char(count?'1'+index:'0'),'/','3',0};
  if(force)label(16,54,"APRS messages",accent,bg);
  if(changed(1,page,force))stat(76,"MESSAGE",page);
  if(changed(2,from,force))stat(108,"FROM",from[0]?from:"---");
  if(changed(3,to,force))stat(140,"TO",to[0]?to:"---");
  unsigned offset=0;
  for(unsigned row=0;row<3;++row) {
   char text[27]{};
   for(unsigned i=0;message && i<26 && message->body[offset];++i)
    text[i]=message->body[offset++];
   if(changed(4+row,text,force)){
    rect(8,176+row*16,224,16,card);lcd_draw_string(16,178+row*16,text,ink,card);
   }
  }
  if(force)label(16,240,"ARROWS SELECT MESSAGE",muted,bg);
 } else if(a.page()==Page::ClearAprs) {
  if(force){panel(52,46);label(16,63,"Clear APRS list",accent);label(16,79,"OK: CLEAR  BACK: CANCEL");}
 } else if(a.page()==Page::BeaconSymbol) {
  if(force)label(10,52,"Beacon symbol",accent,bg);
  for(unsigned i=0;i<sizeof beacon_symbols/sizeof beacon_symbols[0];++i) {
   const bool highlight=i==a.beacon_symbol_draft();
   char signature[]={char('0'+i),char(highlight?'1':'0'),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned y=72+36*i;
   const uint16_t fill=highlight?selected:card;
   rect(8,y,224,30,fill);label(17,y+11,beacon_symbols[i].label,ink,fill);
   if(i==a.beacon_symbol_index())label(212,y+11,"*",accent,fill);
  }
 } else if(a.page()==Page::KeyBinding) {
  draw_key_binding(a,force);
 } else if(a.page()==Page::Import) {
  draw_import(a,input,force);
 } else if(a.page()==Page::Reset) {
  draw_reset(a,force);
 } else if(a.page()==Page::Backup) {
  draw_backup(input,force);
 } else if(a.page()==Page::Waypoint || a.page()==Page::Distance) {
  draw_navigation(a,now,force);
 } else if(a.page()==Page::FmStep || a.page()==Page::AmStep) {
  draw_tuner_steps(a,force);
 } else if(a.page()==Page::About) {
  if(force) {
   label(10,52,"About firmware",accent,bg);
   panel(76,100);label(16,88,"RT950 Custom UI",ink);
   label(16,112,RT950_CUSTOM_VERSION,accent);
   label(16,144,"TEST BUILD / NOT RELEASE",muted);
   panel(188,94);label(16,200,"C++17 / GPL-3.0-or-later",muted);
   label(16,224,"by wizado",accent);
   label(16,248,"OEM BOOTLOADER RETAINED",muted);
  }
  char memory[30];num(memory,board_flash_kib());
  unsigned length=0;while(memory[length])++length;copy(memory+length," KiB flash");
  if(changed(1,memory,force)){rect(16,164,208,10,card);label(16,164,memory,muted);}
 } else if(presets) {
  draw_broadcast_presets(a,force);
 } else if(a.page()==Page::Monitor) {
  if(force)label(10,52,"Monitor",accent,bg);
  for(unsigned i=0;i<2;++i) {
   const bool highlight=i==unsigned(a.monitor_draft());
   char signature[]={char('0'+i),char(highlight?'1':'0'),char(i==unsigned(a.monitor_enabled())?'1':'0'),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned y=76+36*i;const uint16_t fill=highlight?selected:card;
   rect(8,y,224,30,fill);label(17,y+11,i?"ON":"OFF",ink,fill);
   if(i==unsigned(a.monitor_enabled()))label(212,y+11,"*",accent,fill);
  }
  if(force) {
   label(10,172,"ACTIVE SLOT AUDIO",muted,bg);
   label(10,198,"BYPASS SQL / RX TONES",muted,bg);
   label(10,224,"PTT KEEPS TX SETTINGS",muted,bg);
   label(10,266,"* CURRENT SETTING",muted,bg);
  }
 } else if(a.page()==Page::MicGain) {
  static constexpr const char *gains[]={"Low","Middle","High (factory)"};
  if(force)label(10,52,"Mic Gain",accent,bg);
  for(unsigned i=0;i<3;++i) {
   const bool highlight=i==unsigned(a.mic_gain_draft());
   char signature[]={char('0'+i),char(highlight?'1':'0'),char(i==unsigned(a.mic_gain())?'1':'0'),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned y=76+36*i;const uint16_t fill=highlight?selected:card;
   rect(8,y,224,30,fill);label(17,y+11,gains[i],ink,fill);
   if(i==unsigned(a.mic_gain()))label(212,y+11,"*",accent,fill);
  }
  if(force) {
   label(10,208,"GLOBAL VOICE SETTING",muted,bg);
   label(10,230,"HIGH USES FACTORY LEVEL",muted,bg);
   label(10,266,"* CURRENT SETTING",muted,bg);
  }
 } else if(a.page()==Page::PowerStatus) {
  draw_power_status(a,input,force);
 } else if(a.page()==Page::BandProfile) {
  const auto power=a.band_power();const unsigned count=preset_channels(power);
  const unsigned choice=a.band_draft_channel();
  const unsigned selected_channel=choice && choice<=count?choice:a.band_channel();
  const unsigned first=selected_channel<=3?1:selected_channel+2>count?count-4:selected_channel-2;
  if(force)label(10,52,"LPD / PMR profiles (RU)",accent,bg);
  const char *band=power==TxPower::Lpd10mW?"LPD: 10 mW target":"PMR: 0.5 W target";
  if(changed(1,band,force)){rect(8,68,224,20,bg);label(10,74,band,accent,bg);}
  for(unsigned row=0;row<5;++row) {
   const unsigned channel=first+row,y=94+row*30;
   const uint32_t hz=preset_frequency(power,channel);
   const bool record=a.measured_power().find(power,hz,a.power_battery())!=nullptr;
   char signature[24],hz_text[16];num(signature,channel);
   unsigned n=0;while(signature[n])++n;
   signature[n++]=channel==choice?'S':'-';signature[n++]=record?'R':'-';
   signature[n++]=power==TxPower::Lpd10mW?'L':'P';signature[n]=0;
   if(!changed(row+2,signature,force))continue;
   const uint16_t fill=channel==choice?selected:card;
   rect(8,y,224,26,fill);
   char number[8];num(number,channel);freq(hz_text,hz);
   label(16,y+9,number,muted,fill);label(48,y+9,hz_text,ink,fill);
   if(record)label(212,y+9,"*",accent,fill);
  }
  if(force)label(10,250,"LEFT/RIGHT BAND, OK APPLY",muted,bg);
  char entered_channel[24];
  const char *note=*a.feedback()?a.feedback():"* STORED MEASUREMENT";
  if(!*a.feedback() && a.editor_digits()) {
   copy(entered_channel,translate(draw_language,"Channel"));
   unsigned n=0;while(entered_channel[n])++n;
   entered_channel[n++]=' ';num(entered_channel+n,a.editor_value());note=entered_channel;
  }
  if(changed(8,note,force)){rect(8,266,224,20,bg);label(10,271,note,muted,bg);}
 } else if(a.page()==Page::PowerClear) {
  if(force) {
   label(10,52,"CLEAR POWER RECORDS?",accent,bg);
   panel(84,100);label(16,98,"Custom measurements only",ink);
   label(16,126,"Factory data preserved",muted);
   label(16,154,"OK DELETE / BACK CANCEL",accent);
  }
 } else if(a.page()==Page::PowerCalibration || a.page()==Page::PowerMeasured) {
  const bool measuring=a.page()==Page::PowerMeasured;
  if(force)label(10,52,measuring?"Enter meter reading":"TX power calibration",accent,bg);
  char hz[16],value[16],limit[12],voltage[12],count[8];
  freq(hz,a.config().vfo[a.config().active].tx_hz);
  num(value,a.editor_digits()?a.editor_value():measuring?0:a.power_code());
  num(limit,a.power_code_limit());num(voltage,a.power_battery());num(count,a.measured_power().count());
  unsigned count_length=0;while(count[count_length])++count_length;
  copy(count+count_length,"/16");
  if(changed(9,count,force)){rect(192,48,40,16,bg);label(192,52,count,muted,bg);}
  if(changed(1,hz,force))stat(74,"TX MHz",hz);
  const char *target=a.active_radio_profile().power==TxPower::Lpd10mW?"10 mW target":"0.5 W target";
  if(changed(2,target,force))stat(102,"Target",translate(draw_language,target));
  if(measuring)power_text(value,a.editor_value(),false,a.editor_digits()!=0);
  if(changed(3,value,force))stat(130,measuring?"Measured":"DAC code",value);
  if(measuring) {
   char watts[20];power_text(watts,a.editor_value(),true,a.editor_digits()!=0);
   if(changed(4,watts,force))stat(158,"In watts",watts);
   num(limit,a.power_code());
   if(changed(5,limit,force))stat(186,"DAC code",limit);
  } else {
   if(changed(4,limit,force))stat(158,"Low DAC",limit);
   if(changed(5,voltage,force))stat(186,"Battery mV",voltage);
  }
  const auto *saved=a.measured_power().find(a.active_radio_profile().power,
      a.config().vfo[a.config().active].tx_hz,a.power_battery());
  char recorded[12]="--";
  power_text(recorded,saved?saved->measured_mw:0,false,saved && saved->code==a.power_code());
  if(changed(6,recorded,force))stat(214,"Recorded",recorded);
  const char *power_state=measuring?"OK: save measured mW":a.power_test(a.config().active)?"TEST ARMED: HOLD PTT":"OK: arm 5-second test";
#ifdef RT950_BENCH_TX_HZ
  if(input.bench_ptt[0])power_state=input.bench_ptt;
#endif
  if(changed(7,power_state,force)){rect(8,244,224,18,bg);label(10,248,power_state,accent,bg);}
  const char *hint=*a.feedback()?a.feedback():measuring?"TYPE RF METER READING":"CODE IS NOT POWER";
  if(changed(8,hint,force)){rect(8,266,224,20,bg);label(10,271,hint,muted,bg);}
  if(force)label(10,286,measuring?"1 mW = 0.001 W":"50 OHM LOAD + RF METER",muted,bg);
 } else if(a.page()==Page::MenuTimeout || a.page()==Page::AutoLock) {
  const bool automatic=a.page()==Page::AutoLock;
  const unsigned draft=automatic?a.auto_lock_draft():a.menu_timeout_draft();
  const unsigned applied=automatic?a.auto_lock_selection():a.menu_timeout_selection();
  if(force)label(10,52,automatic?"Auto keypad lock":"Menu timeout",accent,bg);
  for(unsigned i=0;i<sizeof Application::menu_timeouts/sizeof Application::menu_timeouts[0];++i) {
   const bool highlight=i==draft;
   char signature[]={char('0'+i),char(highlight?'1':'0'),char(i==applied?'1':'0'),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned y=72+36*i;
   const uint16_t fill=highlight?selected:card;
   rect(8,y,224,30,fill);label(17,y+11,Application::menu_timeout_labels[i],ink,fill);
   if(i==applied)label(212,y+11,"*",accent,fill);
  }
  if(force)label(10,266,"* CURRENT SETTING",muted,bg);
 } else if(a.page()==Page::ScanMode) {
  static constexpr const char *modes[]={"Scan by time","Scan by carrier","Stop on signal"};
  if(force)label(10,52,"SCAN MODE",accent,bg);
  for(unsigned i=0;i<3;++i) {
   const bool highlight=i==unsigned(a.scan_mode_draft());
   char signature[]={char('0'+i),char(highlight?'1':'0'),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned y=72+36*i;
   const uint16_t fill=highlight?selected:card;
   rect(8,y,224,30,fill);label(17,y+11,modes[i],ink,fill);
   if(i==unsigned(a.scan_mode()))label(212,y+11,"*",accent,fill);
  }
 } else if(a.scan_page()) {
  const bool vfo_scan=a.page()==Page::VfoScan;
  const auto *memory=a.memory(a.scan_channel());
  const char *scan_status=input.ptt_mask?"SCAN PAUSED":a.scan_frequency()?
      (a.scan_paused()?"SCAN PAUSED":input.voice.receiving?"SCAN SIGNAL":"SCANNING"):
      vfo_scan?"SET FREQUENCY FIRST":"NO CHANNELS TO SCAN";
#ifdef RT950_MAIN_TX
  if(input.bench_ptt[0])scan_status=input.bench_ptt;
#endif
  if(changed(1,scan_status,force)) {
   panel(52,28);label(16,55,vfo_scan?"VFO SCAN":"CHANNEL SCAN",muted);label(16,67,scan_status);
  }
  if(!vfo_scan) {
   if(changed(2,a.zone_name(a.active_zone()),force))stat(84,"ZONE",a.zone_name(a.active_zone()));
   char channel[20]="---";
   if(memory) {
    num(channel,a.scan_channel()+1);
    unsigned end=0;while(channel[end])++end;
    channel[end++]=' ';copy(channel+end,memory->name);
   }
   if(changed(3,channel,force))stat(116,"CHANNEL",channel);
  }
  char value[16];freq(value,a.scan_frequency());
  if(changed(4,value,force))stat(148,"MHz",value);
  const char *direction=a.scan_reverse()?"SCAN DOWN":"SCAN UP";
  if(changed(6,direction,force))stat(180,"DIRECTION",direction);
  const char *hint=a.scan_paused()?(vfo_scan?"OK: USE FREQ   >: RESUME":"OK: USE CHANNEL   >: RESUME"):"OK PAUSE / ARROWS STEP";
  if(changed(5,hint,force)){rect(8,228,224,17,bg);label(10,232,hint,muted,bg);}
  const char *extra=a.feedback()[0]?a.feedback():!vfo_scan && a.scan_frequency()?"LEFT: EXCLUDE CHANNEL":"";
  if(changed(7,extra,force)){rect(8,248,224,17,bg);label(10,252,extra,muted,bg);}
 } else if(a.page()==Page::VoiceRx) {
  const auto &rx=input.voice;
  const char *rx_state=rx.fault?"RX BUS FAULT":!rx.ready?"RX STARTING":rx.receiving?"RX CARRIER":"RX LISTENING";
  if(changed(1,rx_state,force)){panel(52,28);label(16,62,rx_state,accent); }
  char slot[2]={char('A'+rx.slot),0};
  if(changed(2,slot,force))stat(88,"RX SLOT",slot);
  char value[16];freq(value,rx.frequency);
  if(changed(3,value,force))stat(120,"MHz",value);
  num(value,rx.rssi);
  if(changed(4,value,force))stat(152,"RSSI RAW",value);
  num(value,rx.status);
  if(changed(5,value,force))stat(184,"REG 0C",value);
  const char *mode=watch_names[a.watch_slots_selection()];
  if(changed(6,mode,force))stat(216,"MODE",mode);
  const char *hardware=!rx.sampled?"RX STARTING":voice::hardware_sql_open(rx.status)?"SQL OPEN":"SQL CLOSED";
  if(changed(7,hardware,force))stat(248,"RF SQL BIT",hardware);
  if(force)label(10,282,"READ ONLY / PTT DISABLED",muted,bg);
 } else if(a.page()==Page::Weather) {
  const auto &wx=a.weather();
  const char *title=wx.scanning() || wx.alerted()?"NOAA Alert":"NOAA";
  if(changed(1,title,force)){rect(8,52,224,18,bg);label(10,55,title,accent,bg);}
  char channel[8]="WX-";num(channel+3,wx.channel()+1);
  if(changed(2,channel,force))stat(80,"Channel",channel);
  char value[16];freq(value,wx.frequency());
  if(changed(3,value,force))stat(116,"MHz",value);
  const auto &rx=input.voice;
  const char *status=rx.fault?"RX BUS FAULT":wx.alerted()?"WX SIGNAL FOUND":
      wx.scanning()?"WX ALERT SCANNING":!rx.sampled?"RX STARTING":
      voice::hardware_sql_open(rx.status)?"RX CARRIER":"RX LISTENING";
  if(changed(4,status,force))stat(152,"RX",status);
 } else if(a.page()==Page::FmProbe) {
  draw_fm_probe(a,input,force);
 } else if(a.page()==Page::FlashInfo) {
  if(force) {
   char value[16];const unsigned kib=board_flash_kib();
   if(kib==256 || kib==512 || kib==1024)num(value,kib);else copy(value,"UNKNOWN");
   stat(52,"FLASH KiB",value);
   const unsigned size=unsigned(a.compact_snapshot_size());num(value,size);stat(154,"Data bytes",value);
   if(size && size<=8168u && board_storage_probe()!=255)num(value,8168u-size);else copy(value,"--");
   stat(188,"Free bytes",value);num(value,unsigned(a.canonical_snapshot_size()));stat(222,"Raw bytes",value);
   panel(260,28);label(16,270,"COMPACT V2 / AUTO SAVE",muted);
  }
  const unsigned slots=board_storage_probe();
  const char *first=slots==255?"UNKNOWN":slots&1u?"NOT EMPTY":"EMPTY";
  const char *second=slots==255?"UNKNOWN":slots&2u?"NOT EMPTY":"EMPTY";
  if(changed(1,first,force))stat(86,"SLOT A",first);
  if(changed(2,second,force))stat(120,"SLOT B",second);
 } else if(a.page()==Page::Zones) {
  const char *title=a.channel_action()==Key::MoveChannel?"Destination zone":a.channel_action()==Key::RenameZone?"Rename zone":"Select zone";
  if(force)label(10,53,title,accent,bg);
  const unsigned sel=a.zone_selection(),first=sel>=5?sel-4:0;
  for(unsigned row=0;row<5;++row) {
   const unsigned index=first+row,y=76+row*36;
   char key[16];copy(key,a.zone_name(index));unsigned n=0;while(key[n])++n;
   key[n++]=char('A'+index);key[n++]=index==sel?'S':'-';key[n++]=index==a.active_zone()?'A':'-';key[n]=0;
   if(!changed(row+1,key,force))continue;
   const uint16_t fill=index==sel?selected:card;
   rect(8,y,224,30,fill);rect(8,y,3,30,index==sel?accent:card);
   char number[4];num(number,index+1);label(17,y+11,number,muted,fill);label(52,y+11,a.zone_name(index),ink,fill);
   if(index==a.active_zone())label(212,y+11,"*",accent,fill);
  }
 } else if(a.page()==Page::EditName) {
  char value[12];bool ended=false;
  for(unsigned i=0;i<8;++i){if(!a.name_draft()[i])ended=true;value[i]=ended?'_':a.name_draft()[i];}
  value[8]=char('0'+a.name_cursor());value[9]=0;
  if(changed(1,value,force)) {
   value[8]=0;panel(52,96);label(16,65,a.channel_action()==Key::RenameZone?"Rename zone":"Rename channel",accent);
   lcd_draw_string_2x(16,96,value,ink,card);rect(16+a.name_cursor()*16,116,16,2,accent);
  }
  if(force){panel(160,64);label(16,174,"UP / DOWN OR WHEEL",muted);label(16,197,"LEFT / RIGHT: CHARACTER",muted);}
  if(changed(8,a.feedback(),force)){rect(8,266,224,20,bg);label(10,271,a.feedback(),muted,bg);}
 } else if(a.page()==Page::ConfirmDelete) {
  if(force){panel(52,88);label(16,66,"Delete this channel?",accent);label(16,98,a.memory(a.channel_selection())->name);}
  if(force)label(10,266,"OK DELETE / BACK CANCEL",muted,bg);
 } else if(a.page()==Page::ChannelCommands) {
  if(force) {
   const auto &memory=*a.memory(a.channel_selection());
   char title[18]="CH ";num(title+3,a.channel_selection()+1);
   unsigned end=0;while(title[end])++end;
   title[end++]=' ';
   const char *name=memory.name;
   while(*name)title[end++]=*name++;
   title[end]=0;
   label(10,52,title,accent,bg);
   char value[16];
   freq(value,memory.rx_hz);stat(211,"Base RX MHz",value);
   freq(value,memory.tx_hz);stat(245,"Base TX MHz",value);
   label(10,280,memory.radio.tx_inhibit?"TX INHIBITED":"",muted,bg);
  }
  for(unsigned i=0;i<Application::ChannelCommandCount;++i) {
   const bool active=i==a.channel_command();
   if(!changed(i+1,active?"selected":"idle",force))continue;
   const unsigned y=73+i*33;
   const uint16_t fill=active?selected:card;
   rect(8,y,224,28,fill);label(16,y+10,Application::channel_commands[i],ink,fill);
  }
 } else if(a.page()==Page::ConfirmStore) {
  if(force) {
   const auto &memory=*a.memory(a.channel_selection());
   char channel[8]="CH ";num(channel+3,a.channel_selection()+1);
   panel(52,55);label(16,63,"Replace this channel?",accent);
   label(16,86,channel);label(72,86,memory.name);
   char value[16];freq(value,memory.rx_hz);stat(120,"Saved MHz",value);
   freq(value,a.config().vfo[a.config().active].rx_hz);stat(154,"New MHz",value);
   label(10,266,"OK REPLACE / BACK CANCEL",muted,bg);
  }
 } else if(a.page()==Page::Channels || a.page()==Page::StoreChannel || a.page()==Page::ManageChannel) {
  unsigned occupied[Application::MemoryCount],count=0;
  for(unsigned i=0;i<Application::MemoryCount;++i)if(a.channel_visible(i) || (a.page()==Page::StoreChannel && !a.memory(i)->occupied))occupied[count++]=i;
  unsigned selected_row=0;for(unsigned i=0;i<count;++i)if(occupied[i]==a.channel_selection())selected_row=i;
  const unsigned first=selected_row<6?0:selected_row-5;
  for(unsigned row=0;row<6;++row) {
   char name[30]{};const unsigned n=first+row;
   if(n<count){
    const auto &memory=*a.memory(occupied[n]);
    copy(name,"CH ");num(name+3,occupied[n]+1);
    unsigned j=0;while(name[j])++j;name[j++]=' ';
    if(memory.occupied){
     copy(name+j,memory.name);while(name[j])++j;name[j++]=' ';
     freq(name+j,memory.rx_hz);
    } else copy(name+j,translate(draw_language,"FREE SLOT"));
    while(name[j])++j;
    name[j++]=occupied[n]==a.channel_selection()?'>':' ';
    if(a.page()==Page::Channels && !a.scan_eligible(occupied[n]))name[j++]='X';
    name[j]=0;
   }
   if(changed(row+1,name,force)) {const uint16_t fill=n<count&&occupied[n]==a.channel_selection()?selected:card;rect(8,52+row*35,224,30,fill);label(17,63+row*35,name,ink,fill);}
  }
  const bool entering=a.page()!=Page::ManageChannel && a.editor_digits();
  const char *hint=a.feedback()[0]?a.feedback():entering?"Channel number":
     a.page()==Page::ManageChannel?(!count?"NO SAVED CHANNELS":
     a.channel_action()==Key::RenameChannel?"Rename channel":a.channel_action()==Key::DeleteChannel?"Delete channel":"Move channel"):
     a.page()==Page::StoreChannel?"OK SAVE / TYPE CH NUMBER":count?"OK LOAD   >: SCAN ON/OFF":"EMPTY: SAVE FROM VFO FIRST";
  char channel_number[8]{};
  if(entering && !a.feedback()[0])num(channel_number,a.editor_value());
  const bool hint_changed=changed(8,hint,force),number_changed=changed(10,channel_number,force);
  if(hint_changed || number_changed){
   rect(8,266,224,16,bg);label(10,271,hint,muted,bg);
   if(channel_number[0])label(155,271,channel_number,ink,bg);
  }
  const char *save_status=save_state_label(input.save_state);
  if(changed(9,a.zone_name(a.active_zone()),force)){rect(8,282,72,10,bg);label(10,283,a.zone_name(a.active_zone()),accent,bg);}
  if(changed(7,save_status,force)){rect(88,282,144,10,bg);label(88,283,save_status,muted,bg);}
  if(force && a.page()==Page::Channels) {
   rect(8,300,224,16,bg);
   label(10,304,count?"LEFT: ACTIONS / ABC: ZONES":"ABC: ZONES",muted,bg);
  }
 } else if(a.page()==Page::Battery) {
  char adc[16],errors[16];num(adc,board_battery_raw());num(errors,board_battery_errors());
  char volts[16];if(mv){num(volts,mv/1000);unsigned v=0;while(volts[v])++v;volts[v++]='.';for(unsigned scale=100;scale;scale/=10)volts[v++]=char('0'+mv/scale%10);volts[v++]='V';volts[v]=0;}else copy(volts,"UNKNOWN");
  if(changed(1,volts,force))stat(52,"VOLTAGE",volts);
  if(changed(2,adc,force))stat(88,"ADC 8BIT",adc);
  if(changed(3,errors,force))stat(124,"ERRORS",errors);
  if(force){panel(164,84);label(16,177,"ADC2 / PA1 / CHANNEL 1",muted);label(16,199,"OEM 0.29 VOLTAGE SCALE",muted);label(16,221,"ESTIMATE: 6.4V TO 8.32V",muted);}
 } else if(a.page()==Page::BeaconIdentity) {
  const auto &id=a.identity_draft();
  char value[16];bool end=false;
  for(unsigned i=0;i<6;++i){if(!id.callsign[i])end=true;value[i]=end?'_':id.callsign[i];}
  value[6]='-';value[7]=char('0'+id.ssid/10);value[8]=char('0'+id.ssid%10);
  value[9]=char('0'+a.identity_cursor());value[10]=0;
  if(changed(1,value,force)) {
   value[9]=0;panel(52,96);label(16,65,"BEACON CALLSIGN / SSID",accent);
   lcd_draw_string_2x(16,96,value,ink,card);
   const unsigned cursor=a.identity_cursor();
   rect(16+(cursor==6?7:cursor)*16,116,cursor==6?32:16,2,accent);
  }
  if(force){panel(160,88);label(16,174,"UP / DOWN OR WHEEL",muted);label(16,196,"LEFT / RIGHT: CHARACTER",muted);label(16,218,"ABC: SSID   * DELETE",muted);}
  if(changed(8,a.feedback(),force)){rect(8,266,224,20,bg);label(10,271,a.feedback(),muted,bg);}
 } else if(a.page()==Page::RadioOptions) {
  radio_options(a,force);
 } else if(a.page()==Page::LanguageSelect || a.page()==Page::VfoStep) {
  const bool language=a.page()==Page::LanguageSelect;
  const unsigned selection=language?unsigned(a.language_draft()):a.vfo_step_selection();
  const unsigned applied=language?unsigned(a.language()):a.applied_vfo_step();
  const unsigned first=!language && selection>=5?selection-4:0;
  const unsigned count=language?2:5;
  if(force)label(10,53,language?"LANGUAGE":"VFO TUNING STEP",accent,bg);
  for(unsigned row=0;row<count;++row) {
   const unsigned index=first+row,y=76+row*36;
   char key[4]={char('0'+index),char(index==selection?'1':'0'),char(index==applied?'1':'0'),0};
   if(!changed(row+1,key,force))continue;
   const uint16_t fill=index==selection?selected:card;
   rect(8,y,224,30,fill);rect(8,y,3,30,index==selection?accent:card);
   if(language)label(17,y+11,index?u8"Русский":"English",ink,fill);
   else {
    const uint32_t hz=Application::vfo_steps[index];
    char value[16];num(value,hz);label(17,y+11,value,ink,fill);label(81,y+11,"Hz",muted,fill);
   }
   if(index==applied)label(212,y+11,"*",accent,fill);
  }
  if(force)label(10,266,"* CURRENT SETTING",muted,bg);
 } else if(a.page()==Page::AltitudeUnits || a.page()==Page::SpeedUnits || a.page()==Page::CoordinateFormat || a.page()==Page::DistanceUnits) {
  gps_options(a,force);
 } else if(a.frequency_editor()) {
  const bool rx=a.page()==Page::AprsRxFrequency, tx=a.page()==Page::TxFrequency, offset=a.page()==Page::RepeaterOffset;
  char value[16];
  if(a.editor_digits()) {
   num(value,a.editor_value());
   // Preserve any leading zeroes so invalid drafts remain visible.
   unsigned n=0;while(value[n])++n;
   const unsigned pad=a.editor_digits()-n;
   for(unsigned i=n+1;i>0;--i)value[i-1+pad]=value[i-1];
   for(unsigned i=0;i<pad;++i)value[i]='0';
  } else if(offset && !a.active_radio_profile().offset_hz)copy(value,"000.00000");
  else freq(value,rx?a.config().aprs_rx_hz:tx?a.profile_tx_frequency():offset?a.active_radio_profile().offset_hz:a.config().aprs_hz);
  if(changed(1,value,force)){panel(52,92);label(16,65,rx?"APRS RX FREQUENCY":tx?"TX frequency":offset?"Repeater offset":"BEACON FREQUENCY",accent);lcd_draw_string_2x(16,96,value,ink,card);}
  if(force){
   panel(160,72);label(16,174,offset?"TYPE 000600 FOR 0.600 MHz":"TYPE 144800 FOR 144.800 MHz",muted);
   label(16,198,tx?"LEFT: TX = RX / RIGHT: CLEAR":"RF OFF",muted);
   if(tx)label(16,216,"OK APPLY / BACK CANCEL",muted);
  }
  if(changed(8,a.feedback(),force)){rect(8,266,224,20,bg);label(10,271,a.feedback(),muted,bg);}
 } else if(a.page()==Page::WatchSlots) {
  if(force)label(10,52,"WATCH SLOTS",accent,bg);
  for(unsigned i=0;i<sizeof watch_names/sizeof watch_names[0];++i) {
   const bool highlight=i==a.watch_slots_draft();
   char signature[]={char('0'+i),char(highlight?'1':'0'),0};
   if(!changed(i+1,signature,force))continue;
   const unsigned y=72+36*i;
   const uint16_t fill=highlight?selected:card;
   rect(8,y,224,30,fill);label(17,y+11,watch_names[i],ink,fill);
   if(i==a.watch_slots_selection())label(212,y+11,"*",accent,fill);
  }
  if(force)label(10,263,"ACTIVE SLOT ALWAYS INCLUDED",muted,bg);
 } else if(a.scan_bound_editor()) {
  const bool upper=a.page()==Page::ScanEnd;
  const uint32_t value_hz=a.editor_digits()?a.editor_value():a.scan_bound(upper);
  char value[16];
  if(a.editor_digits())num(value,value_hz);
  else if(value_hz)freq(value,value_hz);
  else copy(value,"AUTO");
  if(changed(1,value,force)) {
   panel(52,92);label(16,65,upper?"VFO SCAN UPPER LIMIT":"VFO SCAN LOWER LIMIT",accent);
   lcd_draw_string_2x(16,96,value,ink,card);
  }
  if(force){panel(160,72);label(16,174,"0 = AUTO BAND EDGE",muted);label(16,198,"OK SAVE, BACK RETURN",muted);}
  if(changed(8,a.feedback(),force)){rect(8,266,224,20,bg);label(10,271,a.feedback(),muted,bg);}
 } else if(a.watch_timer_editor()) {
  const bool dwell=a.page()==Page::WatchDwell;
  char value[16];num(value,a.editor_digits()?a.editor_value():dwell?a.config().dwell_ms:a.config().hold_ms);
  if(changed(1,value,force)) {
   panel(52,92);label(16,65,dwell?"WATCH SLOT TIME (MS)":"WATCH RESUME (MS)",accent);
   lcd_draw_string_2x(16,96,value,ink,card);
  }
  if(force){panel(160,72);label(16,174,dwell?"USE 100 TO 2000 MS":"USE 0 TO 10000 MS",muted);label(16,198,"OK SAVE, BACK RETURN",muted);}
  if(changed(8,a.feedback(),force)){rect(8,266,224,20,bg);label(10,271,a.feedback(),muted,bg);}
 } else if(a.page()==Page::BeaconInterval || a.page()==Page::AprsScanInterval) {
  const bool scan=a.page()==Page::AprsScanInterval;
  char value[16];num(value,a.editor_digits()?a.editor_value():scan?a.config().aprs_scan_seconds:a.config().beacon_seconds);
  if(changed(1,value,force)){panel(52,92);label(16,65,scan?"SCAN PAUSE (SEC)":"BEACON INTERVAL (SEC)",accent);lcd_draw_string_2x(16,96,value,ink,card);}
  if(force){panel(160,72);label(16,174,scan?"USE 0 OR 5 TO 600 SECONDS":"TYPE 10 TO 86400 SECONDS",muted);label(16,198,scan?"0 = BACKGROUND OFF":"OK SAVE, BACK RETURN",muted);}
  if(changed(8,a.feedback(),force)){rect(8,266,224,20,bg);label(10,271,a.feedback(),muted,bg);}
 } else if(a.page()==Page::Brightness || a.page()==Page::ScreenTimer || a.page()==Page::Timezone) {
  char value[30];
  if(a.page()==Page::Brightness){num(value,a.brightness()*25u);unsigned j=0;while(value[j])++j;value[j++]='%';value[j]=0;}
  else if(a.page()==Page::ScreenTimer)num(value,a.editor_digits()?a.editor_value():a.screen_seconds());
  else {int zone=a.timezone_draft();copy(value,zone<0?"UTC -":"UTC +");if(zone<0)zone=-zone;value[5]=char('0'+zone/60/10);value[6]=char('0'+zone/60%10);value[7]=':';value[8]=char('0'+zone%60/10);value[9]=char('0'+zone%10);value[10]=0;}
  if(changed(1,value,force)){panel(52,92);label(16,65,a.page()==Page::Brightness?"LCD BRIGHTNESS":a.page()==Page::ScreenTimer?"SCREEN TIMEOUT SECONDS":"GPS LOCAL TIMEZONE",accent);lcd_draw_string_2x(16,96,value,ink,card);}
  if(force) {
   panel(160,88);
   if(a.page()==Page::Timezone) {
    label(16,174,"UP/DOWN/WHEEL: 15 MIN",muted);
    label(16,196,"LEFT/RIGHT: 30 MIN",muted);
    label(16,218,"ABC: +1 HOUR",muted);
   } else {
    label(16,174,a.page()==Page::ScreenTimer?"TYPE SECONDS: 0 TO 3600":"UP / DOWN OR WHEEL",muted);
    label(16,196,a.page()==Page::ScreenTimer?"0 = ALWAYS ON, * DELETE":"OK SAVE, BACK RETURN",muted);
    label(16,218,"RADIO POWER STAYS ON",muted);
   }
  }
  copy(line,a.feedback());if(changed(8,line,force)){rect(8,266,224,20,bg);label(10,271,line,muted,bg);}
 } else if(a.page()==Page::Gps) {
  const auto *n=nav_get();const bool fix=nav_fix_fresh(now);
  const bool heading=fix && n->course_valid && uint32_t(now-n->last_rmc_ms)<3000u;
  const bool speed_fresh=fix && n->speed_valid && uint32_t(now-n->last_rmc_ms)<3000u;
  char speed[16];
  speed_text(speed,*n,a.speed_unit(),speed_fresh);
  char signature[30];num(signature,heading?n->course_deci:9999);unsigned j=0;while(signature[j])++j;signature[j++]=' ';copy(signature+j,speed);
  if(changed(1,signature,force)) {
   panel(52,100);label(116,57,"N",muted);label(116,125,"S",muted);label(76,91,"W",muted);label(156,91,"E",muted);
   rect(118,78,2,34,selected);rect(102,94,34,2,selected);
   if(heading){static const int8_t dx[8]={0,17,24,17,0,-17,-24,-17},dy[8]={-24,-17,0,17,24,17,0,-17};unsigned sector=((n->course_deci+225u)/450u)%8u;int x=120,y=94,tx=x+dx[sector],ty=y+dy[sector],sx=tx>x?1:-1,sy=ty>y?1:-1;int ax=tx>x?tx-x:x-tx,ay=ty>y?ty-y:y-ty,err=ax-ay;for(unsigned k=0;k<32;++k){rect(unsigned(x),unsigned(y),2,2,accent);if(x==tx&&y==ty)break;int e=err*2;if(e> -ay){err-=ay;x+=sx;}if(e<ax){err+=ax;y+=sy;}}
    char v[16];num(v,n->course_deci/10);j=0;while(v[j])++j;copy(v+j," deg");label(16,141,v,accent);
   }else label(16,141,"NO COURSE",muted);
   label(128,141,speed,speed_fresh?ink:muted);
  }
  char time[12]="--:--:--",date[12]="----------";gps_local_time(*n,a.timezone_minutes(),now,time,date);
  if(changed(2,time,force))stat(160,"LOCAL TIME",time);
  if(changed(3,date,force))stat(192,"DATE",date);
  char lat[30],lon[30];copy(lat,fix?"LAT ":"LAST LAT ");j=0;while(lat[j])++j;if(n->ever_fix){lat[j++]=n->ns[0];lat[j++]=' ';gps_coordinate(n->lat,2,a.decimal_coordinates(),lat+j,a.dms_coordinates());}else copy(lat+j,"---");
  copy(lon,fix?"LON ":"LAST LON ");j=0;while(lon[j])++j;if(n->ever_fix){lon[j++]=n->ew[0];lon[j++]=' ';gps_coordinate(n->lon,3,a.decimal_coordinates(),lon+j,a.dms_coordinates());}else copy(lon+j,"---");
  if(changed(4,lat,force)){panel(224,25);label(16,233,lat,fix?ink:muted);}
  if(changed(5,lon,force)){panel(254,25);label(16,263,lon,fix?ink:muted);}
  char altitude[16]="---";
  if(a.config().gps_on && gps_altitude_fresh(*n,now)) {
   const int32_t height=gps_altitude_display(n->altitude_decimetres,a.altitude_feet());
   unsigned k=0;if(height<0)altitude[k++]='-';
   num(altitude+k,unsigned(height<0?-height:height));
   while(altitude[k])++k;
   copy(altitude+k,a.altitude_feet()?" ft":" m");
  }
  if(changed(6,altitude,force)){rect(112,212,112,8,card);label(112,212,"ALT",muted);label(144,212,altitude);}
  char hint[30];copy(hint,"RMC:");num(hint+4,n->rmc_count>9999?9999:n->rmc_count);j=0;while(hint[j])++j;copy(hint+j," E:");j+=3;
  uint32_t errors=board_gps_errors(),drops=board_gps_dropped();num(hint+j,errors>9999?9999:errors);while(hint[j])++j;copy(hint+j," D:");j+=3;num(hint+j,drops>9999?9999:drops);
  if(changed(8,hint,force)){rect(8,282,224,10,bg);label(10,283,hint,muted,bg);}
 } else if(a.page()==Page::Debug) {
  char values[7][30]{};
  num(values[0],input.last);
  if(input.sampled==255)copy(values[1],"RELEASED");else num(values[1],input.sampled);
  num(values[2],input.presses);num(values[3],input.encoder_ab);
  num(values[4],input.cw);num(values[5],input.ccw);num(values[6],input.edges);
  const char *names[7]={"LAST KEY","LIVE KEY","EVENTS","ENC AB","TURN +","TURN -","EDGES"};
  for(unsigned i=0;i<7;++i)if(changed(i+1,values[i],force))stat(52+i*32,names[i],values[i]);
  copy(line,"BAD ");num(line+4,input.invalid_edges>9999?9999:input.invalid_edges);unsigned pos=0;while(line[pos])++pos;copy(line+pos," LOST ");pos+=6;num(line+pos,input.encoder_lost>9999?9999:input.encoder_lost);
  if(changed(8,line,force)){rect(8,274,224,9,bg);label(10,274,line,muted,bg);}
  char ptt[]="PTT1 UP PTT2 UP PF1 UP";
  if(input.ptt_mask&1u){ptt[5]='D';ptt[6]='N';}
  if(input.ptt_mask&2u){ptt[13]='D';ptt[14]='N';}
  if(input.sampled==0){ptt[20]='D';ptt[21]='N';}
  if(changed(9,ptt,force)){rect(8,284,224,9,bg);label(10,284,ptt,accent,bg);}
 } else if(a.page()==Page::Aprs) {
  const char *aprs_state=input.aprs_receiving?"APRS RX":a.config().aprs_rx?"APRS WAIT":"APRS OFF";
  if(force)panel(52,84);
  if(changed(1,aprs_state,force)){rect(18,64,204,12,card);label(18,65,aprs_state,accent);}
  char value[24];freq(value,a.config().aprs_rx_hz);
  if(changed(2,value,force)){rect(18,86,204,24,card);lcd_draw_string_2x(18,86,value,ink,card);}
  num(value,input.aprs_frames);
  if(changed(3,value,force))stat(138,"AX.25 RX",value);
  auto show_count=[&](unsigned row,unsigned y,const char *name,uint32_t count) {
   char number[11];num(number,count);
   if(changed(row,number,force)) {
    rect(8,y,224,9,bg);
    label(10,y,name,muted,bg);
    label(152,y,number,ink,bg);
   }
  };
  show_count(12,112,"BG WINDOWS",input.aprs_windows);
  show_count(13,124,"BG CANCELLED",input.aprs_cancelled);
  show_count(14,270,"BG FRAMES",input.aprs_background_frames);
  show_count(15,282,"BLE QUEUE DROPS",input.aprs_export_dropped);
  for(unsigned i=0;i<aprs::RecentStations::Capacity;++i) {
   const auto *entry=input.aprs_stations.at(i);
   char identity[30]{},coordinates[30]{};
   if(entry) {
    const bool station=entry->position.kind==aprs::PositionKind::Station;
    copy(identity,station?entry->position.callsign:entry->position.name);
    unsigned n=0;while(identity[n])++n;
    if(station && entry->position.ssid){identity[n++]='-';num(identity+n,entry->position.ssid);}
    n=0;while(identity[n])++n;
    const uint32_t age=uint32_t(now-entry->received_at)/1000u;
    identity[n++]=' ';identity[n++]=' ';
    num(identity+n,age<60?age:age/60);
    while(identity[n])++n;
    identity[n++]=age<60?'s':'m';identity[n]=0;
    char *end=coordinates;
    if(entry->position.ambiguity)*end++='~';
    end=aprs_coordinate(end,entry->position.latitude_e5);
    *end++=' ';aprs_coordinate(end,entry->position.longitude_e5);
   } else if(i==0)copy(identity,"NO POSITION REPORTS");
   const unsigned y=168+32*i;
   const bool identity_changed=changed(4+2*i,identity,force);
   const bool coordinates_changed=changed(5+2*i,coordinates,force);
   if(identity_changed || coordinates_changed) {
    panel(y,32);
    label(16,y+5,identity,entry?accent:muted);
    label(16,y+18,coordinates);
   }
  }
 }
 cache.valid=true;cache.page=a.page();
}
}

