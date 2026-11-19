// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui_home.hpp"
#include "ui_text.hpp"
#include "ui_icon_data.hpp"
#include "gps_view.hpp"
#include "board.h"
namespace rt950 {
namespace {
using namespace screen;
struct Palette {uint16_t bg,panel,line,ink,dim,meter,track,tx;};
Palette palette(bool light) {
 return light?Palette{color(0xffffff),color(0xe9edf2),color(0xcbd5e1),color(0x0f172a),
  color(0x475569),color(0x334155),color(0xe2e8f0),color(0xe11d48)}:
  Palette{color(0x0c1015),color(0x161c24),color(0x28323e),color(0xf8fafc),
  color(0xb4c2d4),color(0xe2e8f0),color(0x1a2330),color(0xef4444)};
}
struct Line {
 char data[64];
 Line(){data[0]=0;}
 unsigned size=0;
 void add(const char *s){while(*s && size<sizeof data-1)data[size++]=*s++;data[size]=0;}
 void number(unsigned n) {char d[10];unsigned count=0;do{d[count++]=char('0'+n%10);n/=10;}while(n);while(count){char c[]={d[--count],0};add(c);}}
};
void frequency(char *out,uint32_t hz) {
 unsigned div=100000000;
 for(unsigned i=0;i<9;++i) {
  if(i==3){out[i]='.';continue;}
  out[i]=char('0'+hz/div%10);div/=10;
 }
 out[9]=0;
}
void tone(Line &out,Tone t) {
 if(t.kind==ToneKind::Off){out.add("Off");return;}
 if(t.kind==ToneKind::Dcs) {
  const unsigned code=dcs_code(t);char code_text[]={'D',char('0'+(code>>6&7)),char('0'+(code>>3&7)),char('0'+(code&7)),dcs_inverted(t)?'I':'N',0};
  out.add(code_text);return;
 }
 const unsigned value=ctcss_frequency(t);
 out.number(value/10);out.add(".");out.number(value%10);
}
void put(int x,int y,const char *s,const Font &font,uint16_t fg,uint16_t bg,unsigned limit=240) {
 screen::text(x,y,s,font,fg,bg,limit);
}
void right(int x,int y,const char *s,const Font &font,uint16_t fg,uint16_t bg) {
 put(x-int(text_width(s,font)),y,s,font,fg,bg);
}
void badge(int x,int y,const char *s,uint16_t fg,uint16_t bg,uint16_t edge) {
 const unsigned w=text_width(s,Sans11)+8;
 fill(x,y,w,15,bg);border(x,y,w,15,edge);put(x+4,y-1,s,Sans11,fg,bg);
}
uint32_t hash(const char *s) {uint32_t h=2166136261u;while(*s){h^=uint8_t(*s++);h*=16777619u;}return h;}
uint32_t keys[3]{},header_key=0,footer_key=0;
unsigned bars[3]={255,255,255},active=255,ratio=0;
uint16_t heights[3]{},from[3]{};
uint32_t transition_start=0;
bool transitioning=false;
uint32_t tile_keys[9]{};
Page previous=Page::Settings;
bool valid=false,last_theme=false,last_large=false;Language last_language=Language::English;
void status_header(const Application &a,const InputStatus &input,uint32_t now,const Palette &p,bool force) {
 const unsigned mv=board_battery_mv();
 const unsigned charge=mv?board_battery_percent_from_mv(mv):0;
 const unsigned gps=a.low_power() || !a.config().gps_on?0:nav_fix_fresh(now)?3:(!board_gps_powered() || board_gps_sleeping())?1:2;
 const bool ble=!a.low_power() && a.config().ble_on;
 const bool mail=input.aprs_messages.personal_unread();
 const uint32_t key=gps|(ble?4u:0u)|(mail?8u:0u)|
    (a.show_voltage()?16u:0u)|(a.keypad_locked()?32u:0u)|(charge/25u<<6)|(a.show_voltage()?mv/100u<<10:0);
 if(!force && header_key==key)return;
 header_key=key;fill(0,0,240,20,p.bg);fill(0,19,240,1,p.line);
 const bool quick=a.page()==Page::QuickSettings;
 if(quick){char edit[]={'E','D','I','T',' ',char('A'+a.config().active),0};put(8,3,edit,Sans11,p.ink,p.bg);}
 const int gx=quick?57:8,bx=quick?75:28,mx=quick?93:48;
 const uint16_t gps_color=gps==3?p.ink:gps==2?color(a.light_theme()?0x926000:0xfbbf24):gps==1?p.dim:p.track;
 put(gx,4,"g",Icons12,gps_color,p.bg);
 put(bx,4,"b",Icons12,ble?p.ink:p.track,p.bg);
 if(mail){put(mx,4,"m",Icons12,p.ink,p.bg);fill(mx+9,3,4,4,color(0xfbbf24));}
 if(a.keypad_locked()){border(72,8,7,6,p.ink);border(73,4,5,6,p.ink);}
 if(a.show_voltage()){
  Line value;value.number(mv/1000);value.add(".");value.number(mv/100u%10u);value.add("V");
  right(202,3,value.data,Sans11,p.dim,p.bg);
 }
 border(210,5,20,10,p.dim);fill(231,7,2,6,p.dim);
 for(unsigned i=0;i<4;++i)if(charge>i*25u)fill(212+int(i)*4,7,3,6,p.ink);
}
void meter(unsigned slot,unsigned y,unsigned height,const Application &a,const InputStatus &input,uint32_t now,
 const Palette &p,bool force,bool quick) {
 const bool selected=a.config().active==slot;
 const auto bg=selected?p.panel:p.bg;
 const unsigned amount=input.voice_tx_slot==slot?0:input.slot_signal[slot].bars(a.config().vfo[slot].rx_hz,now);
 if(!force && bars[slot]==amount)return;
 bars[slot]=amount;
 const unsigned line_y=quick?y+height-6:y+height-8;
 fill(10,int(line_y),220,4,p.track);
 fill(10,int(line_y),220u*amount/8u,4,p.meter);
 (void)bg;
}
void slot_identity(unsigned slot,unsigned y,unsigned height,const Application &a,const InputStatus &input,
 const Palette &p,bool quick) {
 const auto &v=a.config().vfo[slot];const auto &profile=a.radio_profile(slot);
 const bool selected=a.config().active==slot;
 const auto bg=selected?p.panel:p.bg;
 const char letter[]={char('A'+slot),0};
 Line slot_label;if(quick && selected){slot_label.add("[");slot_label.add(letter);slot_label.add("]");}else slot_label.add(letter);
 badge(10,int(y)+(quick && !selected?12:5),slot_label.data,selected?p.bg:p.ink,selected?p.ink:p.panel,selected?p.ink:p.dim);
 const auto *memory=v.channel_mode?a.memory(v.memory_channel):nullptr;
 Line name;
 name.add(memory?memory->name:v.rx_hz>=400000000u?"UHF":"VHF");
 if(v.channel_mode){name.add(" CH ");if(v.memory_channel<9)name.add("0");name.number(v.memory_channel+1);}else name.add(" VFO");
 const bool sending=input.voice_tx_slot==slot;
 const bool receiving=input.voice.receiving && input.voice.slot==slot && input.voice.frequency==v.rx_hz;
 const char *state=sending?"TX":!v.tx_hz || profile.tx_inhibit?"RX ONLY":receiving?"RX":"STBY";
 const char *badge_text=quick && selected && v.tx_hz && !sending?"TX":state;
 const unsigned badge_width=text_width(badge_text,Sans11)+8;
 const unsigned badge_x=230u-badge_width;
  const bool light=a.light_theme(),only_rx=!v.tx_hz || profile.tx_inhibit;
 const bool ready=(quick && selected && !only_rx) || receiving;
 const auto status_bg=sending?p.tx:only_rx?bg:ready?p.ink:light?color(0xe2e8f0):color(0x334155);
 const auto status_fg=sending?color(0xffffff):only_rx?(light?color(0x475569):color(0xcbd5e1)):ready?p.bg:light?color(0x475569):color(0xcbd5e1);
 badge(int(badge_x),int(y)+(quick && !selected?13:5),badge_text,status_fg,status_bg,only_rx?color(0x64748b):status_bg);
 char hz[12];frequency(hz,sending?v.tx_hz:v.rx_hz);
 const bool large=selected && (quick || height>=90);
 const auto &frequency_font=quick?Mono12:Mono15;
 const unsigned name_x=quick && selected?41:32;
 const unsigned name_end=large?badge_x-5:badge_x-10-text_width(hz,frequency_font);
 const auto &preferred=a.large_font()?Sans13:Sans11;
 put(name_x,int(y)+(quick && !selected?11:3),name.data,fit(name.data,preferred,name_end-name_x),p.ink,bg,name_end);
 if(large) {
  put(10,int(y)+(quick?23:height<135?20:27),hz,quick?Mono24:Mono25,sending?p.tx:p.ink,bg,212);
  right(229,int(y)+(quick?36:height<135?31:41),"MHz",Sans11,p.dim,bg);
 } else right(int(badge_x)-6,int(y)+(quick && !selected?10:4),hz,frequency_font,p.ink,bg);
}
uint32_t card_key(unsigned i,const Application &a,const InputStatus &input,uint32_t now) {
 const auto &v=a.config().vfo[i];const auto &p=a.radio_profile(i);
 uint32_t h=v.rx_hz*31u+v.tx_hz;
 h=h*31u+unsigned(p.tx_inhibit);
 h=h*31u+unsigned(input.voice_tx_slot==i);
 h=h*31u+unsigned(input.voice.receiving && input.voice.slot==i && input.voice.frequency==v.rx_hz);
 if(v.channel_mode){h=h*31u+hash(a.memory(v.memory_channel)->name);h=h*31u+v.memory_channel+1;}
 if(a.page()==Page::QuickSettings)return h;
 h=h*31u+hash(input.dtmf[i].heard(v.rx_hz,now));
 h=h*31u+p.offset_hz;
 h=h*31u+unsigned(p.shift);
 h=h*31u+unsigned(p.reverse);
 h=h*31u+unsigned(p.narrow);
 h=h*31u+unsigned(p.busy_lock);
 h^=p.timeout_seconds<<8|p.squelch<<24|unsigned(p.power)<<28;
 h^=unsigned(p.rx_tone.kind)<<4|p.rx_tone.index<<6|unsigned(p.tx_tone.kind)<<14|p.tx_tone.index<<16;
 h^=Application::vfo_steps[a.applied_vfo_step()];
 return h;
}
void details(unsigned i,unsigned y,unsigned height,const Application &a,const InputStatus &input,uint32_t now,const Palette &p) {
 const auto &v=a.config().vfo[i];const auto &profile=a.radio_profile(i);
 const bool reversed=a.reverse_active(i);
 const char *heard=input.dtmf[i].heard(v.rx_hz,now);
 Line tones;
 if(height<135 && *heard){tones.add("DTMF ");tones.add(heard);}
 else if(profile.rx_tone.kind!=ToneKind::Off || profile.tx_tone.kind!=ToneKind::Off) {
  tones.add("R ");tone(tones,profile.heard_tone(reversed));tones.add(" / T ");tone(tones,profile.sent_tone(reversed));
 }
 Line offset;
 if(!v.tx_hz || v.tx_hz==v.rx_hz)offset.add("SIMPLEX");
 else {const bool minus=v.tx_hz<v.rx_hz;const unsigned delta=minus?v.rx_hz-v.tx_hz:v.tx_hz-v.rx_hz;
  offset.add(minus?"-":"+");offset.number(delta/1000000);offset.add(".");
  const unsigned khz=delta/1000%1000;char fraction[]={char('0'+khz/100),char('0'+khz/10%10),char('0'+khz%10),0};offset.add(fraction);}
 if(profile.reverse)offset.add(" REV");
 const auto &font=a.large_font()?Sans13:Sans11;
 const unsigned tone_end=224-text_width(offset.data,font);
 put(10,int(y)+int(height)-(height>=135?68:45),tones.data,fit(tones.data,font,tone_end-10),p.dim,p.panel,tone_end);
 right(230,int(y)+int(height)-(height>=135?68:45),offset.data,font,p.dim,p.panel);
 Line setup;
 setup.add(profile.narrow?"NFM ":"WFM ");
 constexpr const char *powers[]={"LOW","MID","HIGH","10mW","0.5W"};
 setup.add(powers[unsigned(profile.power)]);setup.add(" SQL");setup.number(profile.squelch);
 setup.add(profile.busy_lock?" BCL+":" BCL-");setup.add(" TOT");
 if(profile.timeout_seconds)setup.number(profile.timeout_seconds);else setup.add("-");
 Line step;step.add(a.language()==Language::Russian?"ШАГ ":"STEP ");
 const unsigned hz=Application::vfo_steps[a.applied_vfo_step()];step.number(hz/1000);
 if(hz%1000){step.add(".");step.number(hz%1000/10);}
 step.add("k");
 const unsigned setup_end=224-text_width(step.data,font);
 put(10,int(y)+int(height)-(height>=135?51:28),setup.data,fit(setup.data,font,setup_end-10),p.dim,p.panel,setup_end);
 right(230,int(y)+int(height)-(height>=135?51:28),step.data,font,p.dim,p.panel);
 if(height>=135) {
  Line tx;char hz_text[12];frequency(hz_text,v.tx_hz);
  tx.add(v.tx_hz?"TX ":"TX DISABLED");if(v.tx_hz){tx.add(hz_text);tx.add(" MHz");}
  if(*heard){tx=Line{};tx.add("DTMF ");tx.add(heard);}
  put(10,int(y)+int(height)-30,tx.data,font,p.dim,p.panel);
  right(230,int(y)+int(height)-30,v.tx_hz?powers[unsigned(profile.power)]:"OFF",font,p.dim,p.panel);
 }
}
void quick_matrix(unsigned y,const Application &a,const Palette &p,bool force) {
 const bool ru=a.language()==Language::Russian;
 constexpr const char *names[]={"Mode","RX tone","TX tone","Offset","Power","Step","Squelch","Busy lock","Reverse"};
 if(force)fill(0,int(y)+60,240,140,p.bg);
 for(unsigned i=0;i<9;++i) {
  constexpr unsigned xs[]={8,85,161},ys[]={66,105,145},ws[]={71,70,71},hs[]={33,34,33};
  const unsigned x=xs[i%3],top=y+ys[i/3],width=ws[i%3],height=hs[i/3];
  const bool chosen=a.quick_tile()==i;
  char value[32];a.quick_value(i,value);
  Line label;
  if(i==2 && a.radio_profile(a.config().active).tx_tone.kind!=ToneKind::Off)label.add("T ");
  label.add(value);
  const uint32_t key=hash(label.data)<<1|unsigned(chosen);
  if(!force && tile_keys[i]==key)continue;
  const bool redraw_tile=force || ((tile_keys[i]^key)&1u);
  tile_keys[i]=key;
  const auto bg=chosen?(a.light_theme()?color(0x0f172a):p.meter):p.panel,fg=chosen?p.bg:p.ink;
  if(redraw_tile){
   fill(int(x),int(top),width,height,bg);border(int(x),int(top),width,height,chosen?p.ink:p.line);
   put(int(x)+4,int(top)+2,names[i],fit(names[i],Sans11,chosen?52:62),chosen?p.bg:p.dim,bg,x+(chosen?56:66));
   if(chosen)put(int(x)+57,int(top)+2,"↺",Sans11,fg,bg);
  }
  fill(int(x)+1,int(top)+16,width-2,height-17,bg);
  put(int(x)+4,int(top)+14,label.data,a.large_font()?fit(label.data,Sans13,62):Mono11,fg,bg,x+67);
 }
 if(!force)return;
 fill(0,int(y)+182,240,1,p.line);
 put(10,int(y)+185,ru?"Удерж. 9: сохранить канал":"Hold 9: save channel",Sans11,p.dim,p.bg,217);
 put(220,int(y)+186,"s",Icons10,p.dim,p.bg);
}
void footer(const Application &a,const InputStatus &input,const Palette &p,bool force) {
 const bool quick=a.page()==Page::QuickSettings;
 const bool ru=a.language()==Language::Russian;
 const bool channel=a.config().vfo[a.config().active].channel_mode;
 const char *notice=a.feedback();
#ifdef RT950_BENCH_TX_HZ
 if(input.bench_ptt && input.bench_ptt[0])notice=input.bench_ptt;
#endif
 Line signature;signature.number(unsigned(a.page()));signature.number(channel);signature.number(ru);
 if(notice)signature.add(notice);
 if(a.editing()){char entry[16];a.entry_text(entry);signature.add(entry);}
 if(!force && footer_key==hash(signature.data))return;
 footer_key=hash(signature.data);fill(0,300,240,20,p.bg);fill(0,300,240,1,p.line);
 if(notice && *notice){
  const char *message=translate(a.language(),notice);
  put(8,302,message,fit(message,a.large_font()?Sans13:Sans11,224),p.tx,p.bg,232);return;
 }
 if(a.editing()){char entry[16];a.entry_text(entry);put(8,304,entry,Mono12,p.ink,p.bg);right(232,304,"OK",Sans11,p.ink,p.bg);return;}
 const char *labels[4];unsigned count;
 if(quick){labels[0]=ru?"◄▲▼► Выбор":"◄▲▼► Pick";labels[1]=ru?"Валкодер Изм.":"Knob Change";labels[2]=ru?"BACK Готово":"BACK Done";count=3;}
 else {labels[0]=ru?"OK Меню":"OK Menu";labels[1]=ru?"ABC Слот":"ABC Slot";labels[2]=channel?"V/M VFO":"V/M CH";
  labels[3]=channel?(ru?"◄►Канал":"◄►Chan"):(ru?"◄►Шаг":"◄►Step");count=4;}
 const Font *font=a.large_font()?&Sans13:&Sans11;
 unsigned total=0;
 for(;;){
  total=0;for(unsigned i=0;i<count;++i)total+=text_width(labels[i],*font);
  if(total+5*(count-1)<=224 || font==&Sans10)break;
  font=font==&Sans13?&Sans11:&Sans10;
 }
 unsigned used=0;
 for(unsigned i=0;i<count;++i){const unsigned gap=total<224?(224-total)*i/(count-1):0;
  put(int(8+used+gap),302,labels[i],*font,p.dim,p.bg,232);used+=text_width(labels[i],*font);}


}
}
void invalidate_home(){valid=false;transitioning=false;}
bool ui_transitioning(){return transitioning;}
void draw_home(Application &a,const InputStatus &input,uint32_t now,bool requested) {
 const auto p=palette(a.light_theme());
 const bool force=requested || !valid || previous!=a.page() ||
     ratio!=a.slot_ratio() || last_theme!=a.light_theme() || last_language!=a.language() || last_large!=a.large_font();
 const bool switched=active!=a.config().active;
 const bool quick=a.page()==Page::QuickSettings;
 status_header(a,input,now,p,force || (quick && switched));
 if(force || quick || a.page()!=Page::Radio || input.ptt_mask)transitioning=false;
 else if(switched) {
  for(unsigned i=0;i<3;++i)from[i]=heights[i];
  transition_start=now;
  transitioning=true;
 }
 if(a.page()==Page::DisplayChoice) {
  const bool ru=a.language()==Language::Russian;
  const char *title=a.display_setting()==Key::Theme?(ru?"Тема экрана":"Screen theme"):
      a.display_setting()==Key::SlotRatio?(ru?"Пропорции слотов":"Slot proportions"):
      a.display_setting()==Key::FontScale?translate(a.language(),"Font size"):(ru?"Напряжение батареи":"Battery voltage");
  Line value;
  if(a.display_setting()==Key::Theme)value.add(a.light_theme()?(ru?"Светлая":"Light"):(ru?"Тёмная":"Dark"));
  else if(a.display_setting()==Key::ShowVoltage)value.add(translate(a.language(),a.show_voltage()?"ON":"OFF"));
  else if(a.display_setting()==Key::FontScale)value.add(translate(a.language(),a.large_font()?"Large":"Standard"));
  else if(a.slot_ratio()==66)value.add("33.3%");
  else {value.number(a.slot_ratio()/2u);value.add(a.slot_ratio()%2u?".5%":"%");}
  const uint32_t choice_key=hash(value.data)^unsigned(a.display_setting());
  if(force || keys[0]!=choice_key) {
   keys[0]=choice_key;
   fill(0,20,240,280,p.bg);
   put(10,40,title,Sans13,p.ink,p.bg);
   put(10,80,value.data,a.large_font()?Sans13:Sans11,p.ink,p.bg);
   put(10,112,ru?"◄► Изменить / OK Готово":"◄► Change / OK Done",Sans11,p.dim,p.bg);
  }
 } else {
  unsigned y=20,old_y=20,weight=4;
  if(transitioning) {
   const unsigned elapsed=now-transition_start;
   weight=elapsed<40?1:elapsed<80?2:elapsed<120?3:4;
   transitioning=weight<4;
  }
  for(unsigned i=0;i<3;++i) {
   const bool chosen=i==a.config().active;
   const unsigned target=quick?(chosen?200u:40u):a.slot_height(i);
   // Interpolate cumulative boundaries to keep the total exactly 280 px.
   unsigned height=target;
   if(weight<4) {
    const unsigned target_bottom=20u+a.slot_height(0)+(i>=1?a.slot_height(1):0u)+(i==2?a.slot_height(2):0u);
    const unsigned from_bottom=20u+from[0]+(i>=1?from[1]:0u)+(i==2?from[2]:0u);
    height=(target_bottom*weight+from_bottom*(4-weight)+2)/4-y;
   }
   const unsigned old_height=heights[i];
   const bool layout=force || y!=old_y || height!=old_height || (switched && (chosen || i==active));
   const uint32_t key=card_key(i,a,input,now);
   const bool update=layout || key!=keys[i];
   if(update) {
    keys[i]=key;
    // The selected quick slot has independent header and matrix regions.
    const unsigned painted=quick && chosen?60:height;
    fill(0,int(y),240,painted,chosen?p.panel:p.bg);
    if(!quick || !chosen)fill(0,int(y+height-1),240,1,p.line);
    slot_identity(i,y,height,a,input,p,quick);
    if(!quick && chosen && height>=90)details(i,y,height,a,input,now,p);
   }
   if(quick && chosen)quick_matrix(y,a,p,layout);
   if(!quick || chosen)meter(i,y,quick?60:height,a,input,now,p,update,quick);
   heights[i]=uint16_t(height);
   old_y+=old_height;
   y+=height;
  }
 }
 footer(a,input,p,force);
 valid=true;active=a.config().active;ratio=a.slot_ratio();previous=a.page();last_theme=a.light_theme();last_language=a.language();last_large=a.large_font();
}
}
