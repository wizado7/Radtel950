#include "settings_export.hpp"
#include "settings_import.hpp"
#include "voice_tx_board.hpp"
#include "voice_tx_registers.hpp"
#include "voice_uhf_tx.hpp"
#include "voice_calibration.hpp"
#include "voice_tx_route.hpp"
#include "voice_tx_values.hpp"
#include "voice_ptt.hpp"
#include "aprs_rx.hpp"
#include "aprs_tx.hpp"
#include "aprs_afsk.hpp"
#include "aprs_dac.hpp"
#include "aprs_beacon.hpp"
#include "aprs_stations.hpp"
#include "aprs_messages.hpp"
#include "aprs_ble.hpp"
#include "aprs_background.hpp"
#include "voice_battery_limits.hpp"
#include "aprs_adc.hpp"
#include "mcu_diagnostics.hpp"
#include "drivers/dma.h"
#include "voice_watch.hpp"
#include "channel_scan.hpp"
#include "voice_bus.hpp"
// SPDX-License-Identifier: GPL-3.0-or-later
#include "application.hpp"
#include "power_diagnostics.hpp"
#include "fm_receiver.hpp"
#include "fm_i2c.hpp"
#include "fm_probe.hpp"
#include "storage_journal.hpp"
#include "controls.hpp"
#include "gps_view.hpp"
#include "protocols/oem_crc.hpp"
using namespace rt950;
extern "C" {uint8_t regression_fixture[8192];uint32_t backup_fixture_size=0;}
#define CHECK(x) do {if(!(x))return __LINE__;} while(0)

extern "C" int oem_channel_import_probe(const uint8_t *payload,unsigned size,unsigned schema) {
 Application app(false);
 CHECK(app.restore_portable_snapshot(payload,size,schema));
 const auto &channel=*app.memory(63);
 CHECK(channel.occupied && channel.zone==9 && channel.rx_hz==446081250 && channel.tx_hz==446006250);
 CHECK(channel.radio.tx_inhibit && channel.radio.narrow && channel.radio.busy_lock);
 CHECK(channel.radio.power==TxPower::Low && channel.radio.rx_tone.kind==ToneKind::Ctcss &&
       ctcss_frequency(channel.radio.rx_tone)==885);
 CHECK(channel.radio.tx_tone.kind==ToneKind::Dcs && dcs_inverted(channel.radio.tx_tone));
 CHECK(channel.name[0]=='T' && channel.name[4]=='-' && channel.name[5]=='O');
 CHECK(app.compact_snapshot_size());
 app.key(Key::SelectZone);for(unsigned i=0;i<9;++i)app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.recall(63) && !app.scan_eligible(63) && !app.config().vfo[0].tx_hz);
 return 0;
}
static void enter(Application &a,const char *s) {while(*s)a.digit(unsigned(*s++-'0'));dispatch_key(a,6);}
static void nmea(const char *body,uint32_t now) {
 nav_feed('$',now);uint8_t checksum=0;
 for(const char *s=body;*s;++s){checksum^=uint8_t(*s);nav_feed(uint8_t(*s),now);}
 const char *hex="0123456789ABCDEF";nav_feed('*',now);nav_feed(uint8_t(hex[checksum>>4]),now);nav_feed(uint8_t(hex[checksum&15]),now);nav_feed('\n',now);
}
extern "C" uint16_t oem_crc_probe(const uint8_t *d,unsigned n){return oem_control_crc(d,n);}
namespace {
struct AprsCapture {
 unsigned count=0;
 size_t length=0,offset=0;
 uint8_t kiss[96]{};
};
static void capture_aprs(void *ctx,const uint8_t *frame,size_t size) {
 auto &result=*static_cast<AprsCapture*>(ctx);
 if(!rt950::aprs::ui_payload(frame,size,result.offset))return;
 ++result.count;
 result.length=rt_kiss_encode(0,frame,size,result.kiss,sizeof result.kiss);
}
struct FactorySpi {
 unsigned count=0,fail_after=255;
 bool selected=false;
 uint8_t bytes[20]{};
 void select(bool active){selected=active;}
 bool transfer(uint8_t out,uint8_t &in) {
  if(!selected || count>=fail_after || count>=sizeof bytes)return false;
  bytes[count++]=out;in=0x71;return true;
 }
};
struct TxBackend {
 unsigned begins=0,keys=0,stops=0,tails=0;
 bool tail_ok=true;
 bool healthy_flag=true,clear=true,key_ok=true,begin_ok=true;
 rt950::voice::Ptt::Preparation readiness=rt950::voice::Ptt::Preparation::Ready;
 bool begin(const rt950::voice::TxRequest &){++begins;return begin_ok;}
 bool key(){++keys;return key_ok;}
 bool start_tail(){++tails;return healthy_flag && tail_ok;}
 void stop(){++stops;}
 bool channel_clear() const {return clear;}
 bool healthy() const {return healthy_flag;}
 auto poll_ready(uint32_t) const {return readiness;}
};
struct TxBoard {
 using Channel=rt950::voice::UhfTransmitter<TxBoard>::Channel;
 rt950::voice::UhfCalibration cal{};
 Channel observation=Channel::Pending;
 bool acquired=false,configure_ok=true,healthy_flag=true;
 uint32_t tuned=0;
 uint8_t bias_value=0;
 unsigned route_writes=0,nonzero_bias=0,restores=0,register_writes=0,delay_total=0;
 bool bad_restore=false;
 bool acquire(){acquired=true;return true;}
 void bias(uint8_t value){bias_value=value;if(value)++nonzero_bias;}
 bool listen(uint32_t hz){tuned=hz;return true;}
 Channel channel(uint32_t) const {return observation;}
 bool read(uint8_t reg,uint16_t &out){
  if(reg==0x7d)out=0xe912;
  else if(reg==0x52)out=0x6fff;
  else if(reg==0x31)out=0x33;
  else if(reg==0x40)out=0xa800;
  else return false;
  return true;
 }
 bool write(uint8_t reg,uint16_t) {
  if(!configure_ok || (reg!=0x36 && bias_value) || (reg==0x36 && !bias_value))return false;
  ++register_writes;return true;
 }
 void delay_us(unsigned duration){delay_total+=duration;}
 bool tx_values(uint16_t mic,rt950::voice::UhfTxValues &out) const {
  return rt950::voice::uhf_tx_values(cal,tuned,power_selector(),mic,out);
 }
 uint8_t power_selector() const {return 2;}
 bool healthy() const {return healthy_flag;}
 void mute_receiver(){}
 void set(rt950::voice::TxRoutePin,bool){++route_writes;}
 bool configure_voice(const rt950::voice::UhfTxValues &values) {
  return rt950::voice::prepare_voice_tx(*this,values,rt950::voice::uhf_tx_chip);
 }
 bool finish_voice(){return rt950::voice::finish_voice_tx(*this,power_selector());}
 bool start_tail(){return healthy_flag;}
 void restore_rx(){bad_restore|=bias_value!=0;acquired=false;++restores;}
};
struct VoicePins {
 uint32_t sent=0;
 uint16_t reply=0xA153;
 unsigned writes=0,reads=0;
 int selected=-1;
 bool listening=false,data=false,bad_sample=false;
 void select(uint8_t chip,bool active){selected=active?int(chip):-1;}
 void drive(bool value){listening=false;data=value;}
 void input(){listening=true;}
 void clock(bool high){if(high && selected>=0 && !listening){sent=(sent<<1)|unsigned(data);++writes;}}
 bool sample(){bad_sample|=!listening || selected<0;return (reply>>(15-reads++))&1u;}
 void delay(){}
};
struct VoiceReceiver {
 uint32_t frequency=0,settled=0;
 bool carrier=false;
 void tune(uint32_t hz,uint32_t now){frequency=hz;settled=now+25;}
 void stop(){frequency=0;}
 bool squelch_open(uint32_t now) const {return frequency && carrier && int32_t(now-settled)>=0;}
};
struct MockFm {
 unsigned operations=0;
 uint8_t command=0,seek_arg=0,status_arg=0,flags=0;
 uint16_t frequency=10030;
 bool seek_stalled=false;
 bool stalled=false,error=false,wire_error=false,wrong_frequency=false;
 bool write(uint8_t address,const uint8_t *data,size_t size) {
  ++operations;command=data[0];
  if(wire_error || address!=0x11)return false;
  if(command==0x20)return size==5 && data[1]==3 && data[2]==0x27 && data[3]==0x2e && data[4]==0;
  if(command==0x21){seek_arg=data[1];return size==2 && seek_arg==0x0c;}
  if(command==0x14)return size==1;
  if(command==0x22 && size==2){status_arg=data[1];return status_arg==1 || status_arg==3;}
  return size==2 && command==0x23 && data[1]==1;
 }
 bool read(uint8_t address,uint8_t *data,size_t size) {
  ++operations;
  if(wire_error || address!=0x11)return false;
  data[0]=error?0xc0:stalled?0:command==0x14 && !seek_stalled?0x81:0x80;
  if(size==7 && command==0x22){data[1]=flags;data[2]=uint8_t(frequency>>8);data[3]=wrong_frequency?0:uint8_t(frequency);}
  if(size==8 && command==0x23){data[2]=1;data[3]=0x80;data[4]=48;data[5]=19;}
  return true;
 }
};
struct MockProbeBus:MockFm {
 unsigned properties=0;
 bool powered=false,revision=false,no_initial_reads=false;
 bool write(uint8_t address,const uint8_t *data,size_t size) {
  if(data[0]==1){powered=!wire_error && address==0x11 && size==3 && data[1]==0x10 && data[2]==5;return powered;}
  if(data[0]==0x10){revision=powered && size==1;return revision;}
  if(data[0]==0x12){
   constexpr uint8_t expected[][6]={{0x12,0,0,1,0,1},{0x12,0,0x14,0,0x19,0x64},
     {0x12,0,0x14,1,0x2a,0x30},{0x12,0,0x14,2,0,10},{0x12,0,0x14,3,0,10},{0x12,0,0x14,4,0,25}};
   if(!revision || address!=0x11 || size!=6 || properties>=6)return false;
   for(size_t i=0;i<size;++i)if(data[i]!=expected[properties][i])return false;
   ++properties;return true;
  }
  if(data[0]==0x20 && properties!=6)return false;
  return MockFm::write(address,data,size);
 }
 bool read(uint8_t address,uint8_t *data,size_t size) {
  if(no_initial_reads && !powered)return false;
  if(size==9){data[0]=0x80;data[1]=32;return revision;}
  return MockFm::read(address,data,size);
 }
};
struct MockAm {
 uint8_t command=0;
 unsigned properties=0,seek_commands=0;
 uint16_t low=2300,high=30000,frequency=7100;
 uint8_t tune_flags=0,status_arg=0,seek_arg=0;
 bool seek_stalled=false;
 uint8_t spacing=5;
 bool write(uint8_t address,const uint8_t *data,size_t size) {
  if(address!=0x11 || !size)return false;
  command=data[0];
  if(command==1)return size==3 && data[1]==0x11 && data[2]==5;
  if(command==0x10 || command==0x14)return size==1;
  if(command==0x12) {
   constexpr uint16_t ids[]={1,0x3202,0x3103,0x3400,0x3401,0x3402};
   const uint16_t vals[]={1,0,0x7800,low,high,spacing};
   if(size!=6 || data[1] || properties>=6)return false;
   const unsigned index=properties++;
   return (uint16_t(data[2])<<8|data[3])==ids[index] &&
          (uint16_t(data[4])<<8|data[5])==vals[index];
  }
  if(command==0x40)return properties==6 && size==6 && data[1]==0x40 &&
      (uint16_t(data[2])<<8|data[3])==frequency && !data[4] && data[5]==1;
  if(command==0x41) {
   ++seek_commands;seek_arg=data[1];
   return size==(low==2300?6u:2u) && (seek_arg==12 || seek_arg==4) &&
       (size==2 || (!data[2] && !data[3] && !data[4] && data[5]==1));
  }
  if(command==0x42 && size==2){status_arg=data[1];return status_arg==1 || status_arg==3;}
  return size==2 && command==0x43 && data[1]==1;
 }
 bool read(uint8_t address,uint8_t *data,size_t size) {
  if(address!=0x11)return false;
  data[0]=command==0x14 && !seek_stalled?0x81:0x80;
  if(size==1)return true;
  if(size==9 && command==0x10){data[1]=32;return true;}
  if(size==8 && command==0x42){data[1]=tune_flags;data[2]=uint8_t(frequency>>8);data[3]=uint8_t(frequency);return true;}
  if(size==6 && command==0x43){data[2]=1;data[4]=42;data[5]=18;return true;}
  return false;
 }
};
struct MockLines {
 unsigned delays=0,reads=0;
 bool clock_high=true,released=true,blocked=false,sampled_driven=false;
 uint32_t samples=0;
 void clock(bool high){clock_high=high;}
 void drive_data(bool){released=false;}
 void release_data(){released=true;}
 void delay(){++delays;}
 bool clock_level() const{return clock_high && !blocked;}
 bool data_level(){sampled_driven|=!released;return (samples>>reads++)&1u;}
};
struct MockSlots {
 uint8_t slots[2][32]{};
 int remaining=-1;
 unsigned erases=0;
 size_t capacity() const {return sizeof slots[0];}
 const uint8_t *view(unsigned slot) const {return slots[slot];}
 bool tick(){if(remaining<0)return true;if(!remaining)return false;--remaining;return true;}
 bool erase(unsigned slot) {
  ++erases;
  for(auto &byte:slots[slot]) {if(!tick())return false;byte=0xff;}
  return true;
 }
 bool program(unsigned slot,size_t offset,const uint8_t *data,size_t size) {
  if(offset>capacity() || size>capacity()-offset)return false;
  for(size_t i=0;i<size;++i) {
   if(!tick())return false;
   auto &byte=slots[slot][offset+i];
   if((byte & data[i])!=data[i])return false;
   byte=data[i];
  }
  return true;
 }
};
}
extern "C" uint32_t voice_sql_status_probe(uint32_t status) {
 return voice::hardware_sql_open(uint16_t(status));
}

extern "C" uint32_t voice_sql_probe(uint32_t hz,uint32_t level) {
 voice::SqlCalibration sql;
 for(unsigned i=1;i<10;++i){sql.open[i]=uint8_t(80+2*i);sql.close[i]=uint8_t(73+2*i);}
 for(unsigned i=0;i<13;++i)sql.uhf[i]=uint8_t(3+i);
 for(unsigned i=0;i<10;++i)sql.vhf[i]=uint8_t(2+i);
 if(!sql.valid())return 0;
 const auto threshold=sql.thresholds(hz,uint8_t(level));
 return unsigned(threshold.open)*256u+threshold.close;
}

extern "C" uint32_t battery_tx_limit_probe(uint32_t raw,uint32_t factory) {
 voice::BatteryTxLimits limits;
 const uint8_t stored[]={127,133,145,154,162,170,255};
 if(factory && !limits.load(stored))return 2;
 return limits.permits(uint8_t(raw));
}

extern "C" int regression_tests() {
 {
  Application scan_app;
  voice::ChannelScan scan;
  scan_app.key(Key::Channels);
  CHECK(scan_app.scan_eligible(0));
  scan_app.key(Key::Right);
  CHECK(!scan_app.scan_eligible(0) && scan_app.channel_visible(0));
  scan_app.key(Key::ChannelScan);
  CHECK(scan.poll(scan_app,0,true,false,false)==1);
  CHECK(scan.poll(scan_app,149,true,true,false)==1);
  CHECK(scan.poll(scan_app,150,true,true,false)==2);
  CHECK(scan.poll(scan_app,180,true,true,true)==2);
  CHECK(scan.poll(scan_app,1179,true,true,false)==2);
  CHECK(scan.poll(scan_app,1180,true,true,false)==3);
  CHECK(scan.poll(scan_app,1181,true,true,true)==3);
  scan_app.key(Key::Down);
  CHECK(scan.poll(scan_app,1182,true,true,true)==2);
  scan_app.key(Key::Up);
  CHECK(scan.poll(scan_app,1183,true,false,false)==3);
  scan_app.scan_channel(3);
  scan_app.key(Key::Ok);
  CHECK(scan_app.scan_paused() && scan_app.page()==Page::ChannelScan);
  CHECK(scan.poll(scan_app,4999,true,false,false,true)==3 && scan_app.scan_paused());
  CHECK(scan.poll(scan_app,5000,true,true,false)==3);
  scan_app.key(Key::Ok);
  CHECK(scan_app.page()==Page::Radio && scan_app.config().vfo[0].memory_channel==3);
  CHECK(scan.poll(scan_app,5001,false,false,false)==voice::ChannelScan::None);
  Application empty(false);
  CHECK(scan.poll(empty,5002,true,false,false)==voice::ChannelScan::None);
  scan_app.key(Key::ScanMode);scan_app.key(Key::Up);scan_app.key(Key::Ok);
  CHECK(scan_app.scan_mode()==ScanMode::Time);
  scan_app.key(Key::ChannelScan);
  scan.poll(scan_app,6000,false,false,false);
  CHECK(scan.poll(scan_app,6001,true,false,false)==1);
  CHECK(scan.poll(scan_app,6100,true,true,true)==1);
  CHECK(scan.poll(scan_app,11099,true,true,true)==1);
  CHECK(scan.poll(scan_app,11100,true,true,true)==2);
  scan_app.key(Key::ScanMode);scan_app.key(Key::Up);scan_app.key(Key::Ok);
  CHECK(scan_app.scan_mode()==ScanMode::Search);
  scan_app.key(Key::ChannelScan);
  scan.poll(scan_app,12000,false,false,false);
  CHECK(scan.poll(scan_app,12001,true,false,false)==1);
  CHECK(scan.poll(scan_app,12100,true,true,true)==1 && scan_app.scan_paused());
  CHECK(scan.poll(scan_app,20000,true,true,false)==1);
  scan_app.key(Key::Right);
  CHECK(scan.poll(scan_app,20001,true,true,true)==2);
  scan_app.scan_channel(2);
  scan_app.key(Key::Left);
  CHECK(!scan_app.scan_eligible(2) && scan_app.channel_visible(2) && !scan_app.scan_frequency());
  CHECK(scan.poll(scan_app,20002,true,true,true)==3);
  scan_app.key(Key::Back);
  scan_app.key(Key::Channels);scan_app.digit(3);scan_app.key(Key::Ok);
  CHECK(scan_app.config().vfo[0].memory_channel==2);
  scan_app.key(Key::Channels);scan_app.key(Key::Right);
  CHECK(scan_app.scan_eligible(2));
  Application last(false);
  CHECK(last.set_frequency(446081250) && last.store(0));
  last.key(Key::ChannelScan);scan.poll(last,20003,false,false,false);
  CHECK(scan.poll(last,20004,true,false,false)==0);
  last.scan_channel(0);last.key(Key::Left);
  CHECK(scan.poll(last,20005,true,true,true)==voice::ChannelScan::None && !scan.frequency());
  Application vfo_scan;
  CHECK(vfo_scan.set_frequency(145500000));
  CHECK(vfo_scan.recall(0));
  CHECK(vfo_scan.config().vfo[0].rx_hz!=vfo_scan.scan_vfo_frequency());
  const uint32_t scan_step=Application::vfo_steps[vfo_scan.applied_vfo_step()];
  vfo_scan.key(Key::ScanStart);
  for(char digit:"145500")if(digit)vfo_scan.digit(unsigned(digit-'0'));
  vfo_scan.key(Key::Ok);
  vfo_scan.key(Key::ScanEnd);
  for(char digit:"433500")if(digit)vfo_scan.digit(unsigned(digit-'0'));
  vfo_scan.key(Key::Ok);
  CHECK(vfo_scan.page()==Page::ScanEnd && vfo_scan.scan_bound(true)==0);
  vfo_scan.key(Key::Back);vfo_scan.key(Key::ScanEnd);
  for(char digit:"145525")if(digit)vfo_scan.digit(unsigned(digit-'0'));
  vfo_scan.key(Key::Ok);
  CHECK(vfo_scan.scan_bound(false)==145500000 && vfo_scan.scan_bound(true)==145525000);
  vfo_scan.key(Key::VfoScan);
  scan.poll(vfo_scan,21000,false,false,false);
  scan.poll(vfo_scan,21001,true,false,false);
  CHECK(scan.frequency()==145500000);
  scan.poll(vfo_scan,21151,true,true,false);
  CHECK(scan.frequency()==145500000+scan_step);
  vfo_scan.key(Key::Down);
  scan.poll(vfo_scan,21301,true,true,false);
  CHECK(scan.frequency()==145500000);
  scan.poll(vfo_scan,21451,true,true,false);
  CHECK(scan.frequency()==145525000-(25000u%scan_step));
  CHECK(vfo_scan.config().vfo[0].channel_mode && vfo_scan.scan_vfo_frequency()==145500000);
  vfo_scan.scan_target(voice::ChannelScan::None,scan.frequency());
  vfo_scan.key(Key::Ok);CHECK(vfo_scan.scan_paused());
  vfo_scan.key(Key::Ok);CHECK(vfo_scan.page()==Page::Radio &&
     vfo_scan.config().vfo[0].rx_hz==scan.frequency());


 }

 rt950::aprs::BackgroundListen schedule(0xfffffff0u);
 CHECK(!schedule.poll(9983,true,true,10000));
 CHECK(!schedule.poll(9984,true,false,10000));
 CHECK(schedule.poll(9985,true,true,10000));
 CHECK(schedule.poll(11184,true,false,10000));
 CHECK(!schedule.poll(11185,true,true,10000));
 CHECK(!schedule.poll(21184,true,true,10000));
 CHECK(schedule.poll(21185,true,true,10000));
 CHECK(!schedule.poll(21186,false,true,10000));
 CHECK(!schedule.poll(31185,true,true,10000));
 CHECK(schedule.poll(31186,true,true,10000));
 CHECK(!schedule.poll(31187,true,true,0));
 CHECK(!schedule.poll(50000,true,true,0));
 CHECK(!schedule.poll(50001,true,true,5000));
 CHECK(schedule.poll(55001,true,true,5000));
 CHECK(schedule.windows()==4 && schedule.cancelled()==2);
 CHECK(!schedule.poll(56201,true,true,5000));
 CHECK(schedule.windows()==4 && schedule.cancelled()==2);
 using namespace rt950::storage;
 // Fixed AX.25 position vector with little-endian on-air FCS 0x5575.
 const uint8_t aprs_packet[]={0x82,0xa0,0xa4,0xa6,0x40,0x40,0x60,0x9c,0x60,0x86,0x82,0x98,
   0x98,0x61,3,0xf0,'!','5','5','4','5','.','0','0','N','/','0','3','7','3','7','.','0','0',
   'E','>',' ','T','E','S','T',0x75,0x55};
 rt950::aprs::FrameReceiver aprs_rx;
 AprsCapture capture;
 auto feed=[&](bool bit){aprs_rx.bit(bit,capture_aprs,&capture);};
 auto flag=[&](){for(unsigned bit=0;bit<8;++bit)feed((0x7eu>>bit)&1u);};
 auto packet=[&](bool corrupt) {
  unsigned ones=0;
  for(unsigned i=0;i<sizeof aprs_packet;++i) {
   const uint8_t value=uint8_t(aprs_packet[i]^(corrupt && i==20?1u:0u));
   for(unsigned bit=0;bit<8;++bit) {
    const bool high=(value>>bit)&1u;
    feed(high);ones=high?ones+1:0;
    if(ones==5){feed(false);ones=0;}
   }
  }
 };
 flag();packet(false);flag();
 CHECK(capture.count==1 && capture.offset==16 && capture.length==sizeof aprs_packet+1);
 CHECK(capture.kiss[0]==0xc0 && capture.kiss[1]==0 && capture.kiss[capture.length-1]==0xc0);
 for(unsigned i=0;i<sizeof aprs_packet-2;++i)CHECK(capture.kiss[i+2]==aprs_packet[i]);
 packet(true);flag();
 CHECK(capture.count==1);
 for(unsigned i=0;i<3000;++i)feed(false);
 flag();packet(false);flag();
 CHECK(capture.count==2);
 rt950::aprs::Position point;
 CHECK(rt950::aprs::position(aprs_packet,sizeof aprs_packet-2,point));
 CHECK(point.latitude_e5==5575000 && point.longitude_e5==3761667 &&
       point.callsign[0]=='N' && point.callsign[5]=='L' && point.ssid==0);
 CHECK(!rt950::aprs::position(aprs_packet,20,point));
 {
  uint8_t compressed[40]{};
  for(unsigned i=0;i<16;++i)compressed[i]=aprs_packet[i];
  const char payload[]="@092345z/NN!!<*e7>7P[";
  for(unsigned i=0;i<sizeof payload-1;++i)compressed[16+i]=uint8_t(payload[i]);
  constexpr size_t size=16+sizeof payload-1;
  CHECK(rt950::aprs::position(compressed,size,point));
  CHECK(point.latitude_e5==0 && point.longitude_e5==-7275000 && point.callsign[0]=='N');
  rt950::aprs::RecentStations compressed_stations;
  compressed_stations.record(compressed,size,2000);
  CHECK(compressed_stations.at(0) && compressed_stations.at(0)->received_at==2000);
  compressed[17]='4';
  CHECK(!rt950::aprs::position(compressed,size,point));
  compressed[17]='0';
  CHECK(!rt950::aprs::position(compressed,size-1,point));
  compressed[25]='~';
  CHECK(!rt950::aprs::position(compressed,size,point));
  compressed[25]='{';compressed[26]='{';
  CHECK(rt950::aprs::position(compressed,size,point) && point.latitude_e5==-9000000);
  compressed[28]='"';
  CHECK(!rt950::aprs::position(compressed,size,point));
  CHECK(point.latitude_e5==-9000000);
 }
 {
  uint8_t mic_e[25]{};
  for(unsigned i=0;i<16;++i)mic_e[i]=aprs_packet[i];
  const char destination[]="S32U6T";
  for(unsigned i=0;i<6;++i)mic_e[i]=uint8_t(destination[i]<<1);
  constexpr uint8_t payload[]={0x60,100,89,103,28,28,28,'>','/'};
  for(unsigned i=0;i<sizeof payload;++i)mic_e[16+i]=payload[i];
  CHECK(rt950::aprs::position(mic_e,sizeof mic_e,point));
  CHECK(point.latitude_e5==3342733 && point.longitude_e5==-7202917 && point.callsign[0]=='N');
  mic_e[16]=0x27;mic_e[3]='5'<<1;mic_e[4]='V'<<1;mic_e[5]='4'<<1;
  mic_e[17]=118;mic_e[18]=88;mic_e[19]=28;
  CHECK(rt950::aprs::position(mic_e,sizeof mic_e,point));
  CHECK(point.latitude_e5==-3342733 && point.longitude_e5==0);
  mic_e[17]=108;
  CHECK(rt950::aprs::position(mic_e,sizeof mic_e,point) && point.longitude_e5==10000000);
  CHECK(!rt950::aprs::position(mic_e,sizeof mic_e-1,point));
  const char ambiguous[]="T4SQZZ";
  for(unsigned i=0;i<6;++i)mic_e[i]=uint8_t(ambiguous[i]<<1);
  mic_e[17]='(';mic_e[18]='_';mic_e[19]='f';
  CHECK(rt950::aprs::position(mic_e,sizeof mic_e,point));
  CHECK(point.ambiguity==2 && point.latitude_e5==4452500 && point.longitude_e5==-11212500);
  rt950::aprs::RecentStations mic_e_stations;
  mic_e_stations.record(mic_e,sizeof mic_e,100);
  CHECK(mic_e_stations.at(0) && mic_e_stations.at(0)->position.ambiguity==2);
  mic_e[5]='0'<<1;
  CHECK(!rt950::aprs::position(mic_e,sizeof mic_e,point));
  mic_e_stations.record(mic_e,sizeof mic_e,200);
  CHECK(mic_e_stations.at(0)->received_at==100);
  const char degree_area[]="T4KZZZ";
  for(unsigned i=0;i<6;++i)mic_e[i]=uint8_t(degree_area[i]<<1);
  CHECK(rt950::aprs::position(mic_e,sizeof mic_e,point));
  CHECK(point.ambiguity==4 && point.latitude_e5==4450000 && point.longitude_e5==-11250000);
  mic_e[3]='K'<<1;
  CHECK(!rt950::aprs::position(mic_e,sizeof mic_e,point));
 }
 {
  uint8_t approximate[36]{};
  for(unsigned i=0;i<16;++i)approximate[i]=aprs_packet[i];
  const char payload[]="!4903.  N/07201.75W>";
  for(unsigned i=0;i<sizeof payload-1;++i)approximate[16+i]=uint8_t(payload[i]);
  CHECK(rt950::aprs::position(approximate,sizeof approximate,point));
  CHECK(point.ambiguity==2 && point.latitude_e5==4905833 && point.longitude_e5==-7202500);
  approximate[32]=' ';approximate[33]=' ';
  CHECK(rt950::aprs::position(approximate,sizeof approximate,point) && point.longitude_e5==-7202500);
  approximate[33]='5';
  CHECK(!rt950::aprs::position(approximate,sizeof approximate,point));
  approximate[33]=' ';approximate[23]='1';
  CHECK(!rt950::aprs::position(approximate,sizeof approximate,point));
  CHECK(point.ambiguity==2 && point.longitude_e5==-7202500);
  int32_t exact=0;
  approximate[23]=' ';
  CHECK(!rt950::aprs::coordinate(approximate+17,2,true,exact));
  const char latitudes[][9]={"4903.5 N","4903.  N","490 .  N","49  .  N"};
  constexpr int32_t midpoints[]={4905917,4905833,4908333,4950000};
  for(unsigned level=0;level<4;++level) {
   uint8_t ambiguity=0;
   const auto *lat=reinterpret_cast<const uint8_t *>(latitudes[level]);
   CHECK(rt950::aprs::coordinate(lat,2,true,exact,&ambiguity));
   CHECK(ambiguity==level+1 && exact==midpoints[level]);
  }
 }
 {
  uint8_t report[64]{};
  for(unsigned i=0;i<16;++i)report[i]=aprs_packet[i];
  auto load_report=[&](const char *payload) {
   unsigned at=16;
   while(*payload)report[at++]=uint8_t(*payload++);
   return at;
  };
  rt950::aprs::RecentStations reports;
  reports.record(aprs_packet,sizeof aprs_packet-2,100);
  const unsigned object_size=load_report(";N0CALL   *092345z/5L!!<*e7>7P[");
  reports.record(report,object_size,200);
  CHECK(reports.at(0)->position.kind==rt950::aprs::PositionKind::Object);
  CHECK(reports.at(0)->position.latitude_e5==4950000 && reports.at(1)->received_at==100);
  const unsigned item_size=load_report(")N0CALL!4903.50N/07201.75W>");
  reports.record(report,item_size,300);
  CHECK(reports.at(0)->position.kind==rt950::aprs::PositionKind::Item);
  CHECK(reports.at(1)->received_at==200 && reports.at(2)->received_at==100);
  report[23]='_';
  reports.record(report,item_size-1,400);
  CHECK(reports.at(0)->received_at==300);
  reports.record(report,item_size,500);
  CHECK(reports.at(0)->received_at==200 && reports.at(1)->received_at==100 && !reports.at(2));
  load_report(";N0CALL   _092345z/5L!!<*e7>7P[");
  report[7]='K'<<1;
  reports.record(report,object_size,600);
  CHECK(reports.at(0)->received_at==100 && !reports.at(1));
 }
 {
  rt950::aprs::RecentMessages messages;
  CHECK(messages.count()==0);
  uint8_t frame[42]{};
  for(unsigned i=0;i<16;++i)frame[i]=aprs_packet[i];
  const char payload[]=":W1ABC    :Hello{12a";
  for(unsigned i=0;i<sizeof payload-1;++i)frame[16+i]=uint8_t(payload[i]);
  const size_t length=16+sizeof payload-1;
  messages.record(frame,length);
  CHECK(messages.at(0) && messages.at(0)->body[0]=='H' &&
      messages.at(0)->destination[5]==0 && messages.at(0)->id[2]=='a');
  messages.record(frame,length);
  CHECK(!messages.at(1) && messages.count()==1);
  frame[length-1]=' ';
  messages.record(frame,length);
  CHECK(!messages.at(1));
  frame[length-1]='B';frame[13]=0x65;
  messages.record(frame,length);
  CHECK(messages.at(0)->ssid==2 && messages.at(1)->ssid==0);
 }
 rt950::aprs::FrameTransmitter aprs_tx;
 CHECK(aprs_tx.encode(aprs_packet,sizeof aprs_packet-2,16));
 bool prior_level=false;
 for(size_t i=0;i<aprs_tx.size();++i) {
  const bool level=aprs_tx.level(i);
  feed(level==prior_level);
  prior_level=level;
 }
 CHECK(capture.count==3 && capture.length==sizeof aprs_packet+1);
 for(unsigned i=0;i<sizeof aprs_packet-2;++i)CHECK(capture.kiss[i+2]==aprs_packet[i]);
 const char beacon_call[7]="N0CALL";
 CHECK(rt950::aprs::encode_beacon(aprs_tx,beacon_call,9,"5545.0000",'N',"03737.0000",'E'));
 for(unsigned offset=0;offset<4;++offset) {
  rt950::aprs::AfskReceiver modem;
  rt950::aprs::AfskWave wave;
  AprsCapture audio_capture;
  for(size_t symbol=0;symbol<aprs_tx.size()+3;++symbol) {
   for(unsigned sample=0;sample<32;++sample) {
    const auto value=wave.sample(symbol<aprs_tx.size() && !aprs_tx.level(symbol));
    if(sample%4==offset) {
     const int noise=int((symbol*17+sample*7)%31)-15;
     modem.sample(uint16_t(1470+(int(value)-2048)/2+noise),capture_aprs,&audio_capture);
    }
   }
  }
  CHECK(audio_capture.count==1 && audio_capture.length==39);
  CHECK(rt950::aprs::position(audio_capture.kiss+2,audio_capture.length-3,point));
  CHECK(point.latitude_e5==5575000 && point.longitude_e5==3761667 && point.ssid==9);
 }

 prior_level=false;
 for(size_t i=0;i<aprs_tx.size();++i) {
  const bool level=aprs_tx.level(i);
  feed(level==prior_level);
  prior_level=level;
 }
 CHECK(capture.count==4 && capture.length==39);
 CHECK(rt950::aprs::position(capture.kiss+2,capture.length-3,point));
 CHECK(point.latitude_e5==5575000 && point.longitude_e5==3761667 && point.ssid==9);
 rt950::aprs::RecentStations stations;
 uint8_t *station_frame=capture.kiss+2;
 for(unsigned ssid=1;ssid<=4;++ssid) {
  station_frame[13]=uint8_t(0x61u|(ssid<<1));
  stations.record(station_frame,capture.length-3,ssid*1000u);
 }
 CHECK(stations.at(0)->position.ssid==4 && stations.at(2)->position.ssid==2 && !stations.at(3));
 station_frame[13]=0x65;
 stations.record(station_frame,capture.length-3,5000);
 CHECK(stations.at(0)->position.ssid==2 && stations.at(0)->received_at==5000);
 CHECK(stations.at(1)->position.ssid==4 && stations.at(2)->position.ssid==3);
 stations.record(station_frame,17,6000);
 CHECK(stations.at(0)->received_at==5000);
 struct PacketAudio {
  enum class State { Running,Complete };
  bool active=false,available=true;
  bool begin(const rt950::aprs::FrameTransmitter &,uint32_t) {return active=available;}
  State poll(uint32_t) const {return State::Running;}
  void cancel(){active=false;}
 } audio;
 TxBackend packet_radio;
 rt950::aprs::BeaconSession<TxBackend,PacketAudio> beacon(packet_radio,audio);
 beacon.start(144800000,0);beacon.poll(aprs_tx,0,false);
 CHECK(!beacon.active() && !audio.active && !packet_radio.keys);
 beacon.start(144800000,1);beacon.poll(aprs_tx,1,true);
 CHECK(beacon.transmitting() && audio.active && packet_radio.keys==1);
 beacon.poll(aprs_tx,2,false);
 CHECK(!beacon.active() && !audio.active && packet_radio.stops==1);
 audio.available=false;
 beacon.start(144800000,3);beacon.poll(aprs_tx,3,true);
 CHECK(!beacon.active() && packet_radio.stops==2);
 CHECK(!aprs_tx.encode(aprs_packet,sizeof aprs_packet-2,0) && !aprs_tx.size());
 CHECK(!aprs_tx.encode(aprs_packet,331,1) && !aprs_tx.size());
 uint8_t max_frame[rt950::aprs::FrameTransmitter::MaxFrame];
 for(unsigned i=0;i<sizeof max_frame;++i)max_frame[i]=i<16?aprs_packet[i]:0xff;
 CHECK(aprs_tx.encode(max_frame,sizeof max_frame,64));
 CHECK(aprs_tx.size()>sizeof max_frame*8 && !aprs_tx.level(aprs_tx.size()));

 FactorySpi spi;
 uint8_t factory_read[2]{};
 CHECK(voice::read_factory_flash(spi,0xf0e0,factory_read,2));
 CHECK(!spi.selected && spi.count==6 && spi.bytes[0]==3 && spi.bytes[1]==0 &&
       spi.bytes[2]==0xf0 && spi.bytes[3]==0xe0 && factory_read[1]==0x71);
 spi.fail_after=8;
 CHECK(!voice::read_factory_flash(spi,0xf0e0,factory_read,2) && !spi.selected);
 voice::UhfCalibration cal;
 bool erased=false;uint8_t raw_tone=0;
 auto read_factory=[&](uint32_t address,uint8_t *data,size_t count) {
  for(size_t i=0;i<count;++i)data[i]=0;
  if(address==0xf0e0 && count==1)data[0]=erased?0xff:6;
  else if(address==0xf210 && count==16) {
   data[0]=55;data[1]=raw_tone;data[2]=uint8_t(255u-raw_tone);data[11]=0xa0;data[12]=0xe1;data[13]=0xff;
  } else if((address==0xf000 || address==0xf040 || address==0xf080) && count==13) {
   for(size_t i=0;i<count;++i)data[i]=uint8_t(100+(address-0xf000)/4+i);
  } else return false;
  return true;
 };
 CHECK(voice::load_uhf_calibration(read_factory,cal) && cal.microphone==55 &&
       cal.frequency_correction==-31 && cal.pa[2][4]==136);
 for(unsigned raw=0;raw<256;++raw) {
  raw_tone=uint8_t(raw);
  CHECK(voice::load_uhf_calibration(read_factory,cal));
  CHECK(cal.ctcss_raw==raw && cal.ctcss_level==(raw&127u));
  CHECK(cal.dcs_raw==255u-raw && cal.dcs_level==((255u-raw)&127u));
 }
 voice::UhfTxValues tx_values;
 CHECK(voice::uhf_tx_values(cal,446081250,2,0xe912,tx_values));
 CHECK((uint32_t(tx_values.frequency_high)<<16 | tx_values.frequency_low)==44608094 &&
       tx_values.mic==0xe937 && tx_values.pa_bias==136);
 CHECK(!voice::uhf_tx_values(cal,145500000,2,0xe912,tx_values));
 CHECK(!voice::uhf_tx_values(cal,446081250,3,0xe912,tx_values));
 erased=true;
 CHECK(!voice::load_uhf_calibration(read_factory,cal) && cal.pa[2][4]==136);
 voice::Ptt ptt;
 TxBackend tx;
 voice::TxRequest request{446081250,15,0,true,true};
 ptt.poll(tx,request,0,true);
 CHECK(tx.begins==0);
 ptt.poll(tx,request,1,false);
 tx.clear=false;ptt.poll(tx,request,2,true);
 CHECK(tx.begins==1 && tx.keys==0 && tx.stops==1);
 tx.clear=true;ptt.poll(tx,request,3,true);
 CHECK(tx.begins==1);
 ptt.poll(tx,request,4,false);
 tx.readiness=voice::Ptt::Preparation::Pending;
 ptt.poll(tx,request,10,true);
 CHECK(tx.keys==0 && ptt.state()==voice::Ptt::State::Preparing);
 ptt.poll(tx,request,260,true);
 CHECK(tx.stops==2 && ptt.state()==voice::Ptt::State::WaitRelease);
 ptt.poll(tx,request,261,false);
 tx.readiness=voice::Ptt::Preparation::Ready;
 ptt.poll(tx,request,300,true);
 CHECK(tx.keys==1 && ptt.state()==voice::Ptt::State::Transmitting);
 ptt.poll(tx,request,15300,true);
 CHECK(tx.stops==3 && ptt.state()==voice::Ptt::State::WaitRelease);
 ptt.poll(tx,request,15301,false);
 ptt.poll(tx,request,15302,true);
 ptt.poll(tx,request,15303,false);
 CHECK(tx.keys==2 && tx.stops==4 && ptt.state()==voice::Ptt::State::Idle);
 request.authorized=false;
 ptt.poll(tx,request,15304,true);
 CHECK(tx.keys==2 && ptt.state()==voice::Ptt::State::WaitRelease);
 {
  TxBackend end_radio;
  voice::Ptt end_ptt;
  voice::TxRequest end_request{446081250,15,1,true,true,true};
  CHECK(voice::tail_inputs_released(6,1));
  CHECK(!voice::tail_inputs_released(1,1) && !voice::tail_inputs_released(2,3));
  end_ptt.poll(end_radio,end_request,0,false);
  end_ptt.poll(end_radio,end_request,1,true);
  end_ptt.poll(end_radio,end_request,10,false);
  CHECK(end_ptt.state()==voice::Ptt::State::Ending && end_ptt.target_slot()==1 && end_radio.tails==1 && !end_radio.stops);
  end_ptt.poll(end_radio,end_request,409,false);
  CHECK(end_ptt.state()==voice::Ptt::State::Ending);
  end_ptt.poll(end_radio,end_request,410,false);
  CHECK(end_ptt.state()==voice::Ptt::State::Idle && end_radio.stops==1);
  end_ptt.poll(end_radio,end_request,500,true);
  end_ptt.poll(end_radio,end_request,501,false);
  end_request.authorized=false;
  end_ptt.poll(end_radio,end_request,502,false);
  CHECK(end_ptt.state()==voice::Ptt::State::Idle && end_radio.stops==2 &&
        end_ptt.release_error()==voice::Ptt::ReleaseError::Permission);
  end_request.authorized=true;
  end_ptt.poll(end_radio,end_request,600,true);
  end_ptt.poll(end_radio,end_request,601,false);
  end_ptt.poll(end_radio,end_request,602,true);
  CHECK(end_ptt.state()==voice::Ptt::State::WaitRelease && end_radio.stops==3);
  end_ptt.poll(end_radio,end_request,603,true);
  CHECK(end_radio.keys==3);
  end_ptt.poll(end_radio,end_request,604,false);
  end_ptt.poll(end_radio,end_request,1000,true);
  end_ptt.poll(end_radio,end_request,15999,false);
  CHECK(end_ptt.state()==voice::Ptt::State::Ending);
  end_ptt.poll(end_radio,end_request,16000,false);
  CHECK(end_ptt.state()==voice::Ptt::State::Idle);
  end_ptt.poll(end_radio,end_request,0xfffffff0u,true);
  end_ptt.poll(end_radio,end_request,0xfffffff1u,false);
  end_ptt.poll(end_radio,end_request,385u,false);
  CHECK(end_ptt.state()==voice::Ptt::State::Idle);
  end_radio.tail_ok=false;
  end_ptt.poll(end_radio,end_request,500,true);
  end_ptt.poll(end_radio,end_request,501,false);
  CHECK(end_ptt.state()==voice::Ptt::State::Idle &&
        end_ptt.release_error()==voice::Ptt::ReleaseError::Signalling);
 }
 TxBoard tx_board;
 tx_board.cal=cal;
 voice::UhfTransmitter<TxBoard> uhf_tx(tx_board);
 voice::Ptt integrated;
 const voice::TxRequest target{446081250,15,1,true,true};
 integrated.poll(uhf_tx,target,0,false);
 integrated.poll(uhf_tx,target,1,true);
 CHECK(tx_board.tuned==446081250 && tx_board.bias_value==0 && tx_board.route_writes==0);
 tx_board.observation=TxBoard::Channel::Busy;
 integrated.poll(uhf_tx,target,10,true);
 CHECK(integrated.blocked_by_carrier() && tx_board.restores==1 && tx_board.nonzero_bias==0 && tx_board.route_writes==0);
 integrated.poll(uhf_tx,target,10,true);CHECK(integrated.blocked_by_carrier() && tx_board.restores==1);
 integrated.poll(uhf_tx,target,11,false);
 CHECK(!integrated.blocked_by_carrier());
 tx_board.observation=TxBoard::Channel::Clear;
 tx_board.configure_ok=false;
 integrated.poll(uhf_tx,target,12,true);
 CHECK(!integrated.blocked_by_carrier() && tx_board.restores==2 && tx_board.nonzero_bias==0 && tx_board.route_writes==6);
 integrated.poll(uhf_tx,target,13,false);
 tx_board.configure_ok=true;
 integrated.poll(uhf_tx,target,14,true);
 CHECK(integrated.state()==voice::Ptt::State::Transmitting && tx_board.bias_value==136 &&
       tx_board.route_writes==15 && tx_board.register_writes==16 && tx_board.delay_total==100);
 integrated.poll(uhf_tx,target,15,false);
 CHECK(tx_board.restores==3 && !tx_board.bad_restore && !tx_board.acquired);
 integrated.poll(uhf_tx,target,16,true);
 tx_board.healthy_flag=false;
 integrated.poll(uhf_tx,target,17,true);
 CHECK(tx_board.restores==4 && tx_board.bias_value==0 && !tx_board.bad_restore &&
       integrated.state()==voice::Ptt::State::WaitRelease);
 VoicePins write_pins;
 voice::RegisterBus<VoicePins> writer(write_pins);
 CHECK(!writer.write(2,0x38,0x1234) && write_pins.writes==0);
 CHECK(writer.write(0,0x38,0x1234) && write_pins.sent==0x381234 &&
       write_pins.writes==24 && write_pins.selected<0);
 VoicePins read_pins;
 voice::RegisterBus<VoicePins> reader(read_pins);
 uint16_t rf_reply=0;
 CHECK(reader.read(0,0x67,rf_reply) && rf_reply==0xA153 && read_pins.sent==0xE7 &&
       read_pins.writes==8 && read_pins.reads==16 && !read_pins.bad_sample && read_pins.selected<0);

 rt_config voice_config;
 rt_defaults(&voice_config);
 voice_config.vfo[0].rx_hz=145500000;
 voice_config.vfo[1].rx_hz=446081250;
 voice_config.vfo[2].rx_hz=433500000;
 voice::Watch watch;
 VoiceReceiver voice_rx;
 watch.poll(voice_rx,voice_config,0,true,false);
 CHECK(watch.slot()==0 && voice_rx.frequency==145500000);
 watch.poll(voice_rx,voice_config,150,true,false);
 CHECK(watch.slot()==1 && voice_rx.frequency==446081250);
 voice_rx.carrier=true;
 watch.poll(voice_rx,voice_config,151,true,false);
 CHECK(!watch.receiving());
 watch.poll(voice_rx,voice_config,175,true,false);
 CHECK(watch.receiving());
 watch.poll(voice_rx,voice_config,500,true,false);
 CHECK(watch.slot()==1 && watch.receiving());
 voice_rx.carrier=false;
 watch.poll(voice_rx,voice_config,1499,true,false);
 CHECK(watch.slot()==1 && !watch.receiving());
 watch.poll(voice_rx,voice_config,1500,true,false);
 CHECK(watch.slot()==2 && voice_rx.frequency==433500000);
 watch.poll(voice_rx,voice_config,1501,true,true);
 CHECK(watch.slot()==0 && voice_rx.frequency==145500000 && !watch.receiving());
 voice_config.active=1;voice_config.watch_mode=RT_WATCH_SINGLE;
 watch.poll(voice_rx,voice_config,2000,true,false);
 watch.poll(voice_rx,voice_config,3000,true,false);
 CHECK(watch.slot()==1 && voice_rx.frequency==446081250);
 watch.poll(voice_rx,voice_config,3001,false,false);
 CHECK(!voice_rx.frequency && !watch.receiving());
 CHECK(voice::ptt_source(2)==2 && voice::ptt_source(6)==4 && voice::ptt_source(4)==4 && voice::ptt_source(5)==1);
 CHECK(voice::ptt_slot(4,0)==2 && voice::ptt_slot(2,0)==1 && voice::ptt_slot(1,1)==1);
 TxBackend side_radio;
 voice::Ptt side_ptt;
 const voice::TxRequest side_request{433500000,60,voice::ptt_slot(4,0),true,true};
 const voice::TxRequest main_request{145500000,60,voice::ptt_slot(1,0),true,true};
 side_ptt.poll(side_radio,side_request,10,true);
 CHECK(!side_radio.keys);
 side_ptt.poll(side_radio,{},11,false);
 side_ptt.poll(side_radio,side_request,12,true);
 CHECK(side_radio.keys==1 && side_ptt.state()==voice::Ptt::State::Transmitting);
 side_ptt.poll(side_radio,main_request,13,true);
 CHECK(side_radio.stops==1 && side_ptt.state()==voice::Ptt::State::WaitRelease);
 side_ptt.poll(side_radio,main_request,14,true);
 CHECK(side_radio.keys==1);
 side_ptt.poll(side_radio,{},15,false);
 side_ptt.poll(side_radio,main_request,16,true);
 CHECK(side_radio.keys==2 && side_ptt.state()==voice::Ptt::State::Transmitting);

 Application watch_settings;
 {
  Application shortcuts;
  shortcuts.key(Key::Left);
  CHECK(shortcuts.page()==Page::WatchSlots && shortcuts.config().active==0);
  shortcuts.key(Key::Back);CHECK(shortcuts.page()==Page::Radio);
  shortcuts.key(Key::Left);shortcuts.key(Key::Down);shortcuts.key(Key::Ok);
  CHECK(shortcuts.page()==Page::Radio && shortcuts.watch_slots_selection()==2);
  const auto frequency=shortcuts.config().vfo[1].rx_hz;
  shortcuts.tick(100);
  shortcuts.observe_voice_slot(1,frequency+10);
  CHECK(shortcuts.recent_voice_slot()==-1);
  shortcuts.observe_voice_slot(1,frequency);
  CHECK(shortcuts.recent_voice_slot()==1 && shortcuts.config().active==0);
  shortcuts.digit(4);shortcuts.key(Key::Right);
  CHECK(shortcuts.config().active==0 && shortcuts.editing() && !shortcuts.voice_tx_page());
  shortcuts.key(Key::Back);shortcuts.key(Key::Right);
  CHECK(shortcuts.config().active==1 && shortcuts.voice_tx_page());
  shortcuts.set_frequency(145500000);CHECK(shortcuts.recent_voice_slot()==-1);
  shortcuts.observe_voice_slot(2,shortcuts.config().vfo[2].rx_hz);
  shortcuts.tick(5099);CHECK(shortcuts.recent_voice_slot()==2);
  shortcuts.tick(5100);shortcuts.key(Key::Right);
  CHECK(shortcuts.recent_voice_slot()==-1 && shortcuts.config().active==1);
 }
 watch_settings.key(Key::Menu);
 watch_settings.key(Key::WatchDwell);
 watch_settings.digit(9);watch_settings.digit(9);watch_settings.key(Key::Ok);
 CHECK(watch_settings.page()==Page::WatchDwell && watch_settings.config().dwell_ms==150);
 watch_settings.key(Key::Erase);watch_settings.key(Key::Erase);
 watch_settings.digit(2);watch_settings.digit(5);watch_settings.digit(0);watch_settings.key(Key::Ok);
 CHECK(watch_settings.page()==Page::Settings && watch_settings.config().dwell_ms==250);
 watch_settings.key(Key::WatchHold);watch_settings.digit(0);watch_settings.key(Key::Ok);
 CHECK(watch_settings.config().hold_ms==0);
 watch_settings.key(Key::WatchHold);watch_settings.digit(8);watch_settings.key(Key::Back);
 CHECK(watch_settings.page()==Page::Settings && watch_settings.config().hold_ms==0);
 rt_watch timing{};
 const auto &watch_config=watch_settings.config();
 CHECK(rt_watch_tick(&timing,&watch_config,0,false,false)==0);
 CHECK(rt_watch_tick(&timing,&watch_config,249,false,false)==0);
 CHECK(rt_watch_tick(&timing,&watch_config,250,false,false)==1);
 CHECK(rt_watch_tick(&timing,&watch_config,501,true,false)==1);
 CHECK(rt_watch_tick(&timing,&watch_config,502,false,false)==2);
 watch_settings.key(Key::WatchSlots);
 CHECK(watch_settings.watch_slots_draft()==1);
 watch_settings.key(Key::Down);watch_settings.key(Key::Ok);
 CHECK(watch_config.watch_mode==RT_WATCH_MASK && watch_config.watch_mask==3);
 timing={};
 CHECK(rt_watch_tick(&timing,&watch_config,0,false,false)==0);
 CHECK(rt_watch_tick(&timing,&watch_config,250,false,false)==1);
 CHECK(rt_watch_tick(&timing,&watch_config,500,false,false)==0);
 watch_settings.key(Key::WatchSlots);watch_settings.key(Key::Down);watch_settings.key(Key::Back);
 CHECK(watch_config.watch_mask==3);
 CHECK(watch_settings.set_option(Option::Watch,false) && watch_config.watch_mode==RT_WATCH_SINGLE);
 CHECK(watch_settings.set_option(Option::Watch,true) && watch_config.watch_mode==RT_WATCH_MASK);
 watch_settings.key(Key::Back);watch_settings.key(Key::Back);
 watch_settings.key(Key::NextSlot);watch_settings.key(Key::NextSlot);
 CHECK(watch_config.active==2);
 CHECK(rt_watch_tick(&timing,&watch_config,750,false,false)==2);
 CHECK(rt_watch_tick(&timing,&watch_config,1000,false,false)==0);
 CHECK(rt_watch_tick(&timing,&watch_config,1001,false,true)==2);


 fm::Receiver receiver;
 MockFm radio;
 fm::Signal signal;
 CHECK(!receiver.tune(10801,0) && receiver.tune(10030,0));
 for(uint32_t now=0;now<=85;now+=5)receiver.poll(radio,now);
 CHECK(radio.operations==9 && receiver.frequency()==10030 && receiver.state()==fm::Receiver::State::Tuned);
 CHECK(receiver.signal(85,signal) && signal.rssi==48 && signal.snr==19 && signal.stereo && signal.valid);
 CHECK(fm::quality(signal)==fm::Quality::Good);
 const uint8_t afc_reply[8]={0x80,0,3,0,48,19,0,0};
 CHECK(fm::decode_signal(afc_reply,8,signal) && fm::quality(signal)==fm::Quality::NoStation);
 CHECK(!receiver.signal(1580,signal));
 CHECK(receiver.tune(10030,0xfffffff0u));
 CHECK(!receiver.signal(0xfffffff0u,signal));
 radio.stalled=true;receiver.poll(radio,0xfffffff0u);receiver.poll(radio,984);
 CHECK(receiver.state()==fm::Receiver::State::Fault);
 radio.stalled=false;radio.error=true;
 CHECK(receiver.tune(10030,1100));receiver.poll(radio,1100);
 CHECK(receiver.state()==fm::Receiver::State::Fault);
 fm::Probe probe;
 MockProbeBus probe_bus;
 bool reset_released=false;
 probe.begin(0);
 for(uint32_t now=0;now<=800;now+=5)probe.poll(probe_bus,[&](bool high){reset_released=high;},now);
 CHECK(reset_released && probe_bus.powered && probe_bus.revision && probe_bus.properties==6);
 CHECK(probe.phase()==fm::Probe::Phase::Receiver && probe.part()==32 && probe.frequency()==10030);
 CHECK(probe.signal(800,signal) && signal.rssi==48);
 probe.end();CHECK(!probe.signal(800,signal) && probe.phase()==fm::Probe::Phase::Off);
 probe_bus.wire_error=true;probe.begin(0);
 for(uint32_t now=0;now<=1000;now+=5)probe.poll(probe_bus,[](bool){},now);
 CHECK(probe.failure()==fm::Probe::Failure::NoReply);
 probe_bus.wire_error=false;probe_bus.stalled=true;probe.begin(0);
 for(uint32_t now=0;now<=1000;now+=5)probe.poll(probe_bus,[](bool){},now);
 CHECK(probe.failure()==fm::Probe::Failure::Timeout);
 MockProbeBus write_first;write_first.no_initial_reads=true;probe.begin(0);
 for(uint32_t now=0;now<=1800;now+=5)probe.poll(write_first,[](bool){},now);
 CHECK(probe.frequency()==10030 && write_first.powered && write_first.revision);
 MockLines nack;nack.samples=7;
 fm::I2cBus<MockLines> nack_bus(nack);
 const uint8_t byte=0x23;
 CHECK(!nack_bus.write(0x11,&byte,1) && nack.clock_high && nack.released && !nack.sampled_driven);
 MockLines reply;
 reply.samples=1u|(1u<<10);
 for(unsigned bit=0;bit<8;++bit)reply.samples|=((0xA5u>>(7-bit))&1u)<<(2+bit);
 fm::I2cBus<MockLines> reply_bus(reply);
 uint8_t status=0;
 CHECK(reply_bus.read(0x11,&status,1) && status==0xA5 && reply.reads==11 &&
       reply.clock_high && reply.released && !reply.sampled_driven);
 MockLines stuck;stuck.blocked=true;
 fm::I2cBus<MockLines> stuck_bus(stuck);
 CHECK(!stuck_bus.write(0x11,&byte,1) && stuck.delays<70);
 uint8_t previous[HeaderSize+3]{},replacement[HeaderSize+3]{};
 previous[HeaderSize]=7;replacement[HeaderSize]=8;
 prepare(previous,1,0xFFFFFFFFu,previous+HeaderSize,3);
 put(previous+CommitOffset,Commit);
 prepare(replacement,1,0,replacement+HeaderSize,3);
 CHECK(newest(inspect(previous,sizeof previous,1),inspect(replacement,sizeof replacement,1)).sequence==0xFFFFFFFFu);
 for(unsigned n=1;n<=4;++n) {
  replacement[CommitOffset+n-1]=uint8_t(Commit>>(8*(n-1)));
  const auto selected=newest(inspect(previous,sizeof previous,1),inspect(replacement,sizeof replacement,1));
  CHECK(selected.payload && selected.payload[0]==(n<4?7:8));
 }
 CHECK(!inspect(replacement,sizeof replacement,2).payload);
 CHECK(!inspect(replacement,sizeof replacement-1,1).payload);
 replacement[HeaderSize]^=1;
 CHECK(newest(inspect(previous,sizeof previous,1),inspect(replacement,sizeof replacement,1)).sequence==0xFFFFFFFFu);
 const uint8_t old_data[4]={7,0,0,0},new_data[4]={8,0,0,0};
 MockSlots intact;
 CHECK(save(intact,1,old_data,sizeof old_data)==SaveResult::Saved);
 CHECK(save(intact,1,old_data,sizeof old_data)==SaveResult::Unchanged && intact.erases==1);
 CHECK(save(intact,2,new_data,sizeof new_data)==SaveResult::Invalid && intact.erases==1);
 // Interrupt each byte of erase, header, payload and commit, then retry.
 for(int cut=0;cut<60;++cut) {
  auto interrupted=intact;interrupted.remaining=cut;
  CHECK(save(interrupted,1,new_data,sizeof new_data)==SaveResult::Failed);
  const auto selected=newest(inspect(interrupted.view(0),32,1),inspect(interrupted.view(1),32,1));
  CHECK(selected.payload && selected.payload[0]==7);
  interrupted.remaining=-1;
  CHECK(save(interrupted,1,new_data,sizeof new_data)==SaveResult::Saved);
  CHECK(inspect(interrupted.view(1),32,1).payload[0]==8);
 }
 CHECK(oem_control_crc(reinterpret_cast<const uint8_t *>("123456789"),9)==0x31c3);
 Application factory;
 const auto *pmr7=factory.memory(6);
 CHECK(pmr7->rx_hz==446081250 && !pmr7->tx_hz && pmr7->radio.narrow);
 CHECK(pmr7->radio.rx_tone.kind==ToneKind::Off && pmr7->radio.tx_tone.kind==ToneKind::Off);
 CHECK(rt_config_valid(&factory.config()));
 CHECK(factory.config().vfo[0].rx_hz==446006250 && factory.config().vfo[1].rx_hz==446018750);
 CHECK(factory.config().vfo[2].rx_hz==433075000);
 for(const auto &slot:factory.config().vfo)CHECK(!slot.tx_hz && !slot.channel_mode);
 factory.key(Key::Up);
 CHECK(factory.config().vfo[0].rx_hz==446018750 && !factory.config().vfo[0].tx_hz);
 CHECK(factory.store(63) && factory.memory(63)->tx_hz==0);
 CHECK(factory.config().aprs_rx_hz==144800000 && factory.config().beacon_seconds==60);
 CHECK(!factory.config().aprs_hz && !factory.config().beacon_on);
 CHECK(factory.enabled(Option::AprsRx) && !factory.enabled(Option::Beacon) &&
       !factory.enabled(Option::Ble) && !factory.enabled(Option::Export));
 CHECK(!factory.set_option(Option::Export,true));
 CHECK(!factory.set_option(Option::Beacon,true));
 Application bilingual;
 bilingual.key(Key::Menu);bilingual.key(Key::LanguageSelect);bilingual.key(Key::Down);
 bilingual.key(Key::Back);CHECK(bilingual.language()==Language::English);
 bilingual.key(Key::LanguageSelect);bilingual.key(Key::Down);bilingual.key(Key::Ok);
 CHECK(bilingual.language()==Language::Russian);
 bilingual.key(Key::VfoStep);bilingual.key(Key::Down);bilingual.key(Key::Ok);
 CHECK(Application::vfo_steps[bilingual.applied_vfo_step()]==20000);
 bilingual.key(Key::Back);bilingual.key(Key::Up);
 CHECK(bilingual.config().vfo[0].rx_hz==446026250 && !bilingual.config().vfo[0].tx_hz);
 bilingual.key(Key::NextSlot);
 CHECK(Application::vfo_steps[bilingual.applied_vfo_step()]==12500);
 Application profiles;
 CHECK(profiles.active_radio_profile().squelch==3 && profiles.active_radio_profile().narrow);
 CHECK(profiles.active_radio_profile().power==TxPower::Low && profiles.active_radio_profile().busy_lock);
 CHECK(profiles.active_radio_profile().timeout_seconds==60);
 profiles.key(Key::Menu);profiles.key(Key::Ok);
 profiles.key(Key::Down);profiles.key(Key::Down);profiles.key(Key::Ok);
 CHECK(profiles.page()==Page::RadioOptions);
 profiles.key(Key::Down);profiles.key(Key::Back);
 CHECK(profiles.active_radio_profile().squelch==3 && profiles.menu_selection()==2);
 for(unsigned i=0;i<unsigned(RadioSetting::RxCtcss);++i) {
  profiles.key(Key::Ok);CHECK(profiles.radio_setting()==RadioSetting(i));
  profiles.key(Key::Down);profiles.key(Key::Ok);
  CHECK(profiles.page()==Page::Settings);
  if(i+1<unsigned(RadioSetting::RxCtcss))profiles.key(Key::Down);
 }
 const auto updated=profiles.active_radio_profile();
 CHECK(updated.squelch==4 && !updated.narrow && updated.power==TxPower::Middle);
 CHECK(!updated.busy_lock && updated.timeout_seconds==120);
 CHECK(profiles.store(63) && profiles.memory(63)->radio.squelch==4 && !profiles.memory(63)->tx_hz);
 profiles.key(Key::Back);profiles.key(Key::Back);profiles.key(Key::NextSlot);
 CHECK(profiles.active_radio_profile().squelch==3);
 CHECK(profiles.recall(63) && profiles.active_radio_profile().timeout_seconds==120);
 profiles.key(Key::Mode);CHECK(profiles.active_radio_profile().timeout_seconds==60);
 RadioProfile tones;
 char label[8];
 CHECK(radio_option_count(RadioSetting::RxCtcss)==51);
 CHECK(radio_option_count(RadioSetting::RxDcs)==211);
 radio_option_label(RadioSetting::RxCtcss,1,label);
 CHECK(label[0]=='6' && label[1]=='7' && label[2]=='.' && label[3]=='0' && !label[4]);
 radio_option_label(RadioSetting::TxDcs,210,label);
 CHECK(label[0]=='D' && label[1]=='7' && label[2]=='5' && label[3]=='4' && label[4]=='I' && !label[5]);
 radio_profile_apply(tones,RadioSetting::RxCtcss,1);
 radio_profile_apply(tones,RadioSetting::TxCtcss,50);
 CHECK(tones.rx_tone.index==1 && tones.tx_tone.index==50);
 CHECK(ctcss_frequency(tones.rx_tone)==670 && ctcss_frequency(tones.tx_tone)==2541);
 radio_profile_apply(tones,RadioSetting::RxDcs,106);
 CHECK(tones.rx_tone.kind==ToneKind::Dcs && radio_profile_option(tones,RadioSetting::RxDcs)==106);
 CHECK(radio_profile_option(tones,RadioSetting::RxCtcss)==0 && tones.tx_tone.kind==ToneKind::Ctcss);
 radio_profile_apply(tones,RadioSetting::RxCtcss,0);
 CHECK(tones.rx_tone.kind==ToneKind::Dcs);
 radio_profile_apply(tones,RadioSetting::TxDcs,210);
 CHECK(tones.tx_tone.kind==ToneKind::Dcs && tones.tx_tone.index==210);
 radio_profile_apply(tones,RadioSetting::TxDcs,0);
 CHECK(tones.tx_tone.kind==ToneKind::Off);
 Application repeater;
 CHECK(repeater.tx_status()[0]=='S');
 CHECK(repeater.set_frequency(145600000));
 repeater.key(Key::Menu);repeater.key(Key::Ok);repeater.key(Key::Up);repeater.key(Key::Ok);
 CHECK(repeater.menu_group()==MenuGroup::Repeater);
 repeater.key(Key::Ok);CHECK(repeater.page()==Page::TxFrequency);
 for(char d:"145000")if(d)repeater.digit(unsigned(d-'0'));
 repeater.key(Key::Ok);CHECK(repeater.config().vfo[0].tx_hz==145000000);
 repeater.key(Key::Down);repeater.key(Key::Ok);repeater.key(Key::Up);repeater.key(Key::Ok);
 CHECK(repeater.active_radio_profile().shift==Shift::Minus && !repeater.config().vfo[0].tx_hz);
 repeater.key(Key::Down);repeater.key(Key::Ok);CHECK(repeater.page()==Page::RepeaterOffset);
 for(char d:"000600")if(d)repeater.digit(unsigned(d-'0'));
 repeater.key(Key::Back);CHECK(!repeater.active_radio_profile().offset_hz);
 repeater.key(Key::Ok);
 for(char d:"000600")if(d)repeater.digit(unsigned(d-'0'));
 repeater.key(Key::Ok);CHECK(repeater.config().vfo[0].tx_hz==145000000);
 repeater.key(Key::Down);repeater.key(Key::Ok);repeater.key(Key::Down);repeater.key(Key::Ok);
 CHECK(repeater.reverse_active(0) && repeater.config().vfo[0].rx_hz==145000000 && repeater.config().vfo[0].tx_hz==145600000);
 repeater.key(Key::Down);repeater.key(Key::Ok);repeater.key(Key::Down);repeater.key(Key::Ok);
 CHECK(repeater.active_radio_profile().tx_inhibit && !repeater.config().vfo[0].tx_hz && repeater.tx_status()[3]=='I');
 CHECK(repeater.store(63) && repeater.memory(63)->rx_hz==145600000 && repeater.memory(63)->tx_hz==145000000);
 repeater.key(Key::Back);repeater.key(Key::Back);repeater.key(Key::Back);repeater.key(Key::NextSlot);
 CHECK(!repeater.active_radio_profile().reverse && repeater.config().vfo[1].rx_hz==446018750);
 CHECK(repeater.recall(63) && repeater.config().vfo[1].rx_hz==145000000 && !repeater.config().vfo[1].tx_hz);
 repeater.key(Key::Mode);CHECK(repeater.config().vfo[1].rx_hz==446018750);
 repeater.key(Key::TxFrequency);repeater.key(Key::Left);repeater.key(Key::Ok);
 CHECK(repeater.config().vfo[1].rx_hz==repeater.config().vfo[1].tx_hz);
 repeater.key(Key::TxFrequency);repeater.key(Key::Right);repeater.key(Key::Back);
 CHECK(repeater.config().vfo[1].tx_hz==446018750);
 repeater.key(Key::TxFrequency);repeater.key(Key::Right);repeater.key(Key::Ok);
 CHECK(!repeater.config().vfo[1].tx_hz && repeater.config().vfo[1].rx_hz==446018750);
 Application zones;
 {
  Application actions;
  actions.key(Key::Channels);actions.key(Key::Down);actions.key(Key::Left);
  CHECK(actions.page()==Page::ChannelCommands && actions.channel_selection()==1);
  actions.key(Key::Ok);actions.digit(7);actions.key(Key::Ok);
  CHECK(actions.page()==Page::Channels && actions.memory(1)->name[0]=='7');
  actions.key(Key::Left);actions.key(Key::Down);actions.key(Key::Ok);
  CHECK(actions.page()==Page::Zones);
  actions.key(Key::Down);actions.key(Key::Ok);
  CHECK(actions.page()==Page::Channels && actions.memory(1)->zone==1 && !actions.channel_visible(1));
  actions.key(Key::Left);actions.key(Key::Up);actions.key(Key::Up);actions.key(Key::Ok);
  CHECK(actions.page()==Page::ConfirmDelete && actions.memory(0)->occupied);
  actions.key(Key::Back);CHECK(actions.page()==Page::Channels && actions.memory(0)->occupied);
  actions.key(Key::Left);actions.key(Key::Up);actions.key(Key::Up);actions.key(Key::Ok);actions.key(Key::Ok);
  CHECK(actions.page()==Page::Channels && !actions.memory(0)->occupied && actions.memory(1)->occupied);
  actions.key(Key::NextSlot);actions.key(Key::Down);actions.key(Key::Back);
  CHECK(actions.page()==Page::Channels && actions.active_zone()==0);
  actions.key(Key::NextSlot);actions.key(Key::Down);actions.key(Key::Ok);
  CHECK(actions.page()==Page::Channels && actions.active_zone()==1 && actions.channel_selection()==1);
  actions.key(Key::Left);actions.key(Key::Up);actions.key(Key::Ok);
  CHECK(actions.page()==Page::Radio && !actions.config().vfo[0].channel_mode &&
        actions.config().vfo[0].rx_hz==446018750 && !actions.config().vfo[0].tx_hz);
  CHECK(actions.set_frequency(145500000) && actions.memory(1)->rx_hz==446018750);
 }
 CHECK(zones.active_zone()==0 && zones.channel_visible(0));
 zones.key(Key::RenameChannel);zones.key(Key::Ok);zones.digit(7);zones.key(Key::Back);
 CHECK(zones.memory(0)->name[0]=='P');
 zones.key(Key::Ok);zones.digit(7);zones.key(Key::Ok);CHECK(zones.memory(0)->name[0]=='7');
 CHECK(zones.recall(0));
 zones.key(Key::DeleteChannel);zones.key(Key::Ok);zones.key(Key::Back);
 CHECK(zones.memory(0)->occupied && zones.config().vfo[0].channel_mode);
 zones.key(Key::MoveChannel);zones.key(Key::Ok);zones.key(Key::Down);zones.key(Key::Ok);
 CHECK(!zones.channel_visible(0) && zones.memory(0)->zone==1 && !zones.config().vfo[0].channel_mode);
 zones.key(Key::SelectZone);zones.key(Key::Down);zones.key(Key::Ok);
 CHECK(zones.active_zone()==1 && zones.channel_visible(0) && !zones.channel_visible(1));
 CHECK(!zones.recall(1) && zones.recall(0));
 zones.key(Key::RenameZone);zones.key(Key::Ok);zones.digit(8);zones.key(Key::Ok);
 CHECK(zones.zone_name(1)[0]=='8');
 zones.key(Key::DeleteChannel);zones.key(Key::Ok);zones.key(Key::Ok);
 CHECK(!zones.memory(0)->occupied && !zones.config().vfo[0].channel_mode);
 CHECK(zones.config().vfo[0].rx_hz==446006250 && !zones.config().vfo[0].tx_hz);
 zones.key(Key::SelectZone);zones.key(Key::Up);zones.key(Key::Ok);
 CHECK(zones.active_zone()==0 && zones.channel_visible(1));
 auto &snapshot=regression_fixture;
 Application saved;
 saved.set_frequency(435125000);
 saved.key(Key::RenameChannel);saved.key(Key::Ok);saved.digit(7);saved.key(Key::Ok);
 saved.key(Key::LanguageSelect);saved.key(Key::Down);saved.key(Key::Ok);
 CHECK(saved.set_aprs_frequency(144800000) && !saved.set_option(Option::Beacon,true));
 CHECK(saved.set_option(Option::Ble,true) && saved.set_option(Option::Export,true));
 saved.key(Key::WatchSlots);saved.key(Key::Down);saved.key(Key::Ok);
 saved.key(Key::WatchDwell);
 saved.digit(2);saved.digit(0);saved.digit(0);saved.key(Key::Ok);
 saved.key(Key::WatchHold);
 saved.digit(5);saved.digit(0);saved.digit(0);saved.key(Key::Ok);
 saved.key(Key::AprsScanInterval);
 saved.digit(4);saved.key(Key::Ok);
 CHECK(saved.page()==Page::AprsScanInterval && saved.config().aprs_scan_seconds==10);
 saved.key(Key::Erase);
 saved.digit(0);saved.key(Key::Ok);
 CHECK(saved.config().aprs_scan_seconds==0);
 CHECK(!saved.side_ptt_enabled() && saved.set_option(Option::SidePtt,true));
 CHECK(!saved.second_ptt_enabled() && saved.set_option(Option::SecondPtt,true));
 saved.key(Key::Channels);saved.key(Key::Right);saved.key(Key::Back);
 CHECK(!saved.scan_eligible(0) && saved.channel_visible(0));
 saved.key(Key::ScanMode);saved.key(Key::Down);saved.key(Key::Ok);
 saved.key(Key::ScanStart);
 for(char digit:"145500")if(digit)saved.digit(unsigned(digit-'0'));
 saved.key(Key::Ok);
 saved.key(Key::BeaconSymbol);saved.key(Key::Down);saved.key(Key::Ok);
 CHECK(saved.beacon_symbol_index()==1);
 saved.key(Key::ClearAprs);saved.key(Key::Back);
 CHECK(!saved.take_aprs_clear());
 saved.key(Key::ClearAprs);saved.key(Key::Ok);
 CHECK(saved.take_aprs_clear() && !saved.take_aprs_clear());
 CHECK(saved.set_option(Option::Tail,true));
 const size_t bytes=saved.save_snapshot(snapshot,sizeof snapshot);
 CHECK(bytes && !saved.save_snapshot(snapshot,0));
 Application loaded;
 CHECK(!loaded.restore_snapshot(snapshot,bytes-1) && loaded.language()==Language::English);
 CHECK(loaded.restore_snapshot(snapshot,bytes-460) && loaded.config().aprs_scan_seconds==10 && !loaded.side_ptt_enabled());
 CHECK(loaded.restore_snapshot(snapshot,bytes-432) && loaded.beacon_symbol_index()==0);
 CHECK(loaded.restore_snapshot(snapshot,bytes) && loaded.language()==Language::Russian);
 CHECK(loaded.beacon_symbol_index()==1 && loaded.enabled(Option::Tail));
 CHECK(loaded.config().aprs_scan_seconds==0);
 CHECK(loaded.side_ptt_enabled() && loaded.second_ptt_enabled());
 CHECK(!loaded.scan_eligible(0) && loaded.channel_visible(0));
 CHECK(loaded.scan_mode()==ScanMode::Search);
 CHECK(loaded.scan_bound(false)==145500000 && !loaded.scan_bound(true));
 CHECK(loaded.config().dwell_ms==200 && loaded.config().hold_ms==500);
 CHECK(loaded.config().watch_mode==RT_WATCH_MASK && loaded.config().watch_mask==3);
 CHECK(loaded.memory(0)->name[0]=='7' && loaded.config().vfo[0].rx_hz==435125000);
 CHECK(loaded.enabled(Option::Ble) && !loaded.enabled(Option::Export));
 CHECK(!loaded.config().beacon_on);
 CHECK(loaded.set_frequency(145500000) && loaded.config().vfo[0].tx_hz==145500000);
 CHECK(loaded.set_frequency(433500000) && loaded.config().vfo[0].tx_hz==433500000);
 snapshot[0]=9;
 CHECK(!loaded.restore_snapshot(snapshot,bytes) && loaded.language()==Language::Russian);
 Application a(false);
 enter(a,"12345");CHECK(a.editing());CHECK(a.config().vfo[0].rx_hz==0);CHECK(a.feedback()[0]);
 for(unsigned i=0;i<5;++i)dispatch_key(a,13);
 enter(a,"435125");CHECK(!a.editing());CHECK(a.config().vfo[0].rx_hz==435125000);
 dispatch_key(a,3);enter(a,"14550000");CHECK(a.config().vfo[1].rx_hz==145500000);
 dispatch_key(a,3);enter(a,"4335");CHECK(a.config().vfo[2].rx_hz==433500000);
 dispatch_key(a,7);CHECK(a.config().vfo[2].rx_hz==433512500);
 dispatch_key(a,8);CHECK(a.config().vfo[2].rx_hz==433500000);
 a.digit(4);a.digit(3);dispatch_key(a,6);CHECK(a.editing());CHECK(a.feedback()[0]);
 dispatch_key(a,134);CHECK(a.page()==Page::Settings);CHECK(a.editing());
 a.key(Key::Gps);a.key(Key::Back);CHECK(a.page()==Page::Settings);
 a.key(Key::Back);CHECK(a.page()==Page::Radio);CHECK(a.entered_khz()==43);
 dispatch_key(a,13);dispatch_key(a,13);CHECK(!a.editing());
 dispatch_key(a,2);CHECK(!a.config().vfo[2].channel_mode);CHECK(a.feedback()[0]);
 a.key(Key::Save);a.key(Key::Ok);dispatch_key(a,2);CHECK(a.config().vfo[2].channel_mode);
 dispatch_key(a,2);CHECK(!a.config().vfo[2].channel_mode);
 a.key(Key::Menu);a.key(Key::Debug);dispatch_key(a,9);CHECK(a.page()==Page::Debug);
 dispatch_key(a,137);CHECK(a.page()==Page::Settings);
 CHECK(a.power_save());a.key(Key::Back);a.key(Key::Menu);
 a.key(Key::Up);a.key(Key::Up);a.key(Key::Ok);
 CHECK(a.menu_group()==MenuGroup::System);a.key(Key::Ok);CHECK(!a.power_save());
 a.key(Key::Ok);CHECK(a.power_save());
 Controls c;CHECK(c.poll(6,0)==255);CHECK(c.poll(6,30)==6);
 CHECK(c.poll(6,729)==255);CHECK(c.poll(6,730)==134);CHECK(c.poll(6,2000)==255);
 char entry[16];a.key(Key::Back);a.key(Key::Back);a.digit(4);a.digit(3);a.digit(5);a.digit(1);a.entry_text(entry);
 CHECK(entry[0]=='4' && entry[3]=='.' && entry[4]=='1' && entry[5]=='_');
 // Notification expiry preserves the full editable input, including at wrap.
 Application timed(false);timed.tick(100);
 for(unsigned d=1;d<=8;++d)timed.digit(d);
 timed.digit(9);CHECK(timed.feedback()[0]);
 timed.tick(2099);CHECK(timed.feedback()[0]);timed.tick(2100);CHECK(!timed.feedback()[0]);
 CHECK(timed.editing() && timed.entered_khz()==12345678);
 dispatch_key(timed,6);CHECK(timed.feedback()[0]);timed.tick(4100);CHECK(!timed.feedback()[0]);
 CHECK(timed.entered_khz()==12345678);timed.digit(9);CHECK(timed.feedback()[0]);
 timed.tick(6100);CHECK(!timed.feedback()[0]);
 timed.tick(0xfffffff0u);dispatch_key(timed,6);timed.tick(1984);CHECK(!timed.feedback()[0]);
 Application channels(false);enter(channels,"435125");CHECK(channels.store(0));
 enter(channels,"145500");CHECK(channels.store(1));enter(channels,"433500");
 channels.key(Key::Mode);CHECK(channels.config().vfo[0].channel_mode);
 channels.key(Key::Up);CHECK(channels.config().vfo[0].memory_channel==1);
 channels.key(Key::Mode);CHECK(channels.config().vfo[0].rx_hz==433500000);
 channels.key(Key::Mode);CHECK(channels.config().vfo[0].memory_channel==1);
 channels.key(Key::Menu);channels.key(Key::Channels);CHECK(channels.page()==Page::Channels);
 channels.key(Key::Up);CHECK(channels.channel_selection()==0);
 channels.key(Key::Ok);CHECK(channels.page()==Page::Radio && channels.config().vfo[0].rx_hz==435125000);
 Application preset;
 for(unsigned i=0;i<64;++i) {
  const auto *m=preset.memory(i);CHECK(m->occupied && m->tx_hz==0);
  CHECK(m->radio.rx_tone.kind==ToneKind::Off && m->radio.tx_tone.kind==ToneKind::Off);
  CHECK(m->rx_hz==(i<16?446006250u+i*12500u:433075000u+(i-16)*25000u));
  CHECK(m->radio.narrow==(i<16));CHECK(m->name[0]==(i<16?'P':'L'));
 }
 dispatch_key(preset,2);CHECK(preset.config().vfo[0].channel_mode && preset.config().vfo[0].rx_hz==446006250);
 dispatch_key(preset,7);CHECK(preset.config().vfo[0].memory_channel==1);
 dispatch_key(preset,2);CHECK(!preset.config().vfo[0].channel_mode && preset.config().vfo[0].rx_hz==446006250 && !preset.config().vfo[0].tx_hz);
 enter(preset,"435125");preset.key(Key::Save);
 CHECK(preset.page()==Page::StoreChannel);CHECK(preset.memory(0)->rx_hz==446006250);
 preset.key(Key::Back);CHECK(preset.memory(0)->rx_hz==446006250);
 preset.key(Key::Save);preset.key(Key::Ok);
 CHECK(preset.page()==Page::ConfirmStore && !preset.voice_tx_page() && preset.memory(1)->rx_hz==446018750);
 preset.key(Key::Back);CHECK(preset.page()==Page::StoreChannel && preset.memory(1)->rx_hz==446018750);
 preset.key(Key::Ok);preset.key(Key::Ok);CHECK(preset.memory(1)->rx_hz==435125000);
 CHECK(preset.memory(1)->name[0]=='P');
 preset.key(Key::Mode);preset.digit(6);preset.digit(4);preset.key(Key::Ok);
 CHECK(preset.page()==Page::Radio && preset.config().vfo[0].channel_mode && preset.config().vfo[0].memory_channel==63);
 preset.digit(9);preset.digit(9);preset.key(Key::Ok);
 CHECK(preset.page()==Page::Channels && preset.config().vfo[0].memory_channel==63);
 preset.key(Key::Erase);preset.key(Key::Erase);preset.digit(2);preset.key(Key::Ok);
 CHECK(preset.page()==Page::Radio && preset.config().vfo[0].memory_channel==1);
 Application free_channels(false);
 CHECK(free_channels.set_frequency(435125000) && free_channels.store(0));
 CHECK(free_channels.set_frequency(435137500));
 free_channels.key(Key::Save);
 CHECK(free_channels.channel_selection()==1);
 free_channels.digit(6);free_channels.digit(4);free_channels.key(Key::Ok);
 CHECK(free_channels.page()==Page::Radio && free_channels.memory(63)->rx_hz==435137500);
 Application fresh;CHECK(fresh.memory(1)->rx_hz==446018750);
 WakeGuard wake;Controls wake_vm;wake.begin();
 CHECK(!wake.poll(2,0));wake_vm.poll(2,0);wake_vm.poll(2,30);
 CHECK(!wake.poll(255,100));wake_vm.poll(255,100);
 CHECK(!wake.poll(255,130));CHECK(wake_vm.poll(255,130)==2);
 CHECK(wake.poll(255,140));CHECK(wake.poll(2,150));
 Controls monitor_keys;
 CHECK(monitor_keys.poll(4,0)==255 && monitor_keys.poll(4,30)==4);
 CHECK(monitor_keys.poll(4,729)==255 && monitor_keys.poll(4,730)==255);
 CHECK(monitor_keys.poll(255,731)==255);
 Controls vm;
 CHECK(vm.poll(2,0)==255);CHECK(vm.poll(2,30)==255);CHECK(vm.poll(2,730)==130);
 CHECK(vm.poll(255,740)==255);CHECK(vm.poll(255,770)==255);
 Controls short_vm;short_vm.poll(2,0);CHECK(short_vm.poll(2,30)==255);short_vm.poll(255,100);CHECK(short_vm.poll(255,130)==2);
 Application sleep(false);sleep.activity(100);sleep.tick(60099);CHECK(sleep.screen_awake());sleep.tick(60100);CHECK(!sleep.screen_awake());sleep.activity(60101);CHECK(sleep.screen_awake());
 sleep.key(Key::ScreenTimer);sleep.digit(0);sleep.key(Key::Ok);CHECK(sleep.screen_seconds()==0);sleep.tick(1000000);CHECK(sleep.screen_awake());
 sleep.key(Key::ScreenTimer);sleep.digit(9);sleep.digit(9);sleep.digit(9);sleep.digit(9);sleep.key(Key::Ok);CHECK(sleep.page()==Page::ScreenTimer);CHECK(sleep.screen_seconds()==0);sleep.key(Key::Back);
 sleep.key(Key::ScreenTimer);sleep.digit(6);sleep.digit(1);sleep.key(Key::Erase);sleep.digit(0);sleep.key(Key::Ok);CHECK(sleep.screen_seconds()==60);
 sleep.key(Key::Brightness);sleep.key(Key::Down);CHECK(sleep.brightness()==3);sleep.key(Key::Ok);
 Application lcd(false);lcd.key(Key::Menu);lcd.key(Key::Up);lcd.key(Key::Ok);lcd.key(Key::Down);lcd.key(Key::Ok);
 CHECK(lcd.page()==Page::LcdTest && !lcd.lcd_test_gray());
 lcd.key(Key::Ok);CHECK(lcd.lcd_test_gray());
 lcd.key(Key::Down);CHECK(lcd.brightness()==1);
 lcd.key(Key::Up);CHECK(lcd.brightness()==4);
 lcd.key(Key::Back);CHECK(lcd.page()==Page::Settings);
 lcd.key(Key::Ok);CHECK(lcd.page()==Page::LcdTest && !lcd.lcd_test_gray());
 Application aprs(false);aprs.key(Key::Menu);
 for(unsigned i=0;i<3;++i)aprs.key(Key::Down);
 aprs.key(Key::Ok);CHECK(aprs.menu_group()==MenuGroup::Aprs);
 CHECK(aprs.enabled(Option::AprsRx) && !aprs.enabled(Option::Beacon) && !aprs.enabled(Option::Export));
 CHECK(aprs.menu_entry(4).toggle==Option::AprsRx && aprs.menu_entry(5).toggle==Option::Beacon &&
       aprs.menu_entry(6).toggle==Option::Export);
 for(unsigned i=0;i<3;++i)aprs.key(Key::Down);
 aprs.key(Key::Ok);CHECK(aprs.menu_group()==MenuGroup::Beacon);
 aprs.key(Key::Down);aprs.key(Key::Ok);CHECK(aprs.page()==Page::BeaconInterval);
 aprs.digit(9);aprs.key(Key::Ok);CHECK(aprs.page()==Page::BeaconInterval);
 aprs.key(Key::Back);CHECK(aprs.menu_selection()==1 && aprs.config().beacon_seconds==60);
 aprs.key(Key::Ok);aprs.digit(1);aprs.digit(2);aprs.digit(0);aprs.key(Key::Ok);
 CHECK(aprs.page()==Page::Settings && aprs.config().beacon_seconds==120);
 aprs.key(Key::Back);CHECK(aprs.menu_group()==MenuGroup::Aprs && aprs.menu_selection()==3);
 aprs.key(Key::Back);CHECK(aprs.menu_group()==MenuGroup::Root && aprs.menu_selection()==3);
 aprs.key(Key::Ok);aprs.key(Key::Ok);aprs.key(Key::Down);aprs.key(Key::Ok);
 CHECK(aprs.page()==Page::BeaconFrequency);
 for(char d:"144800"){if(d)aprs.digit(unsigned(d-'0'));}
 aprs.key(Key::Ok);CHECK(aprs.config().aprs_hz==144800000 && aprs.config().aprs_rx_hz==144800000);
 aprs.key(Key::Back);aprs.key(Key::Up);aprs.key(Key::Ok);aprs.key(Key::Down);aprs.key(Key::Ok);
 CHECK(aprs.page()==Page::AprsRxFrequency);
 for(char d:"000000"){if(d)aprs.digit(unsigned(d-'0'));}
 aprs.key(Key::Ok);CHECK(aprs.page()==Page::AprsRxFrequency && aprs.config().aprs_rx_hz==144800000);
 aprs.key(Key::Back);aprs.key(Key::Ok);
 for(char d:"14501250"){if(d)aprs.digit(unsigned(d-'0'));}
 aprs.key(Key::Ok);CHECK(aprs.config().aprs_rx_hz==145012500 && aprs.config().aprs_hz==144800000);
 Application edit(false);
 edit.key(Key::Menu);edit.key(Key::BeaconIdentity);edit.key(Key::Ok);
 CHECK(edit.page()==Page::BeaconIdentity);
 edit.key(Key::Up);dispatch_key(edit,5);edit.digit(1);
 dispatch_key(edit,3);edit.key(Key::Down);edit.key(Key::Ok);
 CHECK(edit.beacon_identity().callsign[0]=='A' && edit.beacon_identity().callsign[1]=='1');
 CHECK(edit.beacon_identity().ssid==15);
 edit.key(Key::BeaconIdentity);edit.key(Key::Erase);edit.key(Key::Back);
 CHECK(edit.beacon_identity().callsign[0]=='A');
 edit.key(Key::CoordinateFormat);edit.key(Key::Up);edit.key(Key::Back);
 CHECK(!edit.decimal_coordinates());
 edit.key(Key::CoordinateFormat);edit.key(Key::Down);edit.key(Key::Ok);
 CHECK(edit.decimal_coordinates());
 char coord[20];gps_coordinate("5545.0000",2,true,coord);
 CHECK(coord[0]=='5' && coord[1]=='5' && coord[2]=='.' && coord[3]=='7' && coord[4]=='5' && !coord[9]);
 gps_coordinate("03700.0001",3,true,coord);
 CHECK(coord[0]=='0' && coord[1]=='3' && coord[2]=='7' && coord[3]=='.' && coord[9]=='1' && !coord[10]);
 CHECK(gps_speed_tenths(1000,SpeedUnit::Kmh)==185);
 CHECK(gps_speed_tenths(1000,SpeedUnit::Mph)==115);
 CHECK(gps_speed_tenths(1000,SpeedUnit::Knots)==100);
 edit.key(Key::SpeedUnits);edit.key(Key::Up);edit.key(Key::Back);
 CHECK(edit.speed_unit()==SpeedUnit::Kmh);
 edit.key(Key::SpeedUnits);edit.key(Key::Up);edit.key(Key::Ok);
 CHECK(edit.speed_unit()==SpeedUnit::Mph);
 edit.key(Key::SpeedUnits);edit.key(Key::Up);edit.key(Key::Ok);
 CHECK(edit.speed_unit()==SpeedUnit::Knots);
 edit.key(Key::AltitudeUnits);edit.key(Key::Up);edit.key(Key::Back);
 CHECK(!edit.altitude_feet());
 edit.key(Key::AltitudeUnits);edit.key(Key::Up);edit.key(Key::Ok);
 CHECK(edit.altitude_feet());
 nav_reset();
 nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,-12.3,M,46.9,M,,",100);
 CHECK(gps_altitude_fresh(*nav_get(),100) && nav_get()->altitude_decimetres==-123);
 CHECK(gps_altitude_display(-123,false)==-12 && gps_altitude_display(-123,true)==-40);
 CHECK(!gps_altitude_fresh(*nav_get(),3100));
 nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,12.3,F,46.9,M,,",200);
 CHECK(!gps_altitude_fresh(*nav_get(),200));
 Application gps(false);const auto vfo=gps.config().vfo[0];dispatch_key(gps,130);CHECK(gps.page()==Page::Gps && gps.config().vfo[0].channel_mode==vfo.channel_mode);
 gps.key(Key::Menu);gps.key(Key::Gps);dispatch_key(gps,130);gps.key(Key::Back);CHECK(gps.page()==Page::Settings);
 nav_reset();nmea("GPRMC,231500.00,A,5545.0000,N,03737.0000,E,2.50,90.0,021126,,,A",1000);
 CHECK(nav_fix_fresh(1000));CHECK(nav_get()->rmc_count==1);CHECK(nav_get()->course_valid && nav_get()->course_deci==900);CHECK(nav_get()->speed_centi_knots==250);
 char local[12],date[12];CHECK(gps_local_time(*nav_get(),180,1000,local,date));CHECK(local[0]=='0' && local[1]=='2' && date[8]=='0' && date[9]=='3');
 CHECK(!gps_local_time(*nav_get(),180,4000,local,date));CHECK(!nav_fix_fresh(4000));
 nmea("GNRMC,001500.00,A,5545.0000,N,03737.0000,E,0.00,90.0,010126,,,A",5000);
 CHECK(!nav_get()->course_valid && nav_get()->speed_valid && !nav_get()->speed_centi_knots);CHECK(gps_local_time(*nav_get(),-60,5000,local,date));CHECK(local[0]=='2' && local[1]=='3' && date[2]=='2' && date[3]=='5' && date[5]=='1' && date[6]=='2' && date[8]=='3' && date[9]=='1');
 nmea("GPRMC,001500.00,V,,,,,,,010126,,,N",6000);CHECK(!nav_fix_fresh(6000));CHECK(nav_get()->ever_fix);CHECK(!nav_get()->time_valid);
 nmea("GPRMC,001500.00,A,5545.0000,N,03737.0000,E,1.00,0.0,290226,,,A",7000);CHECK(!nav_get()->time_valid);
 nav_reset();CHECK(!nav_fix_fresh(0));
 return 0;
}

extern "C" uint32_t voice_tx_mic_probe(uint32_t old,uint32_t source,uint32_t gain) {
 return rt950::voice::microphone_word(uint16_t(old),uint8_t(source),uint8_t(gain));
}
extern "C" uint32_t voice_tx_freq_probe(uint32_t word,int32_t correction) {
 return rt950::voice::corrected_frequency_word(word,int16_t(correction));
}
extern "C" uint32_t voice_tx_pa_probe(uint32_t hz) {
 return rt950::voice::uhf_pa_index(hz);
}

extern "C" uint32_t voice_tx_route_probe(unsigned mode) {
 struct Pins {
  uint32_t trace=0;
  void set(rt950::voice::TxRoutePin pin,bool high) {
   trace=(trace<<4)|(unsigned(pin)<<1)|unsigned(high);
  }
 } pins;
 if(mode==0)rt950::voice::prepare_uhf_route(pins);
 else rt950::voice::select_uhf_tx_route(pins);
 return pins.trace;
}

// Keep the Thumb entry prologue together across Unicorn translation pages.
extern "C" [[gnu::aligned(16)]] uint32_t voice_tx_register_probe(uint32_t word,uint32_t profile,uint32_t *trace) {
 struct Radio {
  uint32_t *trace;
  unsigned count=0;
  bool write(uint8_t reg,uint16_t value){trace[count++]=(uint32_t(reg)<<16)|value;return true;}
  bool read(uint8_t reg,uint16_t &value) {
   if(reg==0x52)value=0x6fff;
   else if(reg==0x31)value=0x33;
   else if(reg==0x40)value=0xa800;
   else return false;
   return true;
  }
  void delay_us(unsigned us){trace[count++]=0xff0000|us;}
 } radio{trace};
 const rt950::voice::UhfTxValues values{uint16_t(word),uint16_t(word>>16),0xe937,1};
 if(!rt950::voice::prepare_voice_tx(radio,values,uint8_t(profile>>8)) ||
    !rt950::voice::finish_voice_tx(radio,uint8_t(profile)))return 0;
 return radio.count;
}

extern "C" uint32_t voice_board_tx_probe(uint32_t hz,uint32_t tone) {
 using namespace rt950::voice;
 BoardReceiver receiver;
 receiver.enable(true);
 for(uint32_t now=0;now<46;++now)receiver.poll(now,false);
 if(!receiver.status().ready)return 1;
 UhfCalibration factory;
 factory.pa[2][4]=136;factory.microphone=55;factory.ctcss_level=47;factory.dcs_level=75;factory.frequency_correction=-31;
 VhfPaCalibration vhf;vhf.pa[2][5]=98;
 CbPaCalibration cb;cb.pa[2][2]=73;
 BoardUhfTx adapter(receiver,factory,2,nullptr,&vhf,&cb,(tone&0x10000u)!=0);
 const unsigned microphone_mode=(tone>>24)&3u;
 if(microphone_mode)adapter.select_microphone(static_cast<rt950::MicGain>(microphone_mode-1));
 const unsigned power_mode=(tone>>20)&15u;
 if(power_mode)adapter.select_measured_bias(power_mode==3?hz+12500u:hz,power_mode==2?137:power_mode==4?0:42);
 const rt950::Tone selected{static_cast<rt950::ToneKind>((tone>>8)&0xffu),uint8_t(tone)};
 adapter.select_tone(selected);
 if(!adapter.permits_tone(selected))return 4;
 UhfTransmitter<BoardUhfTx> transmitter(adapter);
 Ptt ptt;
 const bool tail=(tone&0x20000u)!=0;
 const TxRequest request{hz,15,1,true,true,tail};
 ptt.poll(transmitter,request,100,false);
 ptt.poll(transmitter,request,101,true);
 ptt.poll(transmitter,request,102,true);
 if(cb_fm_frequency(hz)) {
  for(uint32_t now=103;now<=183 && ptt.state()==Ptt::State::Preparing;++now)
   ptt.poll(transmitter,request,now,true);
 } else ptt.poll(transmitter,request,146,true);
 if(ptt.state()!=Ptt::State::Transmitting)return 2;
 const auto active_microphone=adapter.microphone_gain();
 adapter.select_microphone(active_microphone==rt950::MicGain::Low?rt950::MicGain::High:rt950::MicGain::Low);
 if(adapter.microphone_gain()!=active_microphone)return 8;
 ptt.poll(transmitter,request,184,false);
 if(tail) {
  if(ptt.state()!=Ptt::State::Ending)return 6;
  ptt.poll(transmitter,request,583,false);
  if(ptt.state()!=Ptt::State::Ending)return 7;
  ptt.poll(transmitter,request,584,false);
 }
 if(cb_fm_frequency(hz)) {
  receiver.tune(hz,185);
  for(uint32_t now=185;now<=265;++now)receiver.poll(now,false);
  if(receiver.status().frequency!=hz || !receiver.status().receiving)return 5;
 }
 return ptt.state()==Ptt::State::Idle && !adapter.healthy()?0:3;
}

extern "C" uint32_t voice_factory_selector_probe(unsigned level) {
 return rt950::voice::factory_power_selector(static_cast<rt950::TxPower>(level));
}

extern "C" uint32_t voice_vhf_pa_probe(uint32_t hz,uint32_t selector) {
 rt950::voice::VhfPaCalibration cal;
 const auto read=[](uint32_t address,uint8_t *data,size_t length) {
  if(address==0xf0e0 && length==1) {*data=1;return true;}
  if(length!=16 || (address!=0xf010 && address!=0xf050 && address!=0xf090))return false;
  for(unsigned i=0;i<length;++i)data[i]=uint8_t(1+(address-0xf010)/4+i);
  return true;
 };
 if(!rt950::voice::load_vhf_pa_calibration(read,cal) || selector>=3)return 0;
 return cal.bias(hz,uint8_t(selector));
}

extern "C" uint32_t voice_ctcss_tx_probe(uint32_t tone,uint32_t level,uint32_t *trace) {
 struct Radio {
  uint32_t *trace;
  unsigned count=0;
  bool write(uint8_t reg,uint16_t value){trace[count++]=(uint32_t(reg)<<16)|value;return true;}
 } radio{trace};
 return rt950::voice::configure_ctcss(radio,uint16_t(tone),uint8_t(level),level&0x100)?radio.count:0;
}

extern "C" uint32_t voice_cb_pa_probe(uint32_t hz,uint32_t selector) {
 rt950::voice::CbPaCalibration cal;
 const auto read=[](uint32_t address,uint8_t *data,size_t length) {
  if(address==0xf0e0 && length==1) {*data=1;return true;}
  if(length!=16 || (address!=0xf190 && address!=0xf1a0 && address!=0xf1b0))return false;
  for(unsigned i=0;i<length;++i)data[i]=uint8_t(1+address-0xf190+i);
  return true;
 };
 if(!rt950::voice::load_cb_pa_calibration(read,cal) || selector>=3)return 0;
 return cal.bias(hz,uint8_t(selector));
}

extern "C" uint32_t voice_dcs_tx_probe(uint32_t code,uint32_t level,uint32_t *trace) {
 struct Radio {
  uint32_t *trace;
  unsigned count=0;
  bool write(uint8_t reg,uint16_t value){trace[count++]=(uint32_t(reg)<<16)|value;return true;}
 } radio{trace};
 return rt950::voice::configure_dcs(radio,uint16_t(code),uint8_t(level),level&0x100)?radio.count:0;
}

extern "C" uint32_t voice_board_rx_tone_probe(uint32_t packed) {
 using namespace rt950;
 voice::BoardReceiver rx;
 rx.enable(true);
 for(uint32_t now=0;now<46;++now)rx.poll(now,false);
 if(!rx.status().ready)return 1;
 const Tone tone{static_cast<ToneKind>((packed>>8)&0xffu),uint8_t(packed)};
 rx.set_profile(tone,47,100,(packed&0x10000u)==0);
 rx.tune(446081250,100);
 const bool packet=(packed&0x20000u)!=0;
 rx.poll(101,false,false,packet);
 for(uint32_t now=131;now<=161;now+=15) {
  rx.poll(now,false,false,packet);
  if(!rx.carrier() || rx.status().receiving)return 2;
 }
 for(uint32_t now=176;now<=206;now+=15) {
  rx.poll(now,false,false,packet);
  if(rx.status().receiving!=(now==206))return 3;
 }
 rx.set_profile(tone,47,207);
 rx.tune(446093750,208);
 return rx.status().receiving || rx.carrier()?4:0;
}

extern "C" uint32_t voice_board_monitor_probe() {
 using namespace rt950;
 voice::BoardReceiver rx;rx.enable(true);
 for(uint32_t now=0;now<46;++now)rx.poll(now,false);
 if(!rx.status().ready)return 1;
 rx.set_profile({ToneKind::Ctcss,9},47,100,true,9);
 rx.tune(145500000,100);
 rx.poll(101,false,true);
 if(rx.has_sample())return 2;
 rx.poll(131,false,true);
 if(!rx.has_sample() || rx.status().receiving || rx.carrier())return 3;
 rx.poll(132,true,true); // PTT mutes even with Monitor selected.
 rx.poll(133,false,true);
 rx.poll(134,false,false);
 rx.tune(145512500,135);
 return rx.has_sample() || rx.status().receiving?4:0;
}

extern "C" uint32_t voice_board_sql_probe() {
 using namespace rt950::voice;
 BoardReceiver rx;
 rx.enable(true);
 for(uint32_t now=0;now<46;++now)rx.poll(now,false);
 if(!rx.status().ready)return 1;
 rx.set_profile({},0,100,true,9);
 rx.tune(145500000,100);
 rx.poll(101,false);
 for(unsigned i=0;i<4;++i) {
  rx.poll(131+i*15,false);
  if(rx.status().receiving!=(i==1 || i==2))return 2+i;
 }
 rx.set_profile({},0,177,true,0);
 if(rx.status().receiving || rx.has_sample() || rx.squelch_open(177))return 7;
 rx.poll(177,false);
 if(!rx.has_sample())return 8;
 return rx.status().receiving || rx.carrier()?6:0;
}

extern "C" uint32_t voice_board_cb_rx_probe(uint32_t hz) {
 using namespace rt950;
 Application app;
 const uint32_t expected_tx=voice::cb_fm_frequency(hz)?hz:0;
 if(!app.set_frequency(hz) || app.config().vfo[0].tx_hz!=expected_tx ||
    !rt_config_valid(&app.config()) || app.set_aprs_frequency(hz))return 1;
 if(!expected_tx && (!app.store(0) || app.memory(0)->tx_hz))return 5;
 voice::BoardReceiver rx;
 rx.enable(true);
 rx.tune(app.config().vfo[0].rx_hz,100);
 for(uint32_t now=100;now<=180;++now)rx.poll(now,false);
 if(rx.status().frequency!=hz || !rx.status().receiving)return 2;
 rx.tune(145500000,181);
 if(rx.status().ready || rx.status().receiving)return 3;
 for(uint32_t now=181;now<=260;++now)rx.poll(now,false);
 return rx.status().frequency==145500000 && rx.status().receiving?0:4;
}

extern "C" void aprs_afsk_probe(uint32_t levels,uint16_t *samples,unsigned symbols) {
 rt950::aprs::AfskWave wave;
 for(unsigned bit=0;bit<symbols;++bit)
  for(unsigned sample=0;sample<wave.SamplesPerSymbol;++sample)
   *samples++=wave.sample((levels>>bit)&1u);
}

extern "C" uint32_t aprs_dac_probe(unsigned mode) {
 using namespace rt950::aprs;
 const uint8_t packet[]={0x82,0xa0,0xa4,0xa6,0x40,0x40,0x60,0x9c,0x60,0x86,0x82,0x98,0x98,0x61,3,0xf0,'>'};
 FrameTransmitter frame;
 if(!frame.encode(packet,sizeof packet,2))return 1;
 DacBurst burst;
 const bool pb8_high=mode==2;
 GPIOB->ODR=pb8_high?GPIO_PIN_8:0;
 DAC->CR=0x00010000u;
 DMA2_CH(3)->CCR=mode==3?1:0;
 if(mode==3)return burst.begin(frame,0) || DMA2_CH(3)->CCR!=1?2:0;
 if(!burst.begin(frame,0))return 3;
 if(TIM6->PSC!=3 || TIM6->ARR!=781 || TIM6->CR2!=0x20 || TIM6->CR1!=1 ||
    DMA2_CH(3)->CPAR!=0x40007408 || DMA2_CH(3)->CNDTR!=2048 ||
    DAC->CR!=0x00011007u || !(GPIOB->ODR&GPIO_PIN_8))return 4;
 const auto *samples=reinterpret_cast<const uint16_t *>(DMA2_CH(3)->CMAR);
 AfskWave expected;
 for(unsigned i=0;i<2048;++i)
  if(samples[i]!=expected.sample(!frame.level(i/32)))return 5;
 DAC->SR=0; // Emulate write-one-to-clear status on the register model.
 DMA2->ISR=0;
 if(mode==1) {
  if(burst.poll(40)!=DacBurst::State::Fault)return 6;
 } else if(mode==2)burst.cancel();
 else if(mode==4) {
  DMA2->ISR=DMA_ISR_HTIF(3)|DMA_ISR_TCIF(3);
  if(burst.poll(30)!=DacBurst::State::Fault)return 7;
 } else {
  auto state=DacBurst::State::Running;
  for(unsigned i=0;i<12 && state==DacBurst::State::Running;++i) {
   DMA2->ISR=i%2?DMA_ISR_TCIF(3):DMA_ISR_HTIF(3);
   state=burst.poll((i+1)*27);
  }
  if(state!=DacBurst::State::Complete)return 8;
 }
 if(((GPIOB->ODR&GPIO_PIN_8)!=0)!=pb8_high)return 10;
 return TIM6->CR1 || DMA2_CH(3)->CCR || DAC->CR!=0x00010000u?9:0;
}

extern "C" uint32_t aprs_adc_probe(unsigned mode) {
 rt950::aprs::AdcCapture capture;
 constexpr uint32_t original_pin=0x44444444;
 GPIOA->CRL=original_pin;
 ADC1->CR2=0;
 ADC2->CR2=0x123;
 TIM2->CR1=TIM2->DIER=0;
 DMA1_CH(1)->CCR=0;
 capture.poll(true,0,nullptr,nullptr);
 capture.poll(true,1,nullptr,nullptr);
 if(!(ADC1->CR2&ADC_CR2_RSTCAL))return 1;
 if(mode==1)capture.poll(true,22,nullptr,nullptr);
 else {
  ADC1->CR2&=~ADC_CR2_RSTCAL;
  capture.poll(true,2,nullptr,nullptr);
  if(!(ADC1->CR2&ADC_CR2_CAL))return 2;
  ADC1->CR2&=~ADC_CR2_CAL;
  capture.poll(true,3,nullptr,nullptr);
  if(!capture.running() || TIM2->ARR!=12500 || TIM2->CCR2!=255 ||
     ADC1->SQR3!=2 || DMA1_CH(1)->CPAR!=0x4001244c || DMA1_CH(1)->CNDTR!=2048)
   return 3;
  if(mode==0) {
   DMA1->ISR=DMA_ISR_HTIF(1)|DMA_ISR_TCIF(1);
   capture.poll(true,112,nullptr,nullptr);
  } else {
   rt950::aprs::FrameTransmitter tx;
   const char call[7]="N0CALL";
   if(!rt950::aprs::encode_beacon(tx,call,9,"5545.0000",'N',"03737.0000",'E'))return 5;
   auto *buffer=reinterpret_cast<volatile uint16_t *>(DMA1_CH(1)->CMAR);
   rt950::aprs::AfskWave wave;
   AprsCapture decoded;
   for(unsigned block=0;block<6;++block) {
    for(unsigned i=0;i<1024;++i) {
     const unsigned symbol=(block*1024+i)/8;
     uint16_t value=2048;
     if(symbol<tx.size()+3)for(unsigned n=0;n<4;++n)value=wave.sample(symbol<tx.size() && !tx.level(symbol));
     const int noise=int((symbol*17+((i%8)*4+3)*7)%31)-15;
     buffer[(block%2)*1024+i]=uint16_t(1470+(int(value)-2048)/2+noise);
    }
    DMA1->ISR=block%2?DMA_ISR_TCIF(1):DMA_ISR_HTIF(1);
    capture.poll(true,111+block*107,capture_aprs,&decoded);
   }
   if(!capture.running())return 6;
   if(decoded.count!=1 || decoded.length!=39)return 7;
   capture.poll(false,760,nullptr,nullptr);
  }
 }
 return capture.running() || TIM2->CR1 || ADC1->CR2 || DMA1_CH(1)->CCR ||
     GPIOA->CRL!=original_pin || ADC2->CR2!=0x123?4:0;
}

extern "C" uint32_t aprs_ble_probe() {
 rt950::aprs::BleExport ble;
 const uint8_t frame[]={0x82,0xa0,0xa4,0xa6,0x40,0x40,0x60,0x9c,0x60,0x86,0x82,0x98,0x98,0x61,3,0xf0,0xc0,0xdb};
 uint8_t expected[48]{};
 const size_t length=rt_kiss_encode(0,frame,sizeof frame,expected,sizeof expected);
 if(ble.enqueue(frame,sizeof frame,0))return 1;
 ble.enable(true,0);
 if(USART1->BRR!=6250 || USART1->CR1!=(USART_CR1_UE|USART_CR1_TE))return 2;
 if(!ble.enqueue(frame,sizeof frame,1) || ble.enqueue(frame,sizeof frame,2))return 3;
 USART1->DR=0xff;USART1->SR=USART_SR_TXE;
 ble.poll(true,99);
 if(USART1->DR!=0xff)return 4;
 for(size_t i=0;i<length;++i) {
  ble.poll(true,100+uint32_t(i));
  if(USART1->DR!=expected[i])return 5;
 }
 if(!ble.enqueue(frame,sizeof frame,130))return 6;
 USART1->SR=0;
 ble.poll(true,380);
 if(!ble.enqueue(frame,sizeof frame,381))return 7;
 ble.poll(false,382);
 USART1->DR=0xff;USART1->SR=USART_SR_TXE;
 ble.poll(true,383);
 if(USART1->DR!=0xff)return 8;
 using Telemetry=rt950::aprs::GpsTelemetry;
 nav_status_t nav{};
 if(Telemetry(nav,false,false,390).bytes[4]!=0 ||
    Telemetry(nav,true,false,390).bytes[4]!=1)return 10;
 nav.last_byte_ms=390;
 if(Telemetry(nav,true,true,391).bytes[4]!=2)return 11;
 nav.position_valid=nav.ever_fix=1;nav.last_fix_ms=390;
 const char lat[]="5545.00",lon[]="03737.00";
 for(unsigned i=0;i<sizeof lat;++i)nav.lat[i]=lat[i];
 for(unsigned i=0;i<sizeof lon;++i)nav.lon[i]=lon[i];
 nav.ns[0]='N';nav.ew[0]='W';
 const Telemetry gps(nav,true,true,392);
 auto word=[&gps](unsigned offset) {
  uint32_t result=0;
  for(unsigned i=0;i<4;++i)result|=uint32_t(gps.bytes[offset+i])<<(8*i);
  return result;
 };
 if(gps.bytes[4]!=3 || gps.bytes[5]!=1 || word(6)!=5575000 ||
    word(10)!=uint32_t(-3761667))return 12;
 const Telemetry stale(nav,true,true,3390);
 if(stale.bytes[4]!=4 || stale.bytes[5]!=0)return 13;
 for(unsigned i=6;i<14;++i)if(stale.bytes[i])return 14;
 if(!ble.enqueue_gps(gps,392) || !ble.enqueue(frame,sizeof frame,393))return 15;
 for(size_t i=0;i<length;++i)ble.poll(true,394+uint32_t(i));
 if(!ble.enqueue_gps(gps,450))return 16;
 ble.poll(true,451);
 if(USART1->DR!=0xc0 || ble.enqueue(frame,sizeof frame,452))return 17;
 ble.poll(true,453);
 if(USART1->DR!=Telemetry::Type)return 18;
 ble.enable(false,454);
 return USART1->CR1 || ble.enqueue(frame,sizeof frame,455)?9:0;
}



extern "C" uint32_t voice_tail_probe(uint32_t kind,uint32_t level,uint32_t *trace) {
 struct Radio {
  uint32_t *trace;unsigned count=0;
  bool read(uint8_t reg,uint16_t &value){if(reg!=0x52)return false;value=0x6fff;return true;}
  bool write(uint8_t reg,uint16_t value){trace[count++]=(uint32_t(reg)<<16)|value;return true;}
 } radio{trace};
 if(!rt950::voice::prepare_tail_tx(radio,{static_cast<rt950::ToneKind>(kind),1},rt950::voice::factory_tone_level(uint8_t(level))))return 0;
 return radio.count;
}

extern "C" int monitor_settings_tests() {
 Application app;
 CHECK(!app.monitor_enabled() && !app.monitor_active());
 app.key(Key::Monitor);CHECK(app.page()==Page::Monitor && !app.monitor_draft());
 app.key(Key::Down);CHECK(app.monitor_draft() && !app.monitor_active());
 app.key(Key::Back);CHECK(!app.monitor_enabled() && app.page()==Page::Settings);
 app.key(Key::Monitor);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.page()==Page::Radio && app.monitor_enabled() && app.monitor_active() && app.voice_tx_page());
 CHECK(app.set_option(Option::Watch,true));
 voice::Watch watch;VoiceReceiver receiver;
 watch.poll(receiver,app.config(),0,true,false);
 watch.poll(receiver,app.config(),app.config().dwell_ms+1u,true,false);
 CHECK(watch.slot()!=app.config().active);
 watch.poll(receiver,app.config(),1000,true,app.monitor_active());
 CHECK(watch.slot()==app.config().active && receiver.frequency==app.config().vfo[0].rx_hz);
 watch.poll(receiver,app.config(),5000,true,app.monitor_active());
 CHECK(watch.slot()==app.config().active);
 const auto slot=app.config().active;const auto mode=app.config().watch_mode;
 const auto frequency=app.config().vfo[slot].rx_hz;
 app.key(Key::VoiceRx);CHECK(!app.monitor_active() && !app.voice_tx_page());
 const Key ignored[]={Key::Right,Key::Left,Key::Up,Key::Down,Key::Ok,Key::NextSlot,Key::Mode};
 for(Key k:ignored)app.key(k);
 for(unsigned code=0;code<255;++code)if(code!=9 && code!=1)dispatch_key(app,uint8_t(code));
 for(unsigned d=0;d<10;++d)app.digit(d);
 CHECK(app.page()==Page::VoiceRx && app.config().active==slot && app.config().watch_mode==mode);
 CHECK(app.config().vfo[slot].rx_hz==frequency && app.monitor_enabled() && !app.monitor_active());
 dispatch_key(app,9);CHECK(app.page()==Page::Settings);
 app.key(Key::Back);CHECK(app.page()==Page::Radio && app.monitor_active());
 app.key(Key::KeypadLock);CHECK(app.keypad_locked() && app.monitor_active());
 app.key(Key::Monitor);CHECK(app.page()==Page::Radio && app.monitor_active());
 app.key(Key::KeypadLock);
 uint8_t data[Application::SnapshotCapacity];const size_t length=app.save_snapshot(data,sizeof data);
 Application restored(false);CHECK(length && restored.restore_snapshot(data,length) && !restored.monitor_enabled());
 app.key(Key::Monitor);app.key(Key::Down);app.tick(30000);
 CHECK(app.page()==Page::Radio && app.monitor_enabled());
 app.key(Key::Monitor);app.key(Key::Down);app.key(Key::Ok);
 CHECK(!app.monitor_active() && !app.monitor_enabled());
 app.key(Key::Menu);app.key(Key::Ok);app.key(Key::Up);app.key(Key::Up);app.key(Key::Ok);
 CHECK(app.page()==Page::Monitor);
 return 0;
}

extern "C" int keypad_lock_tests() {
 Application app(false);
 CHECK(!app.keypad_locked() && app.voice_tx_page());
 const uint32_t frequency=app.config().vfo[0].rx_hz;
 const uint8_t slot=app.config().active;
 Controls hash;
 CHECK(hash.poll(21,0)==255 && hash.poll(21,30)==255);
 CHECK(hash.poll(21,729)==255);
 CHECK(hash.poll(21,730)==149);dispatch_key(app,149);
 CHECK(app.keypad_locked() && app.page()==Page::Radio && app.voice_tx_page());
 CHECK(hash.poll(21,2000)==255 && hash.poll(255,2010)==255 && hash.poll(255,2040)==255);
 for(unsigned key=0;key<unsigned(Key::KeypadLock);++key)app.key(static_cast<Key>(key));
 for(unsigned code=0;code<256;++code)if(code!=149)dispatch_key(app,uint8_t(code));
 for(unsigned d=0;d<10;++d)app.digit(d);
 CHECK(app.keypad_locked() && !app.editing() && app.config().active==slot);
 CHECK(app.page()==Page::Radio && app.config().vfo[slot].rx_hz==frequency);
 app.tick(61000);CHECK(!app.screen_awake() && app.keypad_locked());
 app.activity(61001);CHECK(app.screen_awake() && app.keypad_locked());
 uint8_t data[Application::SnapshotCapacity];
 const size_t length=app.save_snapshot(data,sizeof data);
 Application restored(false);
 CHECK(length && restored.restore_snapshot(data,length) && !restored.keypad_locked());
 CHECK(restored.config().vfo[slot].rx_hz==frequency);
 dispatch_key(app,149);CHECK(!app.keypad_locked());
 // Holding # during frequency entry must neither commit nor erase the draft.
 constexpr unsigned entered_digits[]={4,3,5,1,2,5};
 for(unsigned d:entered_digits)app.digit(d);
 const auto digits=app.editor_digits();
 dispatch_key(app,149);CHECK(!app.keypad_locked() && app.editor_digits()==digits);
 CHECK(app.config().vfo[slot].rx_hz==frequency);
 Controls short_hash;
 CHECK(short_hash.poll(21,3000)==255 && short_hash.poll(21,3030)==255);
 CHECK(short_hash.poll(255,3100)==255 && short_hash.poll(255,3130)==21);
 dispatch_key(app,21);CHECK(app.config().vfo[slot].rx_hz==435125000 && !app.editing());
 // System menu exposes the same operation, without reassigning another key.
 app.key(Key::Menu);app.key(Key::Up);app.key(Key::Up);app.key(Key::Ok);
 CHECK(app.menu_group()==MenuGroup::System);
 app.key(Key::Up);app.key(Key::Ok);CHECK(app.keypad_locked() && app.page()==Page::Radio);
 dispatch_key(app,149);CHECK(!app.keypad_locked());
 Controls wrap;
 CHECK(wrap.poll(21,0xffffffe0u)==255 && wrap.poll(21,0xfffffffeu)==255);
 CHECK(wrap.poll(21,697)==255 && wrap.poll(21,698)==149);
 return 0;
}

extern "C" int mic_gain_tests() {
 Application app(false);
 CHECK(app.mic_gain()==MicGain::High);
 const uint32_t frequency=app.config().vfo[0].rx_hz;
 app.key(Key::MicGain);CHECK(app.page()==Page::MicGain);
 app.key(Key::Down);CHECK(app.mic_gain_draft()==MicGain::Low);
 CHECK(app.mic_gain()==MicGain::High);
 app.key(Key::Back);CHECK(app.page()==Page::Settings && app.mic_gain()==MicGain::High);
 app.key(Key::MicGain);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.mic_gain()==MicGain::Low && app.page()==Page::Settings);
 app.key(Key::MicGain);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.mic_gain()==MicGain::Middle);
 app.key(Key::MicGain);app.key(Key::Up);app.tick(30000);
 CHECK(app.page()==Page::Radio && app.mic_gain()==MicGain::Middle);
 uint8_t data[Application::SnapshotCapacity];
 const size_t length=app.save_snapshot(data,sizeof data);
 CHECK(length && app.config().vfo[0].rx_hz==frequency);
 Application restored(false);
 CHECK(restored.restore_snapshot(data,length) && restored.mic_gain()==MicGain::Middle);
 CHECK(restored.restore_snapshot(data,length-100) && restored.mic_gain()==MicGain::High);
 data[length-100]=3;
 CHECK(!restored.restore_snapshot(data,length) && restored.mic_gain()==MicGain::High);
 return 0;
}

extern "C" int menu_timeout_tests() {
 // Menu idle exit cancels pending changes, preserves applied settings and
 // leaves continuous receiver/status pages alone. Timers use unsigned ticks.
 Application menu_idle(false);
 CHECK(menu_idle.menu_timeout_selection()==2);
 menu_idle.tick(100);menu_idle.key(Key::Menu);menu_idle.key(Key::MenuTimeout);
 menu_idle.key(Key::Up);menu_idle.key(Key::Ok); // 10 seconds
 menu_idle.key(Key::TxFrequency);menu_idle.digit(4);menu_idle.digit(3);
 menu_idle.tick(10099);CHECK(menu_idle.page()==Page::TxFrequency);
 menu_idle.tick(10100);CHECK(menu_idle.page()==Page::Radio);
 CHECK(!menu_idle.config().vfo[0].tx_hz);
 menu_idle.key(Key::TxFrequency);CHECK(!menu_idle.editor_digits());menu_idle.key(Key::Back);
 menu_idle.key(Key::RadioOptions);menu_idle.key(Key::Down);
 menu_idle.tick(20100);CHECK(menu_idle.page()==Page::Radio && menu_idle.active_radio_profile().squelch==3);
 menu_idle.key(Key::ClearAprs);menu_idle.tick(30100);
 CHECK(menu_idle.page()==Page::Radio && !menu_idle.take_aprs_clear());
 menu_idle.key(Key::Menu);menu_idle.tick(35000);menu_idle.key(Key::Down);
 menu_idle.tick(44999);CHECK(menu_idle.page()==Page::Settings);
 menu_idle.tick(45000);CHECK(menu_idle.page()==Page::Radio);
 menu_idle.key(Key::Menu);menu_idle.activity(50000);menu_idle.tick(59999);
 CHECK(menu_idle.page()==Page::Settings);menu_idle.tick(60000);CHECK(menu_idle.page()==Page::Radio);
 menu_idle.key(Key::MenuTimeout);menu_idle.key(Key::Down);menu_idle.key(Key::Back);
 CHECK(menu_idle.page()==Page::Settings && menu_idle.menu_timeout_selection()==1);
 menu_idle.tick(0xfffffff0u);menu_idle.key(Key::Menu);
 menu_idle.tick(9983);CHECK(menu_idle.page()==Page::Settings);
 menu_idle.tick(9984);CHECK(menu_idle.page()==Page::Radio);
 constexpr Key status_pages[]={Key::Gps,Key::Aprs,Key::Debug,Key::Battery,Key::LcdTest,Key::FlashInfo,Key::VoiceRx,Key::FmRadio,Key::ChannelScan,Key::VfoScan};
 for(Key status:status_pages) {
  menu_idle.key(status);const Page page=menu_idle.page();
  menu_idle.tick(20000u+uint32_t(status)*20000u);CHECK(menu_idle.page()==page);
  menu_idle.key(Key::Back);
 }
 menu_idle.key(Key::MenuTimeout);menu_idle.key(Key::Up);menu_idle.key(Key::Ok); // OFF
 menu_idle.key(Key::Menu);menu_idle.tick(4000000);CHECK(menu_idle.page()==Page::Settings);
 uint8_t menu_snapshot[Application::SnapshotCapacity];
 const size_t menu_bytes=menu_idle.save_snapshot(menu_snapshot,sizeof menu_snapshot);
 Application menu_restore(false);
 CHECK(menu_bytes && menu_restore.restore_snapshot(menu_snapshot,menu_bytes) && !menu_restore.menu_timeout_selection());
 CHECK(menu_restore.restore_snapshot(menu_snapshot,menu_bytes-424) && menu_restore.menu_timeout_selection()==2);
 CHECK(menu_restore.enabled(Option::Tail));
 CHECK(menu_idle.set_option(Option::Tail,false));
 CHECK(menu_idle.save_snapshot(menu_snapshot,sizeof menu_snapshot)==menu_bytes);
 CHECK(menu_restore.restore_snapshot(menu_snapshot,menu_bytes) && !menu_restore.enabled(Option::Tail));
 CHECK(menu_restore.restore_snapshot(menu_snapshot,menu_bytes-428) && menu_restore.enabled(Option::Tail));
 menu_snapshot[menu_bytes-424]=5;
 CHECK(!menu_restore.restore_snapshot(menu_snapshot,menu_bytes) && menu_restore.menu_timeout_selection()==2);
 menu_idle.key(Key::MenuTimeout);menu_idle.key(Key::Up);menu_idle.key(Key::Ok);
 CHECK(menu_idle.menu_timeout_selection()==4); // wrap OFF -> 120 seconds
 return 0;
}

extern "C" int measured_power_tests() {
 MeasuredPower table;
 RadioProfile profile;profile.power=TxPower::Pmr500mW;
 constexpr uint32_t pmr=446081250, lpd=433075000;
 constexpr uint32_t pmr_ref=446000000, lpd_ref=433000000;
 CHECK(target_mw(profile.power)==500 && target_mw(TxPower::Lpd10mW)==10);
 CHECK(power_channel(profile.power,pmr) && !power_channel(profile.power,pmr+10));
 CHECK(!power_channel(TxPower::Lpd10mW,pmr) && power_channel(TxPower::Lpd10mW,lpd));
 CHECK(!resolve_target_power(table,profile,pmr,pmr,8000,95,false,1).ready);
 CHECK(!resolve_target_power(table,profile,pmr,pmr,8000,95,true,1).ready);
 CHECK(resolve_target_power(table,profile,pmr_ref,pmr_ref,8000,95,true,1).ready);
 CHECK(!resolve_target_power(table,profile,pmr_ref,pmr_ref,8000,95,true,96).ready);
 CHECK(!resolve_target_power(table,profile,pmr_ref,pmr_ref,8000,0,true,1).ready);
 CHECK(table.store({pmr,8000,480,42,TxPower::Pmr500mW}));
 CHECK(resolve_target_power(table,profile,pmr,pmr,7900,95,false,1).code==42);
 CHECK(!resolve_target_power(table,profile,pmr,pmr,7899,95,false,1).ready);
 CHECK(!resolve_target_power(table,profile,pmr,pmr+12500,8000,95,false,1).ready);
 profile.narrow=false;CHECK(!resolve_target_power(table,profile,pmr,pmr,8000,95,false,1).ready);
 profile.narrow=true;profile.reverse=true;
 CHECK(!resolve_target_power(table,profile,pmr,pmr,8000,95,false,1).ready);
 profile.reverse=false;profile.shift=Shift::Plus;
 CHECK(!resolve_target_power(table,profile,pmr,pmr,8000,95,false,1).ready);
 CHECK(!table.store({pmr,8000,501,42,TxPower::Pmr500mW}));
 CHECK(!table.store({lpd,8000,11,42,TxPower::Lpd10mW}));
 CHECK(!table.store({lpd,8000,10,0,TxPower::Lpd10mW}));
 CHECK(table.store({pmr,8050,490,41,TxPower::Pmr500mW}) && table.count()==1);
 for(unsigned i=0;i<15;++i)CHECK(table.store({lpd+25000*i,8000,9,10,TxPower::Lpd10mW}));
 CHECK(table.count()==16 && !table.store({lpd+25000*15,8000,9,10,TxPower::Lpd10mW}));
 CHECK(table.find(TxPower::Pmr500mW,pmr,8050)->code==41);
 CHECK(!table.find(TxPower::Pmr500mW,pmr+12500,8050));
 profile.shift=Shift::None;
 MeasuredPower reference;
 CHECK(reference.store({pmr_ref,8000,480,42,TxPower::Pmr500mW}));
 CHECK(reference.store({lpd_ref,8000,9,12,TxPower::Lpd10mW}));
 CHECK(reference.find(TxPower::Lpd10mW,434750000,8000)->code==12);
 CHECK(resolve_target_power(reference,profile,446006250,446006250,8000,95,false,0).code==42);
 CHECK(resolve_target_power(reference,profile,446193750,446193750,8000,95,false,0).ready);
 CHECK(!resolve_target_power(reference,profile,pmr,pmr,8000,41,false,0).ready);
 CHECK(!resolve_target_power(reference,profile,pmr,pmr,8101,95,false,0).ready);
 CHECK(!resolve_target_power(reference,profile,pmr_ref,pmr_ref,8000,95,false,0).ready);
 Application app(false);CHECK(app.set_frequency(pmr));
 app.key(Key::Menu);CHECK(app.page()==Page::Settings);
 app.key(Key::Ok);CHECK(app.menu_group()==MenuGroup::Radio);
 for(unsigned i=0;i<app.menu_count() && app.menu_entry(app.menu_selection()).radio_setting!=RadioSetting::Power;++i)app.key(Key::Down);
 app.key(Key::Ok);
 CHECK(app.page()==Page::RadioOptions && app.radio_setting()==RadioSetting::Power);
 CHECK(radio_option_count(RadioSetting::Power)==5 && app.radio_draft()==0);
 for(unsigned i=0;i<4;++i)app.key(Key::Down);
 CHECK(app.radio_draft()==4);
 app.key(Key::Ok);CHECK(app.active_radio_profile().power==TxPower::Pmr500mW);
 app.power_battery(8000);app.power_code_limit(95);app.key(Key::PowerCalibration);
 CHECK(app.page()!=Page::PowerCalibration);
 CHECK(app.set_frequency(pmr_ref));app.key(Key::PowerCalibration);
 CHECK(app.page()==Page::PowerCalibration && !app.power_test(0) && app.power_code()==1);
 app.digit(4);app.digit(2);app.key(Key::Ok);CHECK(app.power_code()==42 && !app.power_test(0));
 app.key(Key::Ok);CHECK(app.power_test(0) && !app.power_test(1));
 app.key(Key::Up);CHECK(!app.power_test(0) && app.power_code()==43);
 app.key(Key::Ok);CHECK(app.power_test(0));app.key(Key::Right);
 CHECK(app.page()==Page::PowerMeasured && !app.power_test(0));
 app.key(Key::Ok);CHECK(app.page()==Page::PowerMeasured && !app.measured_power().count());
 app.digit(0);app.key(Key::Ok);CHECK(app.page()==Page::PowerMeasured && !app.measured_power().count());
 app.key(Key::Erase);
 app.digit(5);app.digit(0);app.digit(1);app.key(Key::Ok);
 CHECK(app.page()==Page::PowerMeasured && !app.measured_power().count());
 app.key(Key::Erase);app.digit(0);app.key(Key::Ok);
 CHECK(app.page()==Page::PowerCalibration && app.measured_power().count()==1);
 CHECK(app.measured_power().find(TxPower::Pmr500mW,pmr,8000)->code==43);
 app.key(Key::Ok);CHECK(app.power_test(0));
 uint8_t bytes[Application::SnapshotCapacity];
 const size_t length=app.save_snapshot(bytes,sizeof bytes);CHECK(length);
 Application restored(false);CHECK(restored.restore_snapshot(bytes,length));
 CHECK(restored.measured_power().count()==1 && !restored.power_test(0));
 CHECK(restored.active_radio_profile().power==TxPower::Pmr500mW);
 CHECK(restored.restore_snapshot(bytes,length-420) && !restored.measured_power().count());
 // Corrupt the first measurement's code and ensure atomic rejection.
 bytes[length-420+12]=255;
 CHECK(!restored.restore_snapshot(bytes,length) && !restored.measured_power().count());
 app.tick(30000);CHECK(app.page()==Page::Radio && !app.power_test(0));
 app.key(Key::PowerCalibration);CHECK(app.page()==Page::PowerCalibration && !app.power_test(0));
 app.digit(9);app.digit(6);app.key(Key::Ok);CHECK(!app.power_test(0) && app.power_code()==43);
 app.key(Key::Back);CHECK(app.page()==Page::Settings && !app.power_test(0));
 app.key(Key::PowerCalibration);app.key(Key::Left);CHECK(app.page()==Page::PowerClear);
 app.key(Key::Back);CHECK(app.measured_power().count()==1);
 app.key(Key::Left);app.key(Key::Ok);CHECK(!app.measured_power().count() && !app.power_test(0));
 return 0;
}

extern "C" int band_profile_tests() {
 CHECK(preset_channels(TxPower::Lpd10mW)==68 && preset_channels(TxPower::Pmr500mW)==8);
 CHECK(preset_frequency(TxPower::Lpd10mW,1)==433075000u);
 CHECK(preset_frequency(TxPower::Lpd10mW,68)==434750000u);
 CHECK(preset_frequency(TxPower::Pmr500mW,7)==446081250u);
 CHECK(preset_frequency(TxPower::Pmr500mW,8)==446093750u);
 CHECK(!preset_frequency(TxPower::Pmr500mW,9) && !preset_frequency(TxPower::Lpd10mW,69));
 CHECK(!preset_frequency(TxPower::Low,1) && !preset_frequency(TxPower::Pmr500mW,0));
 for(unsigned channel=1;channel<=68;++channel) {
  const uint32_t hz=preset_frequency(TxPower::Lpd10mW,channel);
  CHECK(preset_channel(TxPower::Lpd10mW,hz)==channel && power_channel(TxPower::Lpd10mW,hz));
  CHECK(!preset_channel(TxPower::Lpd10mW,hz+10));
 }
 for(unsigned channel=1;channel<=8;++channel)CHECK(preset_channel(TxPower::Pmr500mW,preset_frequency(TxPower::Pmr500mW,channel))==channel);
 Application app;const uint32_t a=app.config().vfo[0].rx_hz,b=app.config().vfo[1].rx_hz,c=app.config().vfo[2].rx_hz;
 CHECK(app.recall(0));const uint32_t original=app.memory(0)->rx_hz;
 app.key(Key::BandProfile);CHECK(app.page()==Page::BandProfile);
 app.digit(7);CHECK(app.band_frequency()==446081250u && app.config().vfo[0].rx_hz==a);
 app.key(Key::Back);CHECK(app.config().vfo[0].channel_mode);
 app.key(Key::BandProfile);app.digit(9);app.key(Key::Ok);
 CHECK(app.page()==Page::BandProfile && app.config().vfo[0].rx_hz==a);
 app.key(Key::Erase);app.digit(7);app.key(Key::Ok);
 CHECK(app.page()==Page::Radio && app.config().vfo[0].rx_hz==446081250u && app.config().vfo[0].tx_hz==446081250u);
 CHECK(!app.config().vfo[0].channel_mode && app.memory(0)->rx_hz==original && !app.memory(0)->tx_hz);
 CHECK(app.active_radio_profile().power==TxPower::Pmr500mW && app.active_radio_profile().narrow);
 CHECK(app.config().vfo[1].rx_hz==b && app.config().vfo[2].rx_hz==c);
 CHECK(!resolve_target_power(app.measured_power(),app.active_radio_profile(),446081250,446081250,8000,95,false,0).ready);
 app.key(Key::BandProfile);app.key(Key::Right);CHECK(app.band_power()==TxPower::Lpd10mW);
 app.digit(6);app.digit(8);CHECK(app.band_frequency()==434750000u);
 app.key(Key::Ok);CHECK(app.config().vfo[0].rx_hz==434750000u && app.active_radio_profile().power==TxPower::Lpd10mW);
 app.key(Key::BandProfile);CHECK(app.band_channel()==68);
 app.key(Key::Down);CHECK(app.band_channel()==1);app.key(Key::Up);CHECK(app.band_channel()==68);
 app.key(Key::Left);CHECK(app.band_power()==TxPower::Pmr500mW && app.band_channel()==1);
 app.key(Key::Back);CHECK(app.config().vfo[0].rx_hz==434750000u);
 uint8_t snapshot[Application::SnapshotCapacity];const size_t bytes=app.save_snapshot(snapshot,sizeof snapshot);
 Application loaded;CHECK(bytes && loaded.restore_snapshot(snapshot,bytes));
 CHECK(loaded.config().vfo[0].rx_hz==434750000u && loaded.active_radio_profile().power==TxPower::Lpd10mW);
 CHECK(!loaded.measured_power().count());
 app.key(Key::BandProfile);app.tick(30000);CHECK(app.page()==Page::Radio);
 return 0;
}

extern "C" int power_diagnostics_tests() {
 constexpr voice::BatteryTxLimits limits{};
 CHECK(classify_supply(true,128,limits.low,limits.High)==SupplyFault::Low);
 CHECK(classify_supply(true,187,limits.low,limits.High)==SupplyFault::None);
 CHECK(classify_supply(true,188,limits.low,limits.High)==SupplyFault::High);
 CHECK(classify_supply(false,160,limits.low,limits.High)==SupplyFault::Sensor);
 constexpr SupplyFault faults[]={SupplyFault::Low,SupplyFault::High,SupplyFault::Sensor};
 for(auto fault:faults) {
  TxBackend backend;voice::Ptt ptt;
  voice::TxRequest request{446081250,15,0,true,false,true};
  ptt.poll(backend,request,0,false);
  ptt.poll(backend,request,1,true);
  CHECK(ptt.state()==voice::Ptt::State::Transmitting);
  const bool fresh=fault!=SupplyFault::Sensor;
  const uint8_t raw=fault==SupplyFault::Low?128:fault==SupplyFault::High?188:160;
  request.authorized=classify_supply(fresh,raw,limits.low,limits.High)==SupplyFault::None;
  ptt.poll(backend,request,22,true);
  CHECK(ptt.state()==voice::Ptt::State::WaitRelease && backend.stops==1 && !backend.tails);
  request.authorized=true;ptt.poll(backend,request,23,true);CHECK(backend.keys==1);
 }
 TxSupplyTrace trace;using Owner=TxSupplyTrace::Owner;
 CHECK(!trace.has_session() && !trace.samples());
 trace.observe(100,false,0,Owner::Voice,0,8000,20,true);
 trace.observe(200,true,446081250,Owner::Voice,42,8000,120,true);
 CHECK(trace.has_session() && trace.before()==8000 && !trace.samples() && !trace.sag_valid());
 trace.observe(250,true,446081250,Owner::Voice,42,8000,170,true);
 CHECK(!trace.samples());
 trace.observe(300,true,446081250,Owner::Voice,42,7800,10,true);
 CHECK(trace.samples()==1 && trace.minimum()==7800 && trace.maximum()==7800 && trace.sag()==200);
 trace.observe(310,true,446081250,Owner::Voice,42,7800,20,true);CHECK(trace.samples()==1);
 trace.observe(400,true,446081250,Owner::Voice,42,7600,0,true);
 CHECK(trace.samples()==2 && trace.minimum()==7600 && trace.maximum()==7800 && trace.sag()==400);
 trace.observe(500,true,446081250,Owner::Voice,42,0,0xffffffffu,false);
 CHECK(trace.samples()==2 && trace.minimum()==7600);
 trace.trip(SupplyFault::Low);trace.trip(SupplyFault::Sensor);
 trace.observe(600,false,0,Owner::Voice,0,7900,0,true);
 CHECK(trace.frequency()==446081250 && trace.code()==42 && trace.samples()==2 && trace.fault()==SupplyFault::Low);
 trace.observe(700,true,144800000,Owner::Aprs,20,7900,100,true);
 CHECK(trace.owner()==Owner::Aprs && trace.before()==7900 && !trace.samples() && trace.fault()==SupplyFault::None);
 trace.observe(701,true,144800000,Owner::Aprs,20,7800,0,true);CHECK(trace.samples()==1);
 trace.observe(702,true,145500000,Owner::Voice,10,7800,1,true);
 CHECK(!trace.before() && !trace.samples() && trace.frequency()==145500000);
 trace.observe(0xfffffff0u,false,0,Owner::Voice,0,8100,0,true);
 trace.observe(0xfffffff8u,true,446081250,Owner::Voice,35,8100,8,true);
 trace.observe(5,true,446081250,Owner::Voice,35,7700,0,true);
 CHECK(trace.before()==8100 && trace.samples()==1 && trace.sag()==400);
 trace.observe(100,false,0,Owner::Voice,0,8200,2000,false);
 trace.observe(200,true,446081250,Owner::Voice,35,8200,2100,false);
 CHECK(!trace.before() && !trace.samples());
 WorkloadTime workload;
 workload.observe(0xfffffff0u,1u<<WorkloadTime::Gps);
 workload.observe(0x10u,(1u<<WorkloadTime::Gps)|(1u<<WorkloadTime::WatchRx));
 workload.observe(0x10u+2990u,1u<<WorkloadTime::Tx);
 CHECK(workload.seconds(WorkloadTime::Gps)==3 && workload.seconds(WorkloadTime::WatchRx)==2);
 CHECK(!workload.seconds(WorkloadTime::Tx));
 Application app;
 app.key(Key::PowerStatus);CHECK(app.page()==Page::PowerStatus && !app.power_status_page());
 CHECK(!app.voice_tx_page() && !app.voice_tx_page(true));
 app.tick(100000);CHECK(app.page()==Page::PowerStatus);
 app.key(Key::Down);CHECK(app.power_status_page()==1);
 app.key(Key::Down);CHECK(app.power_status_page()==2);
 app.key(Key::Down);CHECK(app.power_status_page()==3);
 app.key(Key::Down);CHECK(app.power_status_page()==0);
 app.key(Key::Up);CHECK(app.power_status_page()==3);
 const auto hz=app.config().vfo[0].rx_hz;
 app.digit(4);app.key(Key::Ok);CHECK(app.page()==Page::PowerStatus && app.config().vfo[0].rx_hz==hz);
 app.key(Key::Back);CHECK(app.page()==Page::Settings);
 return 0;
}

extern "C" int mcu_diagnostics_probe() {
 McuTemperature sensor;
 ADC1->CR2=ADC_CR2_ADON|ADC_CR2_DMA;
 CRM->APB2EN=CRM_APB2EN_ADC1EN|CRM_APB2EN_ADC2EN;
 sensor.poll(true,0);
 CHECK(ADC1->CR2==(ADC_CR2_ADON|ADC_CR2_DMA));
 ADC1->CR2=0;ADC1->SMPR1=2;ADC1->SQR3=2;
 CRM->APB2EN=CRM_APB2EN_ADC2EN;ADC2->CR2=0x123;
 sensor.poll(false,1);sensor.poll(true,2);sensor.poll(true,3);
 CHECK(ADC1->CR2&ADC_CR2_RSTCAL);
 ADC1->CR2&=~ADC_CR2_RSTCAL;sensor.poll(true,4);
 CHECK(ADC1->CR2&ADC_CR2_CAL);
 ADC1->CR2&=~ADC_CR2_CAL;sensor.poll(true,5);
 CHECK(ADC1->SQR3==16 && (ADC1->CR2&(1u<<23)));
 ADC1->DR=1536;ADC1->SR=ADC_SR_EOC;sensor.poll(true,6);
 CHECK(ADC1->SQR3==17);
 ADC1->DR=1440;ADC1->SR=ADC_SR_EOC;sensor.poll(true,7);
 CHECK(sensor.reading(7)==25 && sensor.reading(6007)==McuTemperature::Unavailable);
 CHECK(ADC1->CR2==0 && ADC1->SMPR1==2 && ADC1->SQR3==2 &&
       ADC2->CR2==0x123 && CRM->APB2EN==CRM_APB2EN_ADC2EN);
 sensor.poll(true,2002);sensor.poll(false,2003);
 CHECK(ADC1->CR2==0 && CRM->APB2EN==CRM_APB2EN_ADC2EN);
 sensor.poll(true,2004);sensor.poll(true,2024);
 CHECK(ADC1->CR2==0 && sensor.reading(2024)==McuTemperature::Unavailable);
 return 0;
}

extern "C" int auto_lock_tests() {
 Application app(false);
 CHECK(!app.auto_lock_selection());
 app.tick(100000);CHECK(!app.keypad_locked());
 app.key(Key::AutoLock);app.key(Key::Up);
 CHECK(app.auto_lock_draft()==4 && !app.auto_lock_selection());
 app.key(Key::Back);CHECK(app.page()==Page::Settings && !app.auto_lock_selection());
 app.key(Key::AutoLock);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.auto_lock_selection()==1 && app.page()==Page::Settings);
 app.key(Key::Back);app.tick(109999);CHECK(!app.keypad_locked());
 app.tick(110000);CHECK(app.keypad_locked() && app.voice_tx_page());
 app.key(Key::NextSlot);app.digit(4);CHECK(app.keypad_locked() && !app.editing());
 dispatch_key(app,149);CHECK(!app.keypad_locked());
 app.tick(119999);CHECK(!app.keypad_locked());
 app.tick(120000);CHECK(app.keypad_locked());
 app.key(Key::KeypadLock);
 // Activity from encoder or held PTT restarts inactivity; receiving does not.
 app.activity(129999);app.tick(139998);CHECK(!app.keypad_locked());
 app.activity(139999);app.tick(139999);CHECK(!app.keypad_locked());
 app.tick(149999);CHECK(app.keypad_locked());app.key(Key::KeypadLock);
 app.digit(4);app.tick(200000);CHECK(!app.keypad_locked() && app.editing());
 app.key(Key::Back);app.tick(209999);CHECK(!app.keypad_locked());
 app.tick(210000);CHECK(app.keypad_locked());app.key(Key::KeypadLock);
 app.key(Key::Gps);app.tick(300000);CHECK(app.page()==Page::Gps && !app.keypad_locked());
 app.key(Key::Back);app.key(Key::Menu);app.tick(330000);
 CHECK(app.page()==Page::Radio && !app.keypad_locked());
 app.tick(339999);CHECK(!app.keypad_locked());app.tick(340000);CHECK(app.keypad_locked());
 // Wrap-safe timer and fresh unlock grace period.
 app.tick(0xfffffff0u);app.key(Key::KeypadLock);
 app.tick(9983);CHECK(!app.keypad_locked());app.tick(9984);CHECK(app.keypad_locked());
 uint8_t data[Application::SnapshotCapacity];const size_t length=app.save_snapshot(data,sizeof data);
 Application restored(false);
 CHECK(length && restored.restore_snapshot(data,length));
 CHECK(restored.auto_lock_selection()==1 && !restored.keypad_locked());
 // v0.48 restores with default OFF; earlier extensions still migrate.
 CHECK(restored.restore_snapshot(data,length-96) && !restored.auto_lock_selection());
 data[length-96]=5;CHECK(!restored.restore_snapshot(data,length) && !restored.auto_lock_selection());
 data[length-96]=1;data[length-95]=1;CHECK(!restored.restore_snapshot(data,length));
 app.key(Key::KeypadLock);app.key(Key::AutoLock);app.key(Key::Up);app.key(Key::Ok);
 CHECK(!app.auto_lock_selection());app.key(Key::Back);app.tick(1000000);CHECK(!app.keypad_locked());
 // Menu idle cancellation cannot apply the auto-lock draft.
 app.key(Key::AutoLock);app.key(Key::Down);app.tick(1030000);
 CHECK(app.page()==Page::Radio && !app.auto_lock_selection());
 return 0;
}

extern "C" int broadcast_bank_tests() {
 Application app(false);
 const auto active=app.config().active;
 const auto voice_hz=app.config().vfo[active].rx_hz;
 auto tune=[&](Key mode,unsigned value) {
  if(app.page()==Page::FmProbe)app.key(Key::Back);
  app.key(mode);
  unsigned digits[5],count=0;
  do{digits[count++]=value%10;value/=10;}while(value);
  while(count)app.digit(digits[--count]);
  app.key(Key::Ok);
 };
 for(unsigned am=0;am<2;++am) {
  for(unsigned i=0;i<Application::BroadcastCount;++i) {
   const unsigned hz=am?2300+i*3900:6500+i*600;
   tune(am?Key::SwAm:Key::FmRadio,hz);CHECK(app.fm_request()==hz);
   app.key(Key::Menu);CHECK(app.page()==(am?Page::AmPresets:Page::FmPresets));
   CHECK(!app.voice_tx_page());
   app.digit(i+1);CHECK(app.broadcast_selection()==i && !app.broadcast_frequency(i));
   app.key(Key::Left);CHECK(app.broadcast_confirm() && !app.broadcast_deleting());
   app.key(Key::Back);CHECK(!app.broadcast_confirm() && !app.broadcast_frequency(i));
   app.key(Key::Left);app.key(Key::Ok);CHECK(app.broadcast_frequency(i)==hz);
   app.key(Key::Erase);CHECK(app.broadcast_deleting());app.digit(1);
   CHECK(app.broadcast_selection()==i);app.key(Key::Back);CHECK(app.broadcast_frequency(i)==hz);
   app.key(Key::Ok);CHECK(app.page()==Page::FmProbe && app.fm_audio() && app.sw_am()==bool(am));
   CHECK(app.fm_request()==hz && !app.fm_seek() && !*app.fm_entry());
  }
 }
 CHECK(app.config().active==active && app.config().vfo[active].rx_hz==voice_hz);
 uint8_t data[Application::SnapshotCapacity];const size_t length=app.save_snapshot(data,sizeof data);
 CHECK(length && length<=Application::SnapshotCapacity);
 Application restored(false);CHECK(restored.restore_snapshot(data,length));
 for(unsigned am=0;am<2;++am) {
  restored.key(am?Key::AmPresets:Key::FmPresets);
  for(unsigned i=0;i<Application::BroadcastCount;++i)CHECK(restored.broadcast_frequency(i)==(am?2300+i*3900:6500+i*600));
 }
 // Drop the new banks: v0.49 and earlier snapshots migrate the legacy FM slot.
 CHECK(restored.restore_snapshot(data,length-92));restored.key(Key::FmPresets);
 CHECK(restored.broadcast_frequency(0)==6500 && !restored.broadcast_frequency(1));
 restored.key(Key::AmPresets);for(unsigned i=0;i<Application::BroadcastCount;++i)CHECK(!restored.broadcast_frequency(i));
 CHECK(!restored.restore_snapshot(data,length-4));
 // Corrupt the first FM word: reject range errors and narrowing before assignment.
 const size_t first=length-92;
 data[first]=0x63;data[first+1]=0x19;CHECK(!restored.restore_snapshot(data,length)); // 6499
 data[first]=0x64;data[first+1]=0x19;data[first+2]=1;CHECK(!restored.restore_snapshot(data,length));
 data[first+2]=0;CHECK(restored.restore_snapshot(data,length));
 const size_t first_am=length-60;data[first_am]=0x31;data[first_am+1]=0x75; // 30001
 CHECK(!restored.restore_snapshot(data,length));
 app.key(Key::Back);app.key(Key::AmPresets);app.digit(8);
 app.key(Key::Erase);app.key(Key::Ok);CHECK(!app.broadcast_frequency(7));
 app.key(Key::Ok);CHECK(app.page()==Page::AmPresets && !app.broadcast_confirm());
 app.key(Key::Up);CHECK(app.broadcast_selection()==6);app.digit(1);app.key(Key::Up);CHECK(app.broadcast_selection()==7);
 app.key(Key::Down);CHECK(!app.broadcast_selection());
 // Leaving a bank opened from the player resumes the same mode; menu timeout never writes.
 app.key(Key::Back);app.key(Key::SwAm);app.key(Key::Menu);CHECK(app.page()==Page::AmPresets);
 app.key(Key::Left);app.tick(30000);CHECK(app.page()==Page::Radio);
 app.key(Key::AmPresets);CHECK(app.broadcast_frequency(0)==2300 && !app.broadcast_confirm());
 app.key(Key::Back);app.key(Key::FmRadio);app.key(Key::Menu);app.key(Key::Back);
 CHECK(app.page()==Page::FmProbe && !app.sw_am());
 return 0;
}

extern "C" int broadcast_player_tests() {
 Application fm_app;
 dispatch_key(fm_app,1);
 CHECK(fm_app.page()==Page::FmProbe && fm_app.fm_audio());
 const unsigned fm_digits[]={9,9,5,0};
 for(unsigned digit:fm_digits)fm_app.digit(digit);
 fm_app.key(Key::Ok);
 CHECK(fm_app.fm_request()==9950);
 fm_app.key(Key::NextSlot);
 fm_app.key(Key::Up);
 CHECK(fm_app.fm_request()==9960);
 fm_app.key(Key::Left);
 CHECK(fm_app.fm_request()==9950);
 fm_app.key(Key::Right);
 CHECK(fm_app.fm_seek());
 fm_app.fm_seek_result(10350);
 CHECK(!fm_app.fm_seek() && fm_app.fm_request()==10350);
 dispatch_key(fm_app,1);
 CHECK(fm_app.page()==Page::Radio);
 fm_app.key(Key::FmProbe);
 CHECK(fm_app.page()==Page::FmProbe && !fm_app.fm_audio());
 auto &fm_snapshot=regression_fixture;
 const size_t fm_size=fm_app.save_snapshot(fm_snapshot,sizeof fm_snapshot);
 CHECK(fm_size>4);
 Application old_fm;
 CHECK(old_fm.restore_snapshot(fm_snapshot,fm_size-464));
 old_fm.key(Key::FmRadio);old_fm.key(Key::Left);
 CHECK(old_fm.fm_request()==10030);
 CHECK(old_fm.restore_snapshot(fm_snapshot,fm_size));
 old_fm.key(Key::FmRadio);old_fm.key(Key::Left);
 CHECK(old_fm.fm_request()==9950);
 Application sw;
 sw.key(Key::SwAm);
 CHECK(sw.page()==Page::FmProbe && sw.sw_am() && sw.fm_request()==7100);
 sw.key(Key::Up);CHECK(sw.fm_request()==7105);
 sw.key(Key::Back);sw.key(Key::FmRadio);
 CHECK(!sw.sw_am() && sw.fm_request()==10030);
 MockAm am_bus;
 fm::Probe am_probe;
 am_probe.begin(0,fm::Band::Shortwave);
 for(unsigned now=0;now<950;++now)am_probe.poll(am_bus,[](bool){},now);
 fm::Signal am_signal;
 CHECK(am_probe.frequency()==7100 && am_probe.part()==32 && am_probe.signal(950,am_signal));
 CHECK(am_signal.rssi==42 && am_signal.snr==18 && !am_signal.stereo);
 return 0;
}

extern "C" int tone_search_tests() {
 ToneSearch search;
 search.begin(ToneKind::Dcs);
 uint32_t now=0xffffff00u;
 for(unsigned i=1;i<=210;++i) {
     CHECK(search.candidate().index==i);
     CHECK(search.poll(now,true,true,false)==ToneSearch::Result::Pending); // Old candidate.
     CHECK(search.take_tune(now) && !search.take_tune(now));
     CHECK(search.poll(now+399,true,false,false)==ToneSearch::Result::Pending);
     CHECK(search.poll(now+400,true,false,false)==
           (i==210?ToneSearch::Result::Exhausted:ToneSearch::Result::Pending));
     now+=400;
 }
 search.begin(ToneKind::Ctcss);
 CHECK(search.take_tune(now));
 CHECK(search.poll(now+149,false,true,false)==ToneSearch::Result::Pending);
 CHECK(search.poll(now+150,false,true,false)==ToneSearch::Result::Unavailable);
 CHECK(search.poll(now,true,true,true)==ToneSearch::Result::Unavailable);

 Application app(false);
 CHECK(app.set_frequency(145500000));
 app.key(Key::Menu);app.key(Key::Ok);
 for(unsigned i=0;i<8;++i)app.key(Key::Down);
 app.key(Key::Ok);
 CHECK(app.radio_setting()==RadioSetting::RxCtcss && app.tone_search_available());
 dispatch_key(app,2);
 CHECK(app.tone_searching() && app.take_tone_tune(0));
 app.tick(31000);
 CHECK(app.page()==Page::RadioOptions);
 app.poll_tone_search(31000,true,false,false);
 CHECK(app.tone_candidate().index==2 && app.take_tone_tune(31000));
 app.poll_tone_search(31076,true,true,false);
 CHECK(!app.tone_searching() && app.radio_draft()==2);
 CHECK(app.active_radio_profile().rx_tone.kind==ToneKind::Off);
 app.key(Key::Ok);
 CHECK(app.active_radio_profile().rx_tone.kind==ToneKind::Ctcss &&
       app.active_radio_profile().rx_tone.index==2 && app.active_radio_profile().tx_tone.kind==ToneKind::Off);
 uint8_t snapshot[Application::SnapshotCapacity];
 const auto size=app.save_snapshot(snapshot,sizeof snapshot);
 Application restored(false);
 CHECK(size && restored.restore_snapshot(snapshot,size) && restored.active_radio_profile().rx_tone.index==2);
 app.key(Key::Ok);app.key(Key::Mode);app.key(Key::Back);
 CHECK(!app.tone_searching() && app.page()==Page::RadioOptions && app.radio_draft()==2);
 app.key(Key::Mode);app.tone_search_ptt();
 CHECK(!app.tone_searching() && app.page()==Page::Radio && app.voice_tx_page());
 CHECK(app.active_radio_profile().rx_tone.index==2);
 return 0;
}

extern "C" int about_page_tests() {
 Application app(false);
 uint8_t before[Application::SnapshotCapacity],after[Application::SnapshotCapacity];
 const size_t length=app.save_snapshot(before,sizeof before);
 app.key(Key::Menu);app.key(Key::Up);app.key(Key::Up);app.key(Key::Ok);
 CHECK(app.menu_group()==MenuGroup::System);
 app.key(Key::Down);app.key(Key::Down);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.page()==Page::About && !app.voice_tx_page());
 app.key(Key::Up);app.key(Key::Down);app.key(Key::NextSlot);app.digit(4);
 app.tick(1000000);CHECK(app.page()==Page::About && !app.editing() && !app.keypad_locked());
 CHECK(app.save_snapshot(after,sizeof after)==length);
 for(size_t i=0;i<length;++i)CHECK(before[i]==after[i]);
 app.key(Key::Back);CHECK(app.page()==Page::Settings && app.menu_group()==MenuGroup::System);
 app.key(Key::About);app.key(Key::Menu);CHECK(app.page()==Page::Settings);
 return 0;
}

extern "C" int tuner_step_tests() {
 Application app(false);
 CHECK(app.tuner_step(false)==10 && app.tuner_step(true)==5);
 const auto hz=app.config().vfo[0].rx_hz;
 for(unsigned am=0;am<2;++am) {
  const Key setting=am?Key::AmStep:Key::FmStep;
  for(unsigned index=0;index<(am?4u:3u);++index) {
   app.key(setting);while(app.tuner_step_draft()!=index)app.key(Key::Down);
   const auto old=app.tuner_step_selection(am);app.key(Key::Back);
   CHECK(app.page()==Page::Settings && app.tuner_step_selection(am)==old);
   app.key(setting);while(app.tuner_step_draft()!=index)app.key(Key::Down);app.key(Key::Ok);
   CHECK(app.tuner_step_selection(am)==index);
   app.key(am?Key::SwAm:Key::FmRadio);CHECK(app.page()==Page::FmProbe);
   // Enter an arbitrary off-grid frequency; a step is relative, not rounding.
   const unsigned digits_fm[]={1,0,0,3,0},digits_am[]={7,1,0,0};
   if(am){for(auto d:digits_am)app.digit(d);}else{for(auto d:digits_fm)app.digit(d);}
   app.key(Key::Ok);const unsigned start=app.fm_request(),step=app.tuner_step(bool(am));
   app.key(Key::Up);CHECK(app.fm_request()==start+step);
   app.key(Key::Down);CHECK(app.fm_request()==start);
   if(!am){app.key(Key::Right);CHECK(app.fm_seek());app.key(Key::Up);CHECK(!app.fm_seek());}
   app.key(Key::Back);
  }
 }
 // Exact band limits wrap once, with no overshoot or invalid stored value.
 for(unsigned am=0;am<2;++am) {
  app.key(am?Key::SwAm:Key::FmRadio);
  const unsigned lower=am?2300u:6500u,upper=am?30000u:10800u;
  unsigned digits[5],count=0,value=upper;
  do{digits[count++]=value%10;value/=10;}while(value);
  while(count)app.digit(digits[--count]);
  app.key(Key::Ok);CHECK(app.fm_request()==upper);
  app.key(Key::Up);CHECK(app.fm_request()==lower);
  app.key(Key::Down);CHECK(app.fm_request()==upper);app.key(Key::Back);
 }
 CHECK(app.config().vfo[0].rx_hz==hz);
 uint8_t data[Application::SnapshotCapacity];const size_t length=app.save_snapshot(data,sizeof data);
 Application restored(false);CHECK(length && restored.restore_snapshot(data,length));
 CHECK(restored.tuner_step(false)==20 && restored.tuner_step(true)==10);
 CHECK(restored.restore_snapshot(data,length-28));
 CHECK(restored.tuner_step(false)==10 && restored.tuner_step(true)==5);
 data[length-28]=3;CHECK(!restored.restore_snapshot(data,length));data[length-28]=2;
 data[length-24]=4;CHECK(!restored.restore_snapshot(data,length));data[length-24]=3;
 CHECK(restored.restore_snapshot(data,length));
 app.key(Key::FmStep);app.key(Key::Down);app.tick(30000);
 CHECK(app.page()==Page::Radio && app.tuner_step(false)==20);
 app.key(Key::AmStep);app.key(Key::Down);app.key(Key::Menu);
 CHECK(app.page()==Page::Settings && app.tuner_step(true)==10);
 return 0;
}

extern "C" int gps_dms_tests() {
 char value[18];
 auto equal=[](const char *a,const char *b){while(*a && *a==*b){++a;++b;}return *a==*b;};
 gps_coordinate("5545.123456",2,false,value,true);CHECK(equal(value,"55d45'07.40\""));
 gps_coordinate("03759.999999",3,false,value,true);CHECK(equal(value,"037d59'59.99\""));
 gps_coordinate("9000.000000",2,false,value,true);CHECK(equal(value,"90d00'00.00\""));
 gps_coordinate("18000.000000",3,false,value,true);CHECK(equal(value,"180d00'00.00\""));
 gps_coordinate("0000.0000",2,false,value,true);CHECK(equal(value,"00d00'00.00\""));
 gps_coordinate("00101.5",3,false,value,true);CHECK(equal(value,"001d01'30.00\""));
 gps_coordinate("5545.123456",2,true,value);CHECK(equal(value,"55.752057"));
 Application app(false);CHECK(!app.coordinate_selection());
 app.key(Key::CoordinateFormat);app.key(Key::Up);CHECK(app.coordinate_draft()==2);
 app.key(Key::Back);CHECK(!app.coordinate_selection());
 app.key(Key::CoordinateFormat);app.key(Key::Down);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.dms_coordinates() && !app.decimal_coordinates());
 app.key(Key::CoordinateFormat);app.key(Key::Down);CHECK(!app.coordinate_draft());
 app.key(Key::Back);CHECK(app.coordinate_selection()==2);
 uint8_t data[Application::SnapshotCapacity];const size_t length=app.save_snapshot(data,sizeof data);
 Application restored(false);CHECK(length && restored.restore_snapshot(data,length) && restored.dms_coordinates());
 CHECK(restored.restore_snapshot(data,length-28) && !restored.dms_coordinates());
 data[length-20]=2;CHECK(!restored.restore_snapshot(data,length));data[length-20]=1;
 // Legacy decimal flag and the new DMS flag may not be true together.
 data[32]=1;CHECK(!restored.restore_snapshot(data,length));data[32]=0;
 CHECK(restored.restore_snapshot(data,length) && restored.coordinate_selection()==2);
 data[length-20]=0;data[32]=1;
 CHECK(restored.restore_snapshot(data,length-28) && restored.decimal_coordinates() && !restored.dms_coordinates());

 app.key(Key::CoordinateFormat);app.key(Key::Up);app.tick(30000);
 CHECK(app.page()==Page::Radio && app.dms_coordinates());
 return 0;
}

extern "C" int gps_point_tests() {
 Application app(false);nav_reset();
 CHECK(app.distance_unit()==DistanceUnit::Kilometres && !app.waypoint().valid);
 app.key(Key::Waypoint);CHECK(app.page()==Page::Waypoint && !app.voice_tx_page());
 app.key(Key::Ok);CHECK(!app.waypoint_confirm() && !app.waypoint().valid);
 nmea("GPRMC,120000.00,A,5545.123456,S,03759.999999,W,1.00,90.0,071126,,,A",1000);
 app.tick(1000);GeoPoint point{};CHECK(gps_current_point(1000,point));
 CHECK(point.valid && point.latitude==-55752057 && point.longitude==-37999999);
 app.key(Key::Ok);CHECK(app.waypoint_confirm());app.key(Key::Back);
 CHECK(!app.waypoint_confirm() && !app.waypoint().valid);
 app.key(Key::Ok);app.tick(4000);app.key(Key::Ok);
 CHECK(!app.waypoint_confirm() && !app.waypoint().valid);
 nmea("GPRMC,120005.00,A,5545.123456,S,03759.999999,W,1.00,90.0,071126,,,A",5000);
 app.tick(5000);app.key(Key::Ok);CHECK(app.waypoint_confirm());
 nmea("GPRMC,120006.00,A,5546.000000,S,03800.000000,W,1.00,90.0,071126,,,A",5100);
 app.tick(5100);app.key(Key::Ok);
 CHECK(app.waypoint().valid && app.waypoint().latitude==-55766666 && app.waypoint().longitude==-38000000);
 const auto saved=app.waypoint();
 CHECK(app.set_option(Option::Gps,false));app.key(Key::Ok);
 CHECK(!app.waypoint_confirm() && app.waypoint().latitude==saved.latitude);
 CHECK(app.set_option(Option::Gps,true));
 app.key(Key::Down);app.key(Key::Ok);CHECK(app.waypoint_confirm() && app.waypoint_clear());
 app.key(Key::Back);CHECK(app.waypoint().valid);
 app.key(Key::Back);app.key(Key::DistanceUnits);app.key(Key::Down);app.key(Key::Back);
 CHECK(app.distance_unit()==DistanceUnit::Kilometres);
 app.key(Key::DistanceUnits);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.distance_unit()==DistanceUnit::Miles);
 app.key(Key::DistanceUnits);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.distance_unit()==DistanceUnit::NauticalMiles);
 uint8_t data[Application::SnapshotCapacity];const size_t length=app.save_snapshot(data,sizeof data);
 CHECK(length && length+24<=8192);
 Application restored(false);CHECK(restored.restore_snapshot(data,length));
 CHECK(restored.distance_unit()==DistanceUnit::NauticalMiles && restored.waypoint().latitude==saved.latitude);
 CHECK(!restored.waypoint_confirm() && restored.page()==Page::Radio);
 CHECK(restored.restore_snapshot(data,length-16) && !restored.waypoint().valid && restored.distance_unit()==DistanceUnit::Kilometres);
 data[length-16]=3;CHECK(!restored.restore_snapshot(data,length));data[length-16]=2;
 const uint32_t invalid=90000001;
 for(unsigned i=0;i<4;++i)data[length-12+i]=uint8_t(invalid>>(8*i));
 CHECK(!restored.restore_snapshot(data,length));
 for(unsigned i=0;i<4;++i)data[length-12+i]=uint8_t(uint32_t(saved.latitude)>>(8*i));
 data[length-4]=0;CHECK(!restored.restore_snapshot(data,length));data[length-4]=1;
 CHECK(restored.restore_snapshot(data,length));
 app.key(Key::Waypoint);app.key(Key::Down);app.key(Key::Ok);app.key(Key::Ok);
 CHECK(!app.waypoint().valid && !app.waypoint().latitude && !app.waypoint().longitude);
 app.key(Key::Back);app.key(Key::Distance);app.tick(100000);CHECK(app.page()==Page::Distance && !app.voice_tx_page());
 app.digit(4);app.key(Key::NextSlot);CHECK(app.page()==Page::Distance && !app.editing());
 app.key(Key::Back);CHECK(app.page()==Page::Settings);
 app.key(Key::DistanceUnits);app.key(Key::Down);app.tick(130000);
 CHECK(app.page()==Page::Radio && app.distance_unit()==DistanceUnit::NauticalMiles);
 nav_reset();return 0;
}
extern "C" void gps_geometry_probe(const int32_t *c,uint32_t *out) {
 const auto result=geo_range({c[0],c[1],true},{c[2],c[3],true});
 out[0]=result.metres;out[1]=result.bearing_valid?result.bearing_deci:0xffffffffu;out[2]=result.valid;
}
extern "C" int gps_geometry_tests() {
 const auto same=geo_range({0,0,true},{0,0,true});CHECK(same.valid && same.metres<=1 && !same.bearing_valid);
 const auto opposite=geo_range({0,0,true},{0,180000000,true});
 CHECK(opposite.valid && opposite.metres>=20015085 && opposite.metres<=20015089 && !opposite.bearing_valid);
 CHECK(!geo_range({90000001,0,true},{0,0,true}).valid);
 CHECK(!geo_range({}, {0,0,true}).valid);
 CHECK(distance_thousandths(1852,DistanceUnit::NauticalMiles)==1000);
 CHECK(distance_thousandths(1609344,DistanceUnit::Miles)==1000000);
 CHECK(distance_thousandths(12345,DistanceUnit::Kilometres)==12345);
 return 0;
}

extern "C" int compact_storage_tests() {
 using namespace rt950::storage;
 Application source(true),restored(false);
 source.key(Key::LanguageSelect);source.key(Key::Down);source.key(Key::Ok);
 source.key(Key::Timezone);source.key(Key::Down);source.key(Key::Ok);
 CHECK(source.set_frequency(435125000));CHECK(source.store(63));
 source.key(Key::FmStep);source.key(Key::Down);source.key(Key::Ok);
 source.key(Key::DistanceUnits);source.key(Key::Down);source.key(Key::Ok);
 nav_reset();nmea("GPRMC,120000.00,A,5545.123456,S,03759.999999,W,1.00,90.0,071126,,,A",1000);
 source.tick(1000);source.key(Key::Waypoint);source.key(Key::Ok);source.key(Key::Ok);source.key(Key::Back);
 uint8_t legacy[Application::SnapshotCapacity],packed[Application::SnapshotCapacity],roundtrip[Application::SnapshotCapacity];
 const size_t old_size=source.save_snapshot(legacy,sizeof legacy);
 const size_t size=source.save_compact_snapshot(packed,sizeof packed);
 CHECK(old_size+16==source.canonical_snapshot_size() && old_size==7992);
 CHECK(size && size<3000 && size%4==0 && size==source.compact_snapshot_size());
 CHECK(!source.save_compact_snapshot(nullptr,sizeof packed) && !source.save_compact_snapshot(roundtrip,19));
 for(auto &byte:roundtrip)byte=0xa5;
 CHECK(!source.save_compact_snapshot(roundtrip,size-1));
 CHECK(roundtrip[size-1]==0xa5 && roundtrip[size]==0xa5);
 CHECK(Application::snapshot_format_supported(packed,size) && Application::snapshot_format_supported(legacy,old_size));
 put(packed+4,3);CHECK(!Application::snapshot_format_supported(packed,size) && !restored.restore_snapshot(packed,size));
 put(packed+4,2);
 CHECK(restored.restore_snapshot(packed,size));CHECK(restored.save_snapshot(roundtrip,sizeof roundtrip)==old_size);
 for(size_t i=0;i<old_size;++i)CHECK(legacy[i]==roundtrip[i]);
 CHECK(restored.restore_snapshot(legacy,old_size));
 CHECK(restored.save_compact_snapshot(roundtrip,sizeof roundtrip)==size);
 for(size_t i=0;i<size;++i)CHECK(packed[i]==roundtrip[i]);
 // CRC/header/payload damage never replaces the previous in-memory settings.
 for(size_t offset=0;offset<size;offset+=size/32) {
  packed[offset]^=0x80;CHECK(!restored.restore_snapshot(packed,size));packed[offset]^=0x80;
 }
 CHECK(!restored.restore_snapshot(packed,size-1));
 CHECK(!restored.restore_snapshot(packed,size+4));
 auto fix_crc=[&](){put(packed+16,~crc_bytes(crc_bytes(0xffffffffu,packed,16),packed+20,word(packed+12)));};
 const auto words=word(packed+8);put(packed+8,words+1);fix_crc();CHECK(!restored.restore_snapshot(packed,size));
 put(packed+8,words);packed[20]=3;fix_crc();CHECK(!restored.restore_snapshot(packed,size));
 source.save_compact_snapshot(packed,sizeof packed);
 for(unsigned i=0;i<4;++i)packed[20+i]=255;
 packed[24]=16;fix_crc();CHECK(!restored.restore_snapshot(packed,size));
 source.save_compact_snapshot(packed,sizeof packed);
 const size_t encoded=word(packed+12),end=20+encoded;
 for(size_t i=end;i>20;--i)packed[i]=packed[i-1];
 packed[20]|=128;packed[21]=0;put(packed+12,uint32_t(encoded+1));
 const size_t overlong=(end+1+3)&~size_t(3);
 for(size_t i=end+1;i<overlong;++i)packed[i]=0;
 fix_crc();CHECK(!restored.restore_snapshot(packed,overlong));
 source.save_compact_snapshot(packed,sizeof packed);
 if(size>end){packed[end]=1;CHECK(!restored.restore_snapshot(packed,size));}
 CHECK(restored.save_snapshot(roundtrip,sizeof roundtrip)==old_size);
 for(size_t i=0;i<old_size;++i)CHECK(legacy[i]==roundtrip[i]);
 // A syntactically committed but semantically unusable newer slot must be
 // overwritten first. Every interrupted byte leaves the usable copy intact.
 const uint8_t good[4]={7,0,0,0},bad[4]={99,0,0,0},replacement[4]={8,0,0,0};
 for(unsigned usable=1;usable<=2;++usable) {
  MockSlots base;
  CHECK(save(base,1,good,4)==SaveResult::Saved);
  CHECK(save(base,1,bad,4)==SaveResult::Saved);
  if(usable==2){for(unsigned i=0;i<32;++i){const auto byte=base.slots[0][i];base.slots[0][i]=base.slots[1][i];base.slots[1][i]=byte;}}
  for(int cut=0;cut<60;++cut) {
   auto interrupted=base;interrupted.remaining=cut;
   CHECK(save(interrupted,1,replacement,4,uint8_t(usable))==SaveResult::Failed);
   CHECK(inspect(interrupted.view(usable==1?0:1),32,1).payload[0]==7);
  }
  CHECK(save(base,1,replacement,4,uint8_t(usable))==SaveResult::Saved);
  CHECK(inspect(base.view(usable==1?0:1),32,1).payload[0]==7);
 }
 nav_reset();return 0;
}

namespace {
struct BackupMockPort {
 uint8_t wire[8240]{};unsigned written=0,opens=0,closes=0;
 bool can_write=true,tx_done=true,overflow=false;const char *incoming=nullptr;
 void open(){++opens;written=0;incoming=nullptr;}
 void close(){++closes;}
 bool read(uint8_t &byte){if(!incoming || !*incoming)return false;byte=uint8_t(*incoming++);return true;}
 bool ready() const {return can_write;}
 bool done() const {return tx_done;}
 void put(uint8_t byte){if(written<sizeof wire)wire[written++]=byte;else overflow=true;}
};
}
// Reuse the historical FM test buffer after its suite has returned.
// A separate 8 KiB BSS fixture would overlap that suite's large legacy frame.
extern "C" int reset_defaults_tests() {
 Application app(false);
 CHECK(app.set_frequency(433100000u) && app.store(63));
 auto &saved=*const_cast<Memory *>(app.memory(63));saved.name[0]='X';saved.zone=2;
 const_cast<char *>(app.zone_name(2))[0]='X';
 const_cast<GeoPoint &>(app.waypoint())={55000000,37000000,true};
 CHECK(const_cast<MeasuredPower &>(app.measured_power()).store({446081250u,8000,470,40,TxPower::Pmr500mW}));
 app.key(Key::LanguageSelect);app.key(Key::Down);app.key(Key::Ok);
 app.key(Key::FmRadio);app.key(Key::Menu);app.key(Key::Left);app.key(Key::Ok);
 CHECK(app.broadcast_frequency(0)==10030);
 app.key(Key::Back);app.key(Key::Back);
 app.key(Key::MenuTimeout);app.key(Key::Up);app.key(Key::Ok);
 app.tick(1000);
 uint8_t before[Application::SnapshotCapacity],after[Application::SnapshotCapacity];
 const auto length=app.save_snapshot(before,sizeof before);CHECK(length==7992);
 app.key(Key::ResetSettings);CHECK(app.page()==Page::Reset && !app.reset_confirm() && !app.voice_tx_page());
 app.digit(1);app.key(Key::Right);app.key(Key::NextSlot);CHECK(!app.reset_confirm());
 app.key(Key::Ok);CHECK(app.page()==Page::Settings);
 CHECK(app.save_snapshot(after,sizeof after)==length);
 for(unsigned i=0;i<length;++i)CHECK(before[i]==after[i]);
 app.key(Key::ResetAll);app.key(Key::Down);CHECK(app.reset_confirm());
 app.key(Key::Back);app.key(Key::ResetAll);CHECK(!app.reset_confirm());
 app.key(Key::Down);app.tick(11000);CHECK(app.page()==Page::Radio); // timeout cancels
 CHECK(app.memory(63)->rx_hz==433100000u);
 CHECK(app.set_option(Option::Ble,true) && app.set_option(Option::Export,true));
 const_cast<rt_config &>(app.config()).beacon_on=true;
 app.tick(12000);app.key(Key::ResetSettings);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.page()==Page::Settings && app.menu_group()==MenuGroup::System && !app.voice_tx_page());
 CHECK(app.menu_timeout_selection()==2 && app.language()==Language::Russian);
 CHECK(app.memory(63)->rx_hz==433100000u && app.memory(63)->name[0]=='X' && app.memory(63)->zone==2);
 CHECK(app.zone_name(2)[0]=='X' && app.waypoint().valid && app.measured_power().count()==1);
 CHECK(!app.config().beacon_on && !app.config().ble_export && !app.monitor_enabled() && !app.fm_audio());
 CHECK(!app.config().vfo[0].tx_hz && !app.config().vfo[0].rx_hz);
 app.key(Key::FmPresets);CHECK(app.broadcast_frequency(0)==10030);
 app.key(Key::ResetAll);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.page()==Page::Settings && !app.voice_tx_page() && app.language()==Language::Russian);
 CHECK(app.measured_power().count()==1 && app.measured_power().find(TxPower::Pmr500mW,446081250u,8000)->code==40);
 CHECK(!app.waypoint().valid && app.zone_name(2)[0]=='Z');
 for(unsigned i=0;i<Application::MemoryCount;++i)CHECK(app.memory(i)->occupied && !app.memory(i)->tx_hz);
 CHECK(app.memory(63)->rx_hz!=433100000u && !app.config().vfo[0].tx_hz);
 app.key(Key::FmPresets);CHECK(!app.broadcast_frequency(0));
 app.key(Key::AmPresets);CHECK(!app.broadcast_frequency(0));
 CHECK(app.save_compact_snapshot(after,sizeof after));
 Application loaded(false);CHECK(loaded.restore_snapshot(after,app.compact_snapshot_size()));
 CHECK(loaded.measured_power().count()==1 && !loaded.reset_confirm() && loaded.memory(63)->occupied);
 CHECK(loaded.language()==Language::Russian && !loaded.waypoint().valid);
 // Runtime confirmation state is not serialized, and tick wrap never confirms.
 app.tick(0xfffffff0u);app.key(Key::ResetAll);app.key(Key::Down);app.tick(30000u);
 CHECK(app.page()==Page::Radio);
 app.key(Key::Menu);app.key(Key::Up);app.key(Key::Up);app.key(Key::Ok);
 CHECK(app.menu_group()==MenuGroup::System);
 bool settings=false,all=false;
 for(unsigned i=0;i<app.menu_count();++i) {
  settings|=app.menu_entry(i).action==Key::ResetSettings;all|=app.menu_entry(i).action==Key::ResetAll;
 }
 CHECK(settings && all);
 return 0;
}
extern "C" int settings_export_tests() {
 using namespace rt950::storage;
 Application app(true);SettingsExport<BackupMockPort> service;
 CHECK(!service.active() && !service.port().opens);
 app.key(Key::Backup);CHECK(app.page()==Page::Backup && !app.take_backup_request());
 app.key(Key::Ok);CHECK(app.take_backup_request() && !app.take_backup_request());
 app.tick(100000);CHECK(app.page()==Page::Backup);
 service.begin(app,100);CHECK(service.active() && service.phase()==BackupPhase::Waiting);
 service.port().incoming="PROGRAMBT9000U\nW\nERASE\n";service.poll(101,true);
 CHECK(!service.sent() && !service.port().written);
 service.port().incoming="RT950";service.poll(102,true);
 service.port().incoming="CFG\n";service.poll(400,true);CHECK(!service.sent());
 service.port().incoming="RT950CFG\n";service.port().tx_done=false;service.poll(401,true);
 CHECK(service.phase()==BackupPhase::Sending && service.sent()<=32);
 for(unsigned now=402;now<800 && service.sent()<service.total();++now)service.poll(now,true);
 CHECK(service.sent()==service.total() && service.active() && !service.port().closes);
 service.port().tx_done=true;service.poll(800,true);
 CHECK(service.phase()==BackupPhase::Complete && !service.active() && service.port().closes==1);
 CHECK(!service.port().overflow && service.port().written==8056);
 const auto *wire=service.port().wire;
 CHECK(word(wire+8)==1 && word(wire+12)==8008 && word(wire+16)==2);
 CHECK(word(wire+20)==~crc_bytes(0xffffffffu,wire+48,8008));
 CHECK(word(wire+44)==~crc_bytes(0xffffffffu,wire,44));
 // The exported bytes are the complete existing canonical snapshot.
 uint8_t canonical[Application::SnapshotCapacity];CHECK(app.save_portable_snapshot(canonical,sizeof canonical)==8008);
 for(unsigned i=0;i<8008;++i)CHECK(wire[48+i]==canonical[i]);
 backup_fixture_size=service.port().written;
 for(unsigned i=0;i<backup_fixture_size;++i)regression_fixture[i]=wire[i];
 service.begin(app,0xfffffff0u);service.poll(29983,true);CHECK(service.active());
 service.poll(29984,true);CHECK(service.phase()==BackupPhase::Timeout);
 service.begin(app,1000);service.port().can_write=false;service.port().incoming="RT950CFG\n";
 service.poll(1001,true);service.poll(1501,true);CHECK(service.phase()==BackupPhase::Timeout);
 service.port().can_write=true;service.begin(app,2000);service.port().incoming="RT950CFG\n";
 service.poll(2001,true);const auto sent=service.sent();service.poll(2002,false);
 CHECK(service.phase()==BackupPhase::Cancelled && service.sent()==sent);
 app.backup_ptt();CHECK(app.page()==Page::Radio && app.voice_tx_page());
 CHECK(!app.take_backup_request());app.key(Key::Backup);app.key(Key::Back);
 CHECK(app.page()==Page::Settings && !app.take_backup_request());
 return 0;
}

namespace {
struct ImportMockPort {
 uint8_t incoming[64]{},ack[32]{};unsigned at=0,size=0,written=0,opens=0,closes=0;
 bool can_write=true,tx_done=true,overflow=false;
 void open(){++opens;at=size=written=0;}
 void close(){++closes;}
 bool read(uint8_t &byte){if(at==size)return false;byte=incoming[at++];return true;}
 bool ready() const{return can_write;}
 bool done() const{return tx_done;}
 void put(uint8_t byte){if(written<sizeof ack)ack[written++]=byte;else overflow=true;}
 void feed(const uint8_t *data,unsigned length){at=0;size=length;written=0;for(unsigned i=0;i<length;++i)incoming[i]=data[i];}
};
}
extern "C" int settings_import_tests() {
 using namespace rt950::storage;
 Application app(false);CHECK(app.set_frequency(145500000u) && app.store(1));
 CHECK(const_cast<MeasuredPower &>(app.measured_power()).store({446081250u,8000,470,40,TxPower::Pmr500mW}));
 SettingsImport<ImportMockPort> service;uint32_t now=1;
 auto feed=[&](const uint8_t *data,unsigned size) {
  service.port().feed(data,size);
  for(unsigned i=0;i<3;++i)service.poll(now++,true);
 };
 auto start=[&]() {
  service.begin(now++);feed(reinterpret_cast<const uint8_t *>("RT950PUT\n"),9);
 };
 auto payload=[&](bool hold=false) {
  for(unsigned at=48;at<8056;at+=32){if(hold && at+32>=8056)service.port().tx_done=false;feed(regression_fixture+at,8056-at<32?8056-at:32);}
 };
 CHECK(service.phase()==ImportPhase::Off && !service.active());
 app.key(Key::Import);CHECK(!app.voice_tx_page() && !app.import_confirm() && !app.import_ready());
 app.key(Key::Ok);CHECK(app.take_import_request() && !app.take_import_request());
 start();CHECK(service.phase()==ImportPhase::Receiving && service.port().written==6);
 service.port().feed(regression_fixture,48);service.poll(now++,true);CHECK(service.received()==32);
 service.poll(now++,true);CHECK(service.received()==48);service.poll(now++,true);CHECK(service.port().written==5);
 payload(true);CHECK(service.active() && !service.port().closes);
 service.port().tx_done=true;service.poll(now++,true);CHECK(service.phase()==ImportPhase::Checked && !service.active() && service.received()==8056);
 CHECK(service.channels()==64 && service.port().closes==1 && !service.port().overflow);
 CHECK(app.memory(1)->rx_hz==145500000u && !app.memory(0)->occupied); // Not applied by upload.
 CHECK(!service.apply(app,now));app.import_ready(true);app.digit(1);CHECK(!app.import_confirm());
 app.key(Key::Ok);CHECK(app.page()==Page::Settings && !app.take_import_apply());
 CHECK(!service.apply(app,now)); // Even a staged frame needs the confirmation page.
 app.key(Key::Import);app.import_ready(true);app.key(Key::Down);CHECK(app.import_confirm());
 app.key(Key::Ok);CHECK(app.take_import_apply());
 CHECK(service.apply(app,now) && service.phase()==ImportPhase::Applied);
 CHECK(app.page()==Page::Settings && app.menu_group()==MenuGroup::System && !app.voice_tx_page());
 CHECK(app.memory(0)->occupied && app.memory(1)->rx_hz!=145500000u);
 CHECK(app.measured_power().count()==1 && app.measured_power().find(TxPower::Pmr500mW,446081250u,8000)->code==40);
 CHECK(!app.config().beacon_on && !app.config().ble_export && !app.import_confirm() && !app.import_ready());
 uint8_t compact[Application::SnapshotCapacity];const auto size=app.save_compact_snapshot(compact,sizeof compact);
 Application loaded(false);CHECK(size && loaded.restore_snapshot(compact,size));
 CHECK(loaded.memory(0)->occupied && loaded.measured_power().count()==1 && !loaded.import_ready());
 // Corrupt transport CRC, then a semantic error protected by correct CRCs.
 for(unsigned kind=0;kind<2;++kind) {
  app.key(Key::Import);start();
  regression_fixture[52]=99;
  if(kind) {
   put(regression_fixture+20,~crc_bytes(0xffffffffu,regression_fixture+48,8008));
   put(regression_fixture+44,~crc_bytes(0xffffffffu,regression_fixture,44));
  }
  feed(regression_fixture,48);payload();CHECK(service.phase()==ImportPhase::Error);
  CHECK(!service.apply(app,now) && app.memory(0)->occupied);
 }
 start();regression_fixture[12]^=1;feed(regression_fixture,48);CHECK(service.phase()==ImportPhase::Error);
 service.begin(0xfffffff0u);service.poll(29983,true);CHECK(service.active());
 service.poll(29984,true);CHECK(service.phase()==ImportPhase::Timeout);
 service.begin(now++);service.port().can_write=false;
 feed(reinterpret_cast<const uint8_t *>("RT950PUT\n"),9);service.poll(now+2000,true);
 CHECK(service.phase()==ImportPhase::Timeout);service.port().can_write=true;
 start();service.poll(now++,false);CHECK(service.phase()==ImportPhase::Cancelled);
 app.key(Key::Import);app.import_ready(true);app.key(Key::Down);app.backup_ptt();
 CHECK(app.page()==Page::Radio && !app.import_confirm() && !app.take_import_apply());
 return 0;
}

extern "C" int user_key_tests() {
 using namespace rt950::storage;
 Application app(false);
 auto home=[&](){for(unsigned i=0;i<5 && app.page()!=Page::Radio;++i)app.key(Key::Back);};
 auto bind=[&](Key key,KeyAction action) {
  app.key(key);for(unsigned i=0;i<unsigned(KeyAction::Count) && app.binding_draft()!=unsigned(action);++i)app.key(Key::Down);
  app.key(Key::Ok);home();
 };
 CHECK(app.key_action(0)==KeyAction::None && app.key_action(2)==KeyAction::Fm);
 bind(Key::Pf1Short,KeyAction::Monitor);bind(Key::Pf1Hold,KeyAction::Gps);bind(Key::Pf2Hold,KeyAction::Aprs);
 dispatch_key(app,0);CHECK(app.monitor_enabled());
 dispatch_key(app,128);CHECK(app.page()==Page::Gps && app.monitor_enabled());home();
 dispatch_key(app,129);CHECK(app.page()==Page::Aprs);home();
 dispatch_key(app,1);CHECK(app.page()==Page::FmProbe);dispatch_key(app,1);CHECK(app.page()!=Page::FmProbe);home();
 CHECK(app.set_option(Option::SidePtt,true));dispatch_key(app,128);CHECK(app.page()==Page::Radio);
 dispatch_key(app,0);CHECK(app.monitor_enabled());CHECK(app.set_option(Option::SidePtt,false));
 app.digit(4);dispatch_key(app,128);CHECK(app.page()==Page::Radio && app.editing());app.key(Key::Back);
 app.key(Key::KeypadLock);dispatch_key(app,128);CHECK(app.page()==Page::Radio && app.keypad_locked());app.key(Key::KeypadLock);
 app.key(Key::Pf1Short);app.key(Key::Down);app.key(Key::Back);CHECK(app.key_action(0)==KeyAction::Monitor);home();
 app.tick(1000);app.key(Key::Pf1Short);app.key(Key::Down);app.tick(31000);CHECK(app.page()==Page::Radio && app.key_action(0)==KeyAction::Monitor);
 for(unsigned raw=0;raw<2;++raw) {
  Controls held;CHECK(held.poll(uint8_t(raw),0)==255 && held.poll(uint8_t(raw),30)==255);
  CHECK(held.poll(uint8_t(raw),729)==255 && held.poll(uint8_t(raw),730)==(raw|128u));
  CHECK(held.poll(uint8_t(raw),2000)==255);held.poll(255,2010);CHECK(held.poll(255,2040)==255);
  Controls short_press;short_press.poll(uint8_t(raw),0);short_press.poll(uint8_t(raw),30);
  short_press.poll(255,100);CHECK(short_press.poll(255,130)==raw);
  Controls wrap;wrap.poll(uint8_t(raw),0xffffff00u);wrap.poll(uint8_t(raw),0xffffff1eu);
  CHECK(wrap.poll(uint8_t(raw),0xffffff1eu+700u)==(raw|128u));
 }
 uint8_t portable[Application::SnapshotCapacity],packed[Application::SnapshotCapacity];
 CHECK(app.save_portable_snapshot(portable,sizeof portable)==8008 && app.save_snapshot(packed,sizeof packed)==7992);
 Application loaded(false);CHECK(loaded.restore_portable_snapshot(portable,8008,2) && loaded.key_action(1)==KeyAction::Gps);
 portable[7992]=unsigned(KeyAction::Count);CHECK(!loaded.restore_portable_snapshot(portable,8008,2) && loaded.key_action(0)==KeyAction::Monitor);portable[7992]=1;
 CHECK(!loaded.restore_portable_snapshot(portable,0,2) && !loaded.restore_portable_snapshot(nullptr,8008,2));
 CHECK(loaded.restore_portable_snapshot(packed,7992,1) && loaded.key_action(0)==KeyAction::None && loaded.key_action(2)==KeyAction::Fm);
 const auto size=app.save_compact_snapshot(packed,sizeof packed);CHECK(size && word(packed+4)==2);
 CHECK(loaded.restore_snapshot(packed,size) && loaded.key_action(0)==KeyAction::Monitor && loaded.key_action(3)==KeyAction::Aprs);
 bind(Key::Pf1Short,KeyAction::ScanChannels);bind(Key::Pf2Hold,KeyAction::ScanVfo);
 dispatch_key(app,0);CHECK(app.page()==Page::ChannelScan);
 dispatch_key(app,0);CHECK(app.page()==Page::Radio);
 dispatch_key(app,129);CHECK(app.page()==Page::VfoScan);
 dispatch_key(app,129);CHECK(app.page()==Page::Radio);
 const auto scan_size=app.save_compact_snapshot(packed,sizeof packed);
 CHECK(scan_size && loaded.restore_snapshot(packed,scan_size) &&
       loaded.key_action(0)==KeyAction::ScanChannels && loaded.key_action(3)==KeyAction::ScanVfo);
 // Freeze the v1 decoder: remove exactly the four new one-byte fields.
 const auto encoded=word(packed+12);CHECK(encoded>=4);
 const auto old_encoded=encoded-4,old_size=(20+old_encoded+3u)&~3u;
 put(packed+4,1);put(packed+8,word(packed+8)-4);put(packed+12,old_encoded);
 for(unsigned i=20+old_encoded;i<old_size;++i)packed[i]=0;
 put(packed+16,~crc_bytes(crc_bytes(0xffffffffu,packed,16),packed+20,old_encoded));
 CHECK(loaded.restore_snapshot(packed,old_size) && loaded.key_action(0)==KeyAction::None && loaded.key_action(2)==KeyAction::Fm);
 app.key(Key::ResetSettings);app.key(Key::Down);app.key(Key::Ok);CHECK(app.key_action(0)==KeyAction::None && app.key_action(2)==KeyAction::Fm);
 return 0;
}

extern "C" uint32_t weather_frequency_probe(unsigned channel) {
 return channel<WeatherRx::ChannelCount?weather_channels[channel]:0;
}

extern "C" unsigned weather_tests() {
 Application app;
 const auto start=app.config();
 const auto same_slots=[&]{
  if(app.config().active!=start.active || app.config().watch_mask!=start.watch_mask)return false;
  for(unsigned i=0;i<3;++i)if(app.config().vfo[i].rx_hz!=start.vfo[i].rx_hz ||
      app.config().vfo[i].tx_hz!=start.vfo[i].tx_hz)return false;
  return true;
 };
 app.key(Key::Menu);app.key(Key::Ok);
 for(unsigned i=0;i<4;++i)app.key(Key::Up);
 CHECK(app.menu_entry(app.menu_selection()).action==Key::Weather);
 app.key(Key::Ok);
 CHECK(app.page()==Page::Weather && !app.voice_tx_page() && same_slots());
 auto &wx=app.weather();
 CHECK(wx.channel()==0 && !wx.scanning() && !wx.alerted());
 app.key(Key::Down);CHECK(wx.channel()==9);
 app.key(Key::Right);CHECK(wx.channel()==0);
 app.key(Key::Back);CHECK(app.page()==Page::Settings && app.menu_group()==MenuGroup::Radio);
 app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.page()==Page::Weather && wx.scanning());
 wx.poll(399,true,false,false);CHECK(wx.channel()==0);
 wx.poll(400,true,false,false);CHECK(wx.channel()==1);
 wx.poll(440,false,true,false);CHECK(!wx.alerted());
 wx.poll(470,true,true,false);
 wx.poll(510,true,false,false);
 wx.poll(525,true,true,false);
 wx.poll(584,true,true,false);CHECK(!wx.alerted());
 wx.poll(585,true,true,false);CHECK(wx.alerted() && wx.channel()==1);
 wx.poll(800,true,false,false);CHECK(wx.alerted() && wx.channel()==1);
 app.key(Key::Ok);CHECK(!wx.alerted() && !wx.scanning());
 app.key(Key::Ok);CHECK(wx.scanning());
 wx.poll(900,true,true,true);CHECK(wx.scanning() && !wx.alerted() && wx.channel()==1);
 app.weather_ptt();CHECK(app.page()==Page::Radio && app.voice_tx_page() && same_slots());

 app.key(Key::Pf1Hold);app.key(Key::Up);app.key(Key::Ok);app.key(Key::Back);
 if(app.page()==Page::Settings)app.key(Key::Back);
 CHECK(app.page()==Page::Radio && app.key_action(1)==KeyAction::Weather);
 app.user_key(0,true);CHECK(app.page()==Page::Weather && !wx.scanning());
 app.user_key(0,true);CHECK(app.page()==Page::Radio && same_slots());

 wx.begin(true,0xffffff00u);
 wx.poll(0x8fu,true,false,false);CHECK(wx.channel()==1);
 wx.poll(0x90u,true,false,false);CHECK(wx.channel()==2);
 return 0;
}

extern "C" unsigned gps_timezone_tests() {
 Application app(false);
 CHECK(app.timezone_minutes()==180);
 app.key(Key::Timezone);
 app.key(Key::Down);app.key(Key::Left);
 CHECK(app.timezone_draft()==135 && app.timezone_minutes()==180);
 app.key(Key::Back);
 CHECK(app.page()==Page::Radio && app.timezone_minutes()==180);
 app.key(Key::Timezone);app.key(Key::Right);
 app.tick(30000);
 CHECK(app.page()==Page::Radio && app.timezone_minutes()==180);
 app.key(Key::Timezone);app.key(Key::NextSlot);app.key(Key::Right);app.key(Key::Up);
 CHECK(app.timezone_draft()==285 && app.config().active==0);
 app.key(Key::Ok);CHECK(app.timezone_minutes()==285);
 uint8_t bytes[Application::SnapshotCapacity];
 const auto size=app.save_portable_snapshot(bytes,sizeof bytes);
 Application restored(false);
 CHECK(size==8008 && restored.restore_portable_snapshot(bytes,size,2));
 CHECK(restored.timezone_minutes()==285);
 restored.key(Key::Timezone);
 for(unsigned i=0;i<12;++i)restored.key(Key::NextSlot);
 restored.key(Key::Up);CHECK(restored.timezone_draft()==840);
 for(unsigned i=0;i<110;++i)restored.key(Key::Down);
 restored.key(Key::Left);CHECK(restored.timezone_draft()==-720);
 restored.key(Key::Ok);CHECK(restored.timezone_minutes()==-720);
 return 0;
}

extern "C" unsigned watch_profile_tests() {
 rt_config config;
 rt_defaults(&config);
 config.vfo[0].rx_hz=145500000;
 config.vfo[2].rx_hz=145500000; // Same frequency, different slot profile.
 voice::Watch watch;
 VoiceReceiver receiver;
 unsigned tunes=0;
 uint8_t profile_slot=255;
 const auto tune=[&](uint8_t slot,uint32_t hz) {
  profile_slot=slot;++tunes;
  if(hz)receiver.tune(hz,0);
  else receiver.stop();
 };
 watch.poll(receiver,config,0,true,false,tune);
 CHECK(profile_slot==0 && tunes==1);
 watch.poll(receiver,config,150,true,false,tune);
 CHECK(profile_slot==2 && watch.slot()==2 && tunes==2);
 watch.poll(receiver,config,151,true,true,tune);
 CHECK(profile_slot==0 && tunes==3);
 config.vfo[0].rx_hz=0;
 watch.poll(receiver,config,152,true,true,tune);
 CHECK(profile_slot==0 && !receiver.frequency);
 watch.poll(receiver,config,153,true,false,tune);
 CHECK(profile_slot==2 && receiver.frequency==145500000);
 watch.poll(receiver,config,154,false,false,tune);
 CHECK(!receiver.frequency && !watch.receiving());
 return 0;
}

extern "C" unsigned scan_resume_tests() {
 Application app;
 voice::ChannelScan scan;
 app.key(Key::ChannelScan);
 CHECK(scan.poll(app,0,true,false,false)==0);
 CHECK(scan.poll(app,50,true,true,true)==0);
 // A long PTT can expire the old dwell/hold and leave a matching RF frequency.
 CHECK(scan.poll(app,100,true,true,false,true)==0);
 CHECK(scan.poll(app,5000,true,true,false,true)==0);
 CHECK(scan.poll(app,5001,true,true,true)==0);
 CHECK(scan.poll(app,5150,true,true,false)==0);
 CHECK(scan.poll(app,5151,true,true,false)==1);

 app.scan_channel(1);app.key(Key::Ok);
 CHECK(app.scan_paused());
 CHECK(scan.poll(app,5200,true,false,false,true)==1);
 CHECK(scan.poll(app,8500,true,false,false)==1 && app.scan_paused());
 CHECK(scan.poll(app,8550,true,true,true)==1 && app.scan_paused());

 CHECK(scan.poll(app,8600,true,false,false,true)==1);
 app.key(Key::Right);
 CHECK(scan.poll(app,8601,true,true,true)==2 && !app.scan_paused());
 CHECK(scan.poll(app,8602,false,false,false)==voice::ChannelScan::None);

 app.key(Key::ScanMode);app.key(Key::Up);app.key(Key::Ok);
 CHECK(app.scan_mode()==ScanMode::Time);
 app.key(Key::ChannelScan);
 CHECK(scan.poll(app,9000,true,false,false)==0);
 CHECK(scan.poll(app,9100,true,true,false,true)==0);
 CHECK(scan.poll(app,12000,true,true,true)==0);
 CHECK(scan.poll(app,12150,true,true,false)==1);
 return 0;
}

extern "C" unsigned scan_tone_dwell_tests() {
 Application app;
 voice::ChannelScan scan;
 // Older snapshots can contain 50 ms, below the current editor minimum.
 uint8_t legacy[Application::SnapshotCapacity];
 const auto size=app.save_snapshot(legacy,sizeof legacy);
 CHECK(size!=0);
 legacy[12*4]=50;
 CHECK(app.restore_snapshot(legacy,size) && app.config().dwell_ms==50);
 app.key(Key::Menu);app.key(Key::Ok);
 for(unsigned i=0;i<8;++i)app.key(Key::Down);
 app.key(Key::Ok);CHECK(app.radio_setting()==RadioSetting::RxCtcss);
 app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.store(0) && app.memory(0)->radio.rx_tone.kind==ToneKind::Ctcss);
 app.key(Key::ChannelScan);
 CHECK(scan.poll(app,100,true,false,false)==0);
 CHECK(scan.poll(app,150,true,true,false)==0);
 CHECK(scan.poll(app,161,true,true,true)==0);
 app.key(Key::Right);CHECK(scan.poll(app,162,true,true,true)==1);
 CHECK(scan.poll(app,212,true,true,false)==2);

 scan.poll(app,213,false,false,false);
 CHECK(scan.poll(app,300,true,false,false)==0);
 CHECK(scan.poll(app,375,true,true,false)==0);
 CHECK(scan.poll(app,376,true,true,false)==1);
 scan.poll(app,377,false,false,false);
 CHECK(scan.poll(app,400,true,false,false)==0);
 CHECK(scan.poll(app,649,true,false,false)==0);
 CHECK(scan.poll(app,650,true,false,false)==1);
 scan.poll(app,651,false,false,false);

 app.key(Key::VfoScan);
 scan.poll(app,700,true,false,false);
 const auto first=scan.frequency();
 CHECK(first!=0);
 scan.poll(app,750,true,true,false);CHECK(scan.frequency()==first);
 scan.poll(app,776,true,true,false);CHECK(scan.frequency()!=first);
 return 0;
}

extern "C" unsigned broadcast_lower_bands_tests() {
 Application app;
 const struct {Key key;fm::Band band;uint16_t low,high,start;uint8_t spacing;} cases[]={
  {Key::LongwaveAm,fm::Band::Longwave,153,279,198,9},
  {Key::MediumwaveAm,fm::Band::Mediumwave,520,1710,999,9}
 };
 for(const auto &entry:cases) {
  app.key(entry.key);
  CHECK(app.page()==Page::FmProbe && app.fm_audio() && !app.voice_tx_page());
  CHECK(app.broadcast_band()==entry.band && app.fm_request()==entry.start);
  app.digit(4);app.digit(0);app.digit(0);app.key(Key::Ok);
  CHECK(app.fm_request()==entry.start);
  app.key(Key::Up);
  unsigned low=entry.low;
  unsigned digits[4]{},count=0;
  do{digits[count++]=low%10;low/=10;}while(low);
  while(count)app.digit(digits[--count]);
  app.key(Key::Ok);CHECK(app.fm_request()==entry.low);
  app.key(Key::Down);CHECK(app.fm_request()==entry.high);
  app.key(Key::Menu);app.key(Key::Left);app.key(Key::Ok);
  CHECK(app.broadcast_frequency(0)==entry.high && !app.voice_tx_page());
  uint8_t snapshot[Application::SnapshotCapacity];
  const auto size=app.save_portable_snapshot(snapshot,sizeof snapshot);
  Application restored(false);
  CHECK(restored.restore_portable_snapshot(snapshot,size,Application::PortableSchema));
  restored.key(Key::AmPresets);restored.key(Key::Ok);
  CHECK(restored.broadcast_band()==entry.band && restored.fm_request()==entry.high);
  app.key(Key::Back);app.key(Key::Back);
  fm::Probe probe;
  MockAm bus;
  bus.low=entry.low;bus.high=entry.high;bus.frequency=entry.start;bus.spacing=entry.spacing;
  probe.begin(0,entry.band);
  for(uint32_t now=0;now<=950;now+=5)probe.poll(bus,[](bool){},now);
  CHECK(probe.frequency()==entry.start && probe.failure()==fm::Probe::Failure::None);
  CHECK(!probe.retune(400,951));
  bus.frequency=entry.high;
  CHECK(probe.retune(entry.high,955));
  for(uint32_t now=955;now<=1200;now+=5)probe.poll(bus,[](bool){},now);
  CHECK(probe.frequency()==entry.high);
 }
 return 0;
}


extern "C" unsigned am_seek_tests() {
 const fm::Band bands[]={fm::Band::Longwave,fm::Band::Mediumwave,fm::Band::Shortwave};
 const Key keys[]={Key::LongwaveAm,Key::MediumwaveAm,Key::SwAm};
 for(unsigned i=0;i<3;++i) {
  const auto plan=fm::band_plan(bands[i]);
  MockAm bus;
  bus.low=plan.low;bus.frequency=plan.initial;bus.properties=6;
  fm::Receiver rx;
  auto run=[&](uint32_t from,uint32_t to){for(auto now=from;now<=to;now+=5)rx.poll(bus,now);};
  CHECK(rx.tune(plan.initial,0,bands[i]));
  run(0,100);CHECK(rx.frequency()==plan.initial);
  Application app;
  app.key(keys[i]);app.key(Key::Right);CHECK(app.fm_seek() && !app.voice_tx_page());
  CHECK(rx.seek(105));bus.frequency=plan.high;bus.tune_flags=1;
  run(105,205);
  uint16_t found=0;
  CHECK(rx.take_seek_result(found) && found==plan.high && bus.seek_commands==1 && bus.seek_arg==12 && bus.status_arg==1);
  app.fm_seek_result(found);CHECK(app.fm_request()==plan.high && !app.fm_seek());

  for(unsigned response=0;response<4;++response) {
   bus.frequency=plan.high;CHECK(rx.tune(plan.high,210,bands[i]));run(210,310);
   CHECK(rx.seek(315));
   bus.frequency=response==2?uint16_t(plan.high+1):response==3?plan.high:plan.low;
   bus.tune_flags=response==0?0:response==1?0x81:1;
   run(315,415);
   CHECK(rx.take_seek_result(found) && !found && !rx.frequency());
  }
  app.key(Key::Right);app.fm_seek_result(0);CHECK(app.fm_request()==plan.high && !app.fm_seek());
  bus.frequency=plan.high;CHECK(rx.tune(plan.high,420,bands[i]));run(420,520);
  CHECK(rx.seek(525));bus.seek_stalled=true;run(525,550);
  app.key(Key::Right);app.key(Key::Up);CHECK(!app.fm_seek());
  rx.cancel_seek(555);run(555,585);
  CHECK(bus.status_arg==3 && !rx.seeking() && !rx.take_seek_result(found));
  bus.seek_stalled=false;CHECK(rx.tune(plan.high,590,bands[i]));run(590,690);
  CHECK(rx.seek(695));bus.seek_stalled=true;run(695,745);
  run(180695,180725);
  CHECK(bus.status_arg==3 && !rx.seeking() && rx.take_seek_result(found) && !found);
  app.key(Key::Left);CHECK(app.fm_seek() && !app.fm_seek_up());
  bus.seek_stalled=false;bus.frequency=plan.low;
  CHECK(app.fm_request()==plan.low && rx.tune(plan.low,180730,bands[i]));run(180730,180830);
  CHECK(rx.seek(180835,app.fm_seek_up()));
  bus.frequency=plan.high;bus.tune_flags=1;run(180835,180935);
  CHECK(rx.take_seek_result(found) && found==plan.high && bus.seek_arg==4);
  app.fm_seek_result(found);CHECK(app.fm_request()==plan.high && !app.fm_seek());
  app.key(Key::Left);app.key(Key::Right);CHECK(!app.fm_seek());
 }
 return 0;
}

extern "C" int aprs_personal_tests() {
 aprs::RecentMessages messages;
 messages.select_identity("N0CALL",15);
 uint8_t frame[32]={'A'<<1,'P'<<1,'R'<<1,'S'<<1,' '<<1,' '<<1,0x60,
     'W'<<1,'1'<<1,'A'<<1,'B'<<1,'C'<<1,' '<<1,0x61,0x03,0xf0,':'};
 auto receive=[&](const char *destination,char body) {
     unsigned i=0;
     for(;destination[i];++i)frame[17+i]=uint8_t(destination[i]);
     for(;i<9;++i)frame[17+i]=' ';
     frame[26]=':';frame[27]=uint8_t(body);
     messages.record(frame,28);
 };
 receive("n0call-15",'A');
 CHECK(messages.personal_unread() && !messages.at(0)->unread);
 messages.read_personal(0);CHECK(!messages.personal_unread());
 receive("n0call-15",'A');
 CHECK(messages.count()==1 && messages.count(true)==1 && !messages.personal_unread());
 receive("N0CALL-1",'B');receive("N0CALL",'C');receive("N0CALL-14",'D');
 CHECK(messages.count()==3 && messages.count(true)==1 && messages.at(0,true)->body[0]=='A');
 messages.select_identity("N0CALL",15);CHECK(messages.count(true)==1);
 messages.select_identity("N0CALL",0);CHECK(messages.count(true)==0 && messages.count()==3);
 receive("N0CALL-00",'E');receive("N0CALL-01",'F');receive("N0CALL-1",'G');
 CHECK(messages.count(true)==0);
 receive("N0CALL-0",'H');receive("N0CALL",'I');
 CHECK(messages.count(true)==2 && messages.at(0,true)->body[0]=='I' && messages.at(1,true)->body[0]=='H');
 messages.read_personal(1);CHECK(messages.personal_unread());
 messages.read_personal(0);CHECK(!messages.personal_unread());
 messages.select_identity("",0);receive("N0CALL",'J');CHECK(!messages.count(true) && !messages.personal_unread());
 Application app(false);
 app.key(Key::AprsMessages);app.aprs_message_counts(3,2);
 app.key(Key::Mode);CHECK(app.aprs_personal() && !app.aprs_message_index());
 app.key(Key::Up);CHECK(app.aprs_message_index()==1);
 app.key(Key::Up);CHECK(!app.aprs_message_index());
 app.key(Key::Down);CHECK(app.aprs_message_index()==1);
 app.aprs_message_counts(3,0);CHECK(!app.aprs_message_index());
 app.key(Key::Mode);CHECK(!app.aprs_personal());
 app.key(Key::Up);CHECK(app.aprs_message_index()==2);
 return 0;
}

extern "C" int beacon_altitude_tests() {
 nav_reset();
 nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,123.4,M,46.9,M,,",100);
 const auto &nav=*nav_get();
 CHECK(aprs::beacon_altitude(nav,100)==405 && aprs::beacon_altitude(nav,3100)==-1);
 nmea("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,-0.1,M,46.9,M,,",200);
 CHECK(aprs::beacon_altitude(nav,200)==-1);
 aprs::FrameTransmitter tx;
 const char call[7]="N0CALL";
 const int32_t values[]={-1,0,405,999999,1000000};
 const char *fields[]={"","/A=000000","/A=000405","/A=999999",""};
 for(unsigned index=0;index<5;++index) {
     CHECK(aprs::encode_beacon(tx,call,9,"4807.038",'N',"01131.000",'E','>',values[index]));
     AprsCapture capture;
     aprs::FrameReceiver rx;
     bool previous=false;
     for(size_t i=0;i<tx.size();++i) {
         const bool current=tx.level(i);
         rx.bit(current==previous,capture_aprs,&capture);
         previous=current;
     }
     CHECK(capture.count==1 && capture.length==(fields[index][0]?48u:39u));
     for(unsigned i=0;fields[index][i];++i)CHECK(capture.kiss[38+i]==uint8_t(fields[index][i]));
 }
 return 0;
}

extern "C" int fm_native_seek_tests() {
 for(unsigned trial=0;trial<6;++trial) {
     fm::Receiver rx;
     MockFm bus;
     auto run=[&](uint32_t start,uint32_t end){for(uint32_t now=start;now<=end;now+=5)rx.poll(bus,now);};
     CHECK(rx.tune(10030,0));
     run(0,100);CHECK(rx.frequency()==10030 && rx.seek(105));
     bus.flags=trial==1?0x81:trial==2?0:1;
     bus.frequency=trial==3?6400:10240;
     bus.seek_stalled=trial>=4;
     run(105,205);
     CHECK(bus.seek_arg==0x0c);
     uint16_t found=0;
     if(trial==4) {
         rx.cancel_seek(210);run(210,250);
         CHECK(bus.status_arg==3 && !rx.take_seek_result(found));
     } else {
         if(trial==5)run(30105,30140);
         CHECK(bus.status_arg==(trial==5?3:1) && rx.take_seek_result(found) && found==(trial==0?10240:0));
     }
     if(trial!=0) {
         bus.seek_stalled=false;bus.frequency=10030;
         CHECK(rx.tune(10030,30150));run(30150,30250);CHECK(rx.frequency()==10030);
     }
 }
 return 0;
}

extern "C" int spectrum_tests() {
 Application app(false);
 app.key(Key::Spectrum);CHECK(app.page()==Page::Radio);
 CHECK(app.set_frequency(136000000));
 app.key(Key::Spectrum);CHECK(app.page()==Page::Spectrum && !app.voice_tx_page());
 auto &sweep=app.spectrum();
 CHECK(sweep.count()==11 && sweep.selected()==0 && sweep.frequency(0)==136000000);
 CHECK(sweep.take_tune(0xffffff80u)==136000000);
 sweep.observe(1,136000010,true,511); // Other owner's sample.
 CHECK(!sweep.take_tune(2) && sweep.sample(0)==Spectrum::Unmeasured);
 sweep.observe(122,0,false,0); // 250 ms with tick rollover.
 CHECK(sweep.take_tune(123)==sweep.frequency(1));
 sweep.observe(150,sweep.frequency(1),true,234);
 CHECK(sweep.sample(1)==234);
 for(unsigned i=2;i<sweep.count();++i) {
     CHECK(sweep.take_tune(1000)==sweep.frequency(i));
     sweep.observe(1050,sweep.frequency(i),true,0);
 }
 CHECK(!sweep.sweeping() && sweep.sample(10)==0 && !sweep.take_tune(5000));
 app.key(Key::Right);CHECK(sweep.selected()==1);
 app.key(Key::Ok);CHECK(sweep.sweeping() && sweep.selected()==0 && sweep.sample(1)==Spectrum::Unmeasured);
 app.spectrum_ptt();CHECK(app.page()==Page::Radio && app.config().vfo[0].rx_hz==136000000);
 CHECK(app.set_frequency(470000000));
 app.key(Key::Spectrum);CHECK(sweep.count()==11 && sweep.selected()==10 && sweep.frequency(10)==470000000);
 app.key(Key::Right);CHECK(sweep.selected()==10);
 app.key(Key::Back);CHECK(app.page()==Page::Radio);
 CHECK(app.set_frequency(446081250));
 app.key(Key::Spectrum);CHECK(sweep.count()==21 && sweep.frequency(10)==446081250);
 return 0;
}

extern "C" int aprs_saved_point_tests() {
 Application app(false);nav_reset();
 app.key(Key::BeaconSource);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.page()==Page::BeaconSource && !app.beacon_saved_point());
 app.key(Key::Back);app.key(Key::BeaconIdentity);app.key(Key::Up);app.key(Key::Ok);
 CHECK(app.beacon_identity().callsign[0] && app.set_aprs_frequency(144800000));
 CHECK(app.set_option(Option::Gps,true) && app.set_option(Option::Beacon,true));
 nmea("GPRMC,120000.00,A,5545.123456,S,03759.999999,W,1.00,90.0,071126,,,A",1000);
 app.tick(1000);app.key(Key::Waypoint);app.key(Key::Ok);app.key(Key::Ok);
 CHECK(app.waypoint().valid);
 app.key(Key::Back);app.key(Key::BeaconSource);app.key(Key::Down);app.key(Key::Back);
 CHECK(!app.beacon_saved_point() && app.enabled(Option::Beacon));
 app.key(Key::BeaconSource);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.beacon_saved_point() && !app.enabled(Option::Beacon));
 CHECK(app.set_option(Option::Gps,false) && app.set_option(Option::Beacon,true));
 aprs::FrameTransmitter tx;
 CHECK(aprs::encode_beacon(tx,app.beacon_identity().callsign,0,app.waypoint(),'>'));
 AprsCapture capture;
 aprs::FrameReceiver rx;
 bool previous=false;
 for(size_t i=0;i<tx.size();++i) {
     const bool current=tx.level(i);rx.bit(current==previous,capture_aprs,&capture);previous=current;
 }
 const char expected[]="!5545.12S/03800.00W>";
 CHECK(capture.count==1 && capture.length==39);
 for(unsigned i=0;expected[i];++i)CHECK(capture.kiss[18+i]==uint8_t(expected[i]));
 CHECK(!aprs::encode_beacon(tx,app.beacon_identity().callsign,0,GeoPoint{90000001,0,true},'>'));
 uint8_t data[Application::SnapshotCapacity];const auto size=app.save_compact_snapshot(data,sizeof data);
 Application restored(false);CHECK(size && restored.restore_snapshot(data,size));
 CHECK(restored.waypoint().valid && !restored.beacon_saved_point() && !restored.enabled(Option::Beacon));
 app.key(Key::Waypoint);app.key(Key::Down);app.key(Key::Ok);app.key(Key::Ok);
 CHECK(!app.waypoint().valid && !app.beacon_saved_point() && !app.enabled(Option::Beacon));
 return 0;
}

extern "C" int aprs_clear_messages_tests() {
 Application app(false);
 app.key(Key::AprsMessages);app.key(Key::Erase);
 CHECK(app.page()==Page::ClearAprsMessages && app.voice_tx_page());
 app.key(Key::Back);
 CHECK(app.page()==Page::AprsMessages && !app.take_aprs_messages_clear());
 app.key(Key::Erase);app.key(Key::Ok);
 CHECK(app.page()==Page::AprsMessages && app.take_aprs_messages_clear());
 CHECK(!app.take_aprs_messages_clear() && !app.take_aprs_clear());
 aprs::RecentMessages messages;
 messages.select_identity("N0CALL",0);
 const uint8_t frame[]={'A'<<1,'P'<<1,'R'<<1,'S'<<1,64,64,0x60,
     'W'<<1,'1'<<1,'A'<<1,'B'<<1,'C'<<1,64,0x61,0x03,0xf0,
     ':','N','0','C','A','L','L',' ',' ',' ',':','H'};
 messages.record(frame,sizeof frame);CHECK(messages.personal_unread());
 messages.clear();CHECK(!messages.count() && !messages.count(true) && !messages.personal_unread());
 messages.record(frame,sizeof frame);CHECK(messages.count()==1 && messages.count(true)==1 && messages.personal_unread());
 return 0;
}

extern "C" int monitor_hold_tests() {
 Application app(false);
 app.key(Key::Pf1Hold);app.key(Key::Down);app.key(Key::Ok);
 for(unsigned i=0;i<4 && app.page()!=Page::Radio;++i)app.key(Key::Back);
 CHECK(app.key_action(1)==KeyAction::Monitor && app.page()==Page::Radio);
 Controls controls;
 auto poll=[&](uint8_t raw,uint32_t now) {
     const auto code=controls.poll(raw,now);
     app.update_monitor_hold(controls.accepted());
     if(code!=255)dispatch_key(app,code);
 };
 poll(0,0);poll(0,30);poll(0,729);
 CHECK(!app.monitor_active());
 poll(0,730);CHECK(app.monitor_active() && !app.monitor_enabled());
 poll(255,740);poll(0,750);poll(0,779);CHECK(app.monitor_active());
 poll(255,800);poll(255,829);CHECK(app.monitor_active());
 poll(255,830);CHECK(!app.monitor_active() && !app.monitor_enabled());
 app.key(Key::Monitor);app.key(Key::Down);app.key(Key::Ok);
 poll(0,900);poll(0,930);poll(0,1630);
 poll(255,1640);poll(255,1670);CHECK(app.monitor_active() && app.monitor_enabled());
 app.key(Key::Monitor);app.key(Key::Down);app.key(Key::Ok);
 CHECK(app.set_option(Option::SidePtt,true));
 poll(0,1700);poll(0,1730);poll(0,2430);CHECK(!app.monitor_active());
 return 0;
}
