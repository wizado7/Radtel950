from pathlib import Path
import subprocess,argparse,struct
from unicorn import *
from unicorn.arm_const import *
p=argparse.ArgumentParser();p.add_argument('--prefix',required=True);p.add_argument('--stock',type=Path);p.add_argument('--calibration-dump',type=Path);a=p.parse_args();r=Path(__file__).resolve().parent;o=r/'build'
def run(cmd):return subprocess.check_output([str(x) for x in cmd],cwd=r,text=True)
flags=['-mcpu=cortex-m4','-mthumb','-mfloat-abi=hard','-mfpu=fpv4-sp-d16','-Os','-ffreestanding','-fno-builtin','-fno-exceptions','-fno-rtti','-std=c++17','-DRT950_MAIN_TX','-Wall','-Wextra','-Werror','-I../include']
run([a.prefix+'g++',*flags,'-fno-if-conversion','-fno-if-conversion2','-c','test_regressions_arm.cpp','-o',o/'regressions.o'])
ld=o/'logic.ld';ld.write_text('MEMORY { FLASH (rx) : ORIGIN = 0x08003000, LENGTH = 244K RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96K }\nSECTIONS { .text : { *(.text*) *(.rodata*) } > FLASH .data : { *(.data*) } > RAM AT> FLASH .bss : { *(.bss*) } > RAM }')
elf=o/'logic.elf';run([a.prefix+'g++',*flags,'-nostdlib','-T',ld,'-Wl,-e,regression_tests',o/'regressions.o',o/'application.o',o/'controls.o',o/'core.o',o/'runtime.o',o/'nmea_status.o',o/'gps_view.o',o/'gps_navigation.o',o/'radio_settings.o',o/'settings_snapshot.o',o/'voice_factory_board.o',o/'aprs_dac.o',o/'voice_rx.o',o/'gpio.o','-o',elf])
raw=o/'logic.bin';run([a.prefix+'objcopy','-O','binary',elf,raw]);symbols={}
for line in run([a.prefix+'nm','-n',elf]).splitlines():
 t=line.split()
 if len(t)==3:symbols[t[2]]=int(t[0],16)
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
u.mem_map(0x08000000,0x100000);u.mem_map(0x20000000,0x18000);u.mem_write(0x08003000,raw.read_bytes())
u.reg_write(UC_ARM_REG_C1_C0_2,0xf00000);u.reg_write(UC_ARM_REG_FPEXC,1<<30)
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['regression_tests']|1,0x080ff000,count=12000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000,f'test did not return at {u.reg_read(UC_ARM_REG_PC):#x}'
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'regression failed at C++ line {result}'
# Run this independent suite with a fresh stack; do not add its UI objects
# to the already large historical regression_tests stack frame.
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['menu_timeout_tests']|1,0x080ff000,count=2000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'menu timeout test failed at C++ line {result}'
print('ARM menu timeout: inactivity, input, wrap, draft cancellation, status/scan exclusion, OFF, snapshot migration and tail defaults PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['measured_power_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'measured power test failed at C++ line {result}'
print('ARM numeric power: exact channel/voltage, missing calibration, code limit, table capacity, UI measurement flow and persistence PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['band_profile_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'band profile test failed at C++ line {result}'
print('ARM LPD/PMR profiles: 68/8 grids, exact frequencies, apply/cancel, slot isolation, saved channels unchanged and snapshot PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['power_diagnostics_tests']|1,0x080ff000,count=2000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'power diagnostics test failed at C++ line {result}'
print('ARM power diagnostics: true post-key samples, duplicate/stale rejection, sag, owner/frequency changes, tick wrap and read-only pages PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['mic_gain_tests']|1,0x080ff000,count=2000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'microphone test failed at C++ line {result}'
print('ARM microphone menu: factory default, list, apply/cancel, timeout, persistence, migration and invalid enum rejection PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['keypad_lock_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'keypad lock test failed at C++ line {result}'
print('ARM keypad lock: all keys/digits, encoder actions, 700ms hold/release/wrap, editor safety, PTT eligibility, menu, wake and unlocked reload PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['monitor_settings_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'monitor settings failed at C++ line {result}'
print('ARM monitor settings: explicit list/apply/cancel/timeout, active Watch slot, read-only diagnostics, PTT eligibility, lock and OFF on reload PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['auto_lock_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'auto lock failed at C++ line {result}'
print('ARM auto lock: idle boundary, unlock grace, activity/PTT, editor/status exclusion, menu timeout, wrap, OFF, migration and invalid snapshot PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['weather_tests']|1,0x080ff000,count=2000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'weather test failed at C++ line {result}'
print('ARM NOAA: menu and PF entry/exit, wrap, missed carrier, alert latch, acknowledgement, fault and PTT return PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['gps_timezone_tests']|1,0x080ff000,count=2000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'GPS timezone failed at C++ line {result}'
print('ARM GPS timezone: draft, cancel, timeout, 15/30/60-minute steps, boundaries and existing snapshot schema PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['watch_profile_tests']|1,0x080ff000,count=2000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'Watch profile failed at C++ line {result}'
print('ARM Watch profiles: selection callback, identical frequencies, empty slots, PTT and release PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['scan_resume_tests']|1,0x080ff000,count=2000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'scan resume failed at C++ line {result}'
print('ARM scan resume: fresh dwell, stale sample ignored, pause, manual step and restart PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['scan_tone_dwell_tests']|1,0x080ff000,count=2000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'scan tone dwell failed at C++ line {result}'
print('ARM scan tone dwell: minimum dwell, matching tone hold, wrong tone advance, RX timeout, manual step and VFO PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['broadcast_lower_bands_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'LW/MW AM failed at C++ line {result}'
print('ARM LW/MW AM: band limits, tuning, preset roundtrip and receive-only UI PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['am_seek_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'AM seek failed at C++ line {result}'
print('ARM AM seek: both directions on all bands, valid channel, invalid/wrapped/out-of-band responses, cancel, timeout and retune PASS')
for suite,description in (
 ('aprs_personal_tests','APRS inbox: callsign/SSID, unread, duplicates, Reply-ACK, control packets and navigation'),
 ('beacon_altitude_tests','APRS beacon altitude: fresh GGA, invalid and stale data, six-digit field and AX.25 round trip'),
 ('fm_native_seek_tests','FM native seek: command 0x21, valid and invalid replies, wrap, cancellation, timeout and retune'),
 ('spectrum_tests','RX spectrum: bounded grid, stale samples, timeout, PTT and measured frequency copied RX-only'),
 ('frequency_copy_tests','frequency copy: two fresh samples, OEM rounding, restart, band limits, timeout, fault, cancel, menu and RX-only VFO'),
 ('manual_waypoint_tests','manual GPS point: decimal input, hemisphere, range, cancel, snapshot and beacon revocation'),
 ('spectrum_binding_tests','spectrum PF binding: open/close, PTT C priority and portable storage'),
 ('aprs_distance_tests','GPS APRS distance: selection from visible reports, target isolation, return and RX-only page'),
 ('aprs_saved_point_tests','APRS saved position: explicit source, no GPS dependency, coordinate rounding, AX.25 and startup defaults'),
 ('aprs_routing_tests','APRS routing: AX.25 paths and SSIDs, hop and duplicate bounds, TX waits, voice priority, manual permit and storage'),
 ('aprs_clear_messages_tests','APRS clear messages: cancel, confirm, clear unread and receive again'),
 ('monitor_hold_tests','momentary monitor: hold threshold, contact bounce, release, latched monitor and PF1/PTT interlock')):
 u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
 u.emu_start(symbols[suite]|1,0x080ff000,count=3000000)
 assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
 result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'{suite} failed at C++ line {result}'
 print('ARM '+description+' PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['broadcast_player_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'broadcast player failed at C++ line {result}'
print('ARM broadcast player: radio controls, FM snapshot migration and shortwave receiver PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['tone_search_tests']|1,0x080ff000,count=3000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'tone search failed at C++ line {result}'
print('ARM RX tone search: all DCS candidates, fresh samples, timeout, explicit save, snapshot, cancel and PTT PASS')
for suite,marker in [('broadcast_bank_tests','ARM broadcast banks:'),('about_page_tests','ARM about page:')]:
 u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
 u.emu_start(symbols[suite]|1,0x080ff000,count=4000000)
 assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
 result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'{suite} failed at C++ line {result}'
 description=' FM/AM isolation, 8 slots, confirm/cancel, recall, migration, corrupt snapshots and timeout' if suite=='broadcast_bank_tests' else ' System navigation, no TX, digits ignored, no snapshot changes and timeout exclusion'
 print(marker+description+' PASS')
for suite,marker in [('tuner_step_tests','ARM tuner steps:'),('gps_dms_tests','ARM GPS DMS:')]:
 u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
 u.emu_start(symbols[suite]|1,0x080ff000,count=4000000)
 assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
 result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'{suite} failed at C++ line {result}'
 print(marker+' all values, boundaries, cancel, timeout, save, migration and invalid snapshots PASS')
for suite in ('gps_point_tests','gps_geometry_tests'):
 u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
 u.emu_start(symbols[suite]|1,0x080ff000,count=4000000)
 assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
 result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'{suite} failed at C++ line {result}'
print('ARM GPS point: fresh own FIX, confirmation recheck, cancel/clear, units, snapshot migration/invalid data and capacity PASS')
import math,random
rng=random.Random(95052)
points=[(0,0,0,1000000),(0,0,1000000,0),(0,179999000,0,-179999000),(90000000,0,90000000,90000000),(-90000000,0,90000000,0),(55750000,37616667,55751000,37617667),(0,0,0,0),(0,0,0,180000000)]
points += [tuple(rng.randrange(-89999999,90000000) if j%2==0 else rng.randrange(-180000000,180000001) for j in range(4)) for _ in range(600)]
points += [(55750000,37000000,55750000+d,37000000+d) for d in (1,10,100,1000)]
worst_distance=worst_bearing=0
for coords in points:
 u.mem_write(0x20015000,struct.pack('<4i',*coords))
 u.reg_write(UC_ARM_REG_R0,0x20015000);u.reg_write(UC_ARM_REG_R1,0x20015100)
 u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
 u.emu_start(symbols['gps_geometry_probe']|1,0x080ff000,count=200000)
 assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
 metres,bearing,valid=struct.unpack('<3I',u.mem_read(0x20015100,12));assert valid
 lat1,lon1,lat2,lon2=[math.radians(c/1000000) for c in coords]
 def vector(lat,lon):return (math.cos(lat)*math.cos(lon),math.cos(lat)*math.sin(lon),math.sin(lat))
 vec_a,vec_b=vector(lat1,lon1),vector(lat2,lon2)
 dot=sum(x*y for x,y in zip(vec_a,vec_b));cross=(vec_a[1]*vec_b[2]-vec_a[2]*vec_b[1],vec_a[2]*vec_b[0]-vec_a[0]*vec_b[2],vec_a[0]*vec_b[1]-vec_a[1]*vec_b[0])
 reference=6371000*math.atan2(math.sqrt(sum(x*x for x in cross)),dot)
 error=abs(metres-reference);worst_distance=max(worst_distance,error)
 assert error<3,(coords,metres,reference)
 if bearing!=0xffffffff and 50<reference<math.pi*6371000-50:
  expected=math.degrees(math.atan2(math.sin(lon2-lon1)*math.cos(lat2),math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lon2-lon1)))%360
  error=abs((bearing/10-expected+180)%360-180);worst_bearing=max(worst_bearing,error)
  assert error<0.2,(coords,bearing,expected)
print(f'ARM GPS geometry: {len(points)} spherical oracle pairs, poles/dateline/coincident/antipodal; max error {worst_distance:.3f} m / {worst_bearing:.3f} deg PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['compact_storage_tests']|1,0x080ff000,count=30000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'compact storage failed at C++ line {result}'
print('ARM compact storage: lossless fields, legacy migration, bounded writes, CRC/varint rejection, atomic restore and 120 interrupted semantic-slot saves PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['reset_defaults_tests']|1,0x080ff000,count=5000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'reset defaults failed at C++ line {result}'
print('ARM reset defaults: cancel/confirm/timeout/wrap, memory retention vs factory presets, language/RF points retained, TX-page exclusion and compact reload PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['settings_export_tests']|1,0x080ff000,count=12000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'settings export failed at C++ line {result}'
print('ARM settings export: explicit start, one read command, exact snapshot/CRC, 32-byte work bound, TC drain, timeout/wrap, cancellation and PTT return PASS')
import importlib.util,tempfile
spec=importlib.util.spec_from_file_location('settings_backup',r/'settings_backup.py');backup=importlib.util.module_from_spec(spec);spec.loader.exec_module(backup)
fixture_size=struct.unpack('<I',u.mem_read(symbols['backup_fixture_size'],4))[0]
fixture=bytes(u.mem_read(symbols['regression_fixture'],fixture_size))
assert backup.inspect_backup(fixture)['occupied_channels']==64
expected_version=(r/'firmware_version.hpp').read_text(encoding='utf-8').split('"')[1]
assert backup.inspect_backup(fixture)['firmware']==expected_version
legacy_version=bytearray(fixture)
legacy_version[24:40]=b'0.68'.ljust(16,b'\0')
import zlib
struct.pack_into('<I',legacy_version,44,zlib.crc32(legacy_version[:44]))
assert backup.inspect_backup(legacy_version)['firmware']=='0.68'
from import_oem_channels import CHANNEL_BYTES, merge
dump=bytearray(b'\xff'*CHANNEL_BYTES)
dump[-32:]=struct.pack('<IIHHBBBBI12s',0x44608125,0x44600625,885,106,0,0,2,0x4a,0,b'Test OEM')
for original in (fixture,):
 converted=merge(dump,original,[(990,64)])
 assert backup.inspect_backup(converted)['occupied_channels']==64
 frame=converted[48:]
 u.mem_write(symbols['regression_fixture'],frame)
 for reg,value in ((UC_ARM_REG_R0,symbols['regression_fixture']),(UC_ARM_REG_R1,len(frame)),(UC_ARM_REG_R2,backup.inspect_backup(converted)["snapshot_schema"]),
                   (UC_ARM_REG_SP,0x20017000),(UC_ARM_REG_LR,0x080ff001)):
  u.reg_write(reg,value)
 u.emu_start(symbols['oem_channel_import_probe']|1,0x080ff000,count=5000000)
 assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
 result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'OEM import failed at C++ line {result}'
u.mem_write(symbols['regression_fixture'],fixture)
for mapping in ([(989,1)],[(990,65)],[(990,1),(990,1)]):
 try:merge(dump,fixture,mapping)
 except ValueError:pass
 else:raise AssertionError('invalid OEM mapping accepted')
dump[-17]|=1
try:merge(dump,fixture,[(990,1)])
except ValueError:pass
else:raise AssertionError('unsupported OEM AM accepted')
print('OEM channel import: host conversion to real ARM snapshot reader, split, tones, zone and TX inhibit PASS')
for index in (0,8,12,16,20,24,40,44,48,len(fixture)-1):
 damaged=bytearray(fixture);damaged[index]^=128
 try:backup.inspect_backup(damaged)
 except ValueError:pass
 else:raise AssertionError('backup corruption accepted')
class FakeSerial:
 def __init__(self,data):self.data=data;self.writes=[];self.time=0
 def clock(self):self.time+=0.001;return self.time
 def write(self,data):self.writes.append(data);return len(data)
 def read(self,size):result=self.data[:min(size,7)];self.data=self.data[len(result):];return result
fake=FakeSerial(b'noise RT950CFG'+bytes(40)+fixture)
assert backup.read_backup(fake,clock=fake.clock)==fixture
assert fake.writes and all(request==backup.REQUEST for request in fake.writes)
for broken in (b'',fixture[:-20]):
 fake=FakeSerial(broken)
 try:backup.read_backup(fake,wait_seconds=0.1,transfer_seconds=2,clock=fake.clock)
 except TimeoutError:pass
 else:raise AssertionError('truncated/absent export accepted')
with tempfile.TemporaryDirectory(prefix='rt950-backup-test-') as folder:
 path=Path(folder)/'settings.rtcfg';backup.write_backup(path,fixture)
 assert path.read_bytes()==fixture
 assert backup.load_backup(path)==fixture
 path.write_bytes(fixture+b'extra')
 try:backup.load_backup(path)
 except ValueError:pass
 else:raise AssertionError('oversize file accepted')
 path.write_bytes(fixture)
 try:backup.write_backup(path,fixture)
 except FileExistsError:pass
 else:raise AssertionError('backup overwritten')
print('HOST settings backup: actual ARM frame, noise/partial reads, CRC rejection, bounded timeout, exclusive output and request-only serial PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['settings_import_tests']|1,0x080ff000,count=15000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'settings import failed at C++ line {result}'
print('ARM settings import: paced 32-byte receive, TC drain, CRC/semantic/length rejection, explicit confirm/cancel/PTT, RF point retention, timeout/wrap and compact reload PASS')
class UploadSerial:
 def __init__(self,frame):self.frame=frame;self.answer=b'';self.writes=[];self.at=-1;self.time=0
 def clock(self):self.time+=0.001;return self.time
 def read(self,size):result=self.answer[:min(size,2)];self.answer=self.answer[len(result):];return result
 def write(self,data):
  assert not self.answer,'host must wait for the complete acknowledgement'
  self.writes.append(data)
  if self.at==-1:
   assert data==b'RT950PUT\n';self.answer=b'READY\n';self.at=0
  elif not self.at:
   assert data==self.frame[:48];self.answer=b'NEXT\n';self.at=48
  else:
   assert 0<len(data)<=32 and data==self.frame[self.at:self.at+len(data)]
   self.at+=len(data);self.answer=b'CHECKED\n' if self.at==len(self.frame) else b'NEXT\n'
  return len(data)
fake=UploadSerial(fixture);backup.upload_backup(fake,fixture,clock=fake.clock)
assert fake.at==8140 and b''.join(fake.writes[1:])==fixture
fake=UploadSerial(fixture)
try:backup.upload_backup(fake,fixture[:-1],clock=fake.clock)
except ValueError:pass
else:raise AssertionError('invalid file uploaded')
assert not fake.writes
fake=FakeSerial(b'')
try:backup.upload_backup(fake,fixture,wait_seconds=.1,clock=fake.clock)
except TimeoutError:pass
else:raise AssertionError('missing device acknowledgement ignored')
assert fake.writes==[b'RT950PUT\n']
fake=FakeSerial(b'ERROR\n')
try:backup.upload_backup(fake,fixture,clock=fake.clock)
except ValueError:pass
else:raise AssertionError('device rejection ignored')
print('HOST settings upload: ARM-exported file, complete paced round trip, partial acknowledgements, preflight rejection, device error and bounded timeout PASS')
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(symbols['user_key_tests']|1,0x080ff000,count=6000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
result=u.reg_read(UC_ARM_REG_R0);assert result==0,f'user keys failed at C++ line {result}'
print('ARM user keys: four bindings, release-only short/700ms hold/wrap, PTT-C priority, editor/lock guards, reset, compact v1/v2/v3 and portable schema1/2 migration PASS')
legacy=bytearray(fixture[:48+7992])
import zlib
struct.pack_into('<I',legacy,12,7992);struct.pack_into('<I',legacy,16,1)
struct.pack_into('<I',legacy,20,zlib.crc32(legacy[48:]));struct.pack_into('<I',legacy,44,zlib.crc32(legacy[:44]))
assert backup.inspect_backup(legacy)['snapshot_schema']==1
fake=UploadSerial(legacy);backup.upload_backup(fake,legacy,clock=fake.clock);assert fake.at==8040
assert backup.inspect_backup(fixture)['snapshot_schema']==3
v2=bytearray(fixture[:48+8008])
struct.pack_into('<II',v2,12,8008,2)
struct.pack_into('<I',v2,20,zlib.crc32(v2[48:]));struct.pack_into('<I',v2,44,zlib.crc32(v2[:44]))
assert backup.inspect_backup(v2)['snapshot_schema']==2
fake=UploadSerial(v2);backup.upload_backup(fake,v2,clock=fake.clock);assert fake.at==8056
scan_backup=bytearray(fixture)
struct.pack_into('<4I',scan_backup,48+7992,8,9,2,11)
struct.pack_into('<I',scan_backup,20,zlib.crc32(scan_backup[48:]))
struct.pack_into('<I',scan_backup,44,zlib.crc32(scan_backup[:44]))
assert backup.inspect_backup(scan_backup)['snapshot_schema']==3
fake=UploadSerial(scan_backup);backup.upload_backup(fake,scan_backup,clock=fake.clock);assert fake.at==8140
print('HOST portable migration: schema1/2 upload retained and schema3 8140-byte container validated PASS')
print('ARM APRS background: idle gate, 10 s rest, 1.2 s window, cancellation and tick wrap PASS')
print('ARM APRS beacon: uncompressed position, manual permit, RF/DAC cancellation and DAC failure PASS')
print('ARM APRS TX framing: bounded NRZI encoder, FCS and round trip PASS')
print('ARM APRS stations: SSID identity, refresh, capacity and malformed position PASS')
print('ARM APRS audio RX: Bell 202, four sample phases, DC offset, noise and position PASS')
print('ARM APRS RX: HDLC framing, bit stuffing, FCS rejection, overflow recovery and KISS export PASS')
print('ARM voice TX calibration: read-only SPI frame, timeout deselect, factory tables and erased marker PASS')
print('ARM integrated UHF TX model: selected frequency, busy channel, setup fault, calibrated key and bias-off before RX PASS')
print('ARM voice PTT: held on startup, authorization, busy lock, startup timeout, TOT and release PASS')
print('ARM side PTT: target C, primary priority, no automatic rekey and snapshot PASS')
print('ARM channel/VFO scan: modes, direction, limits, wrap, recall and snapshot PASS')
print('ARM Watch slots: pair selection, cancellation, active slot, PTT priority and snapshot PASS')
print('ARM Watch timing: editors, invalid/cancel, custom dwell, zero hold and snapshot PASS')
print('ARM voice: 24-bit write, directional read, A/B/C dwell, carrier hold, PTT target, single slot and disable PASS')
print('ARM shortwave AM: startup, 7100 kHz, tune status, RSSI/SNR and mode switch PASS')
print('ARM FM: tune 100.30, matching reply, stale RSSI, timeout, NACK, stuck clock and quality PASS')
print('ARM C++ regression logic: invalid 12345, 6/8/4 digit commit, independent slots, 12.5kHz step, draft/menu, GPS parent, VM RAM/list/restore, diagnostics, long OK, 2-second notices with uint32 wrap, 64 RX presets, overwrite/cancel, VM short/hold, display timeout/editor, RMC fresh/stale/course/date rollover PASS')


if a.stock:
 import hashlib,random
 stock=a.stock.read_bytes();assert hashlib.sha256(stock).hexdigest()=='025bc583802a96911836e274cdfdf3ce05d9919bc77451b7f7f1573fb5571907'
 oracle=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);oracle.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
 oracle.mem_map(0x08000000,0x100000);oracle.mem_map(0x20000000,0x18000);oracle.mem_write(0x08003000,stock)
 assert stock[0x2347c:0x23488]==bytes.fromhex('014951f820007047a40c0308')
 for channel in range(10):
  frequencies=[]
  for machine,pc,scale in [(u,symbols['weather_frequency_probe'],1),(oracle,0x0802647c,10)]:
   machine.reg_write(UC_ARM_REG_R0,channel)
   machine.reg_write(UC_ARM_REG_LR,0x080ff001)
   machine.emu_start(pc|1,0x080ff000,count=100)
   assert machine.reg_read(UC_ARM_REG_PC)==0x080ff000
   frequencies.append(machine.reg_read(UC_ARM_REG_R0)*scale)
  assert frequencies[0]==frequencies[1],(channel,frequencies)
 print('OEM 0.29 NOAA: all ten frequencies in original order match executed ARM lookup PASS')
 gps_entries=((0x08036977,0x0801859c,0x0801f970),(0x08036998,0x080185a8,0x0801f9b4),(0x080369b5,0x080199fc,0x08021858),
              (0x080369d6,0x0801922c,0x08020dd0),(0x080369f7,0x08018e88,0x08020aa8),
              (0x08036a18,0x08018510,0x0801f8e0),(0x08036a39,0x08019924,0x08021804))
 for entry,editor,save in gps_entries:
  assert struct.unpack_from('<2I',stock,entry-0x08003000+(21 if entry<=0x08036998 else 25))==(editor|1,save|1)
 assert stock[0x3d12:0x3d1c]==bytes.fromhex('97f84d0001280ad11e33')
 for hour in range(24):
  for offset in range(-12,15):
   oracle.reg_write(UC_ARM_REG_R0,hour);oracle.reg_write(UC_ARM_REG_R1,offset+12)
   oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
   oracle.emu_start(0x08008e79,0x080ff000,count=100)
   assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
   assert oracle.reg_read(UC_ARM_REG_R0)==(hour+offset)%24
 print('OEM 0.29 GPS: menu pointers, half-hour add instruction and 648 hour/zone rollovers PASS')
 gps=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS);gps.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
 gps.mem_map(0x08000000,0x100000);gps.mem_map(0x20000000,0x18000);gps.mem_write(0x08003000,stock)
 gps.reg_write(UC_ARM_REG_C1_C0_2,0xf00000);gps.reg_write(UC_ARM_REG_FPEXC,1<<30)
 # Supply the C runtime character classes used by the OEM numeric parser.
 classes=bytearray(256)
 for char in b' \t\r\n':classes[char]=1
 for char in b'0123456789':classes[char]=8
 gps.mem_write(0x20010000,bytes(classes));gps.mem_write(0x20010f3c,struct.pack('<I',0x20010000))
 def gps_run(start,end,*args):
  gps.reg_write(UC_ARM_REG_SP,0x20017000);gps.reg_write(UC_ARM_REG_LR,0x080ff001)
  for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),args):gps.reg_write(reg,value)
  gps.emu_start(start|1,end,count=100000)
  assert gps.reg_read(UC_ARM_REG_PC)==end
 assert stock[0xf9a0:0xf9a8]==bytes.fromhex('ea10030824ae0020')
 defaults=stock[0x2e0ea:0x2e0ea+121]
 fields=(1,2,3,4,5,6,0x4d)
 assert bytes(defaults[i] for i in fields)==bytes((0,0,0,0,0,21,0))
 gps_run(0x0801298c,0x08012998)
 assert gps.mem_read(0x2000ae24,121)==defaults
 def gps_spi_read(machine,address,size,_):
  source,target,length=(machine.reg_read(reg) for reg in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2))
  assert (source,length) in ((0x10000,8),(0x10008,128))
  machine.mem_write(target,b'\xff'*length)
  machine.reg_write(UC_ARM_REG_PC,machine.reg_read(UC_ARM_REG_LR))
 hook=gps.hook_add(UC_HOOK_CODE,gps_spi_read,begin=0x08024424,end=0x08024424)
 gps.mem_write(0x2000ae24,bytes(121))
 gps_run(0x08012018,0x080120c2)
 assert gps.mem_read(0x2000ae24,121)==defaults
 gps.hook_del(hook)
 gps_run(0x08008e78,0x080ff000,12,defaults[6])
 assert gps.reg_read(UC_ARM_REG_R0)==21 # 12:00 UTC + 9 with the stock default.
 assert stock[0x63c4:0x63d0]==bytes.fromhex('fff77efd00281bd01048e522')
 assert stock[0xfddc:0xfde8]==bytes.fromhex('a8084ff4983101eb00380822')
 record=bytearray(229)
 payload=b'!4903.50S/07201.75W> TEST GPS /A=001234\0'
 record[101:101+len(payload)]=payload
 gps.mem_write(0x2000aa04,bytes(record));gps.mem_write(0x2000a9ef,b'\1')
 gps_run(0x080093a4,0x080093e0)
 position=bytes.fromhex('505327133257211c4b7801000000000000000000')
 assert gps.mem_read(0x2000ad98,20)==position
 assert gps.mem_read(0x2000acb3+101,len(payload))==payload
 gps.reg_write(UC_ARM_REG_R9,1)
 gps_run(0x08007f04,0x08007f56)
 assert gps.mem_read(0x20017010,20)==position
 def gps_read_aprs_record(machine,address,size,_):
  index,target,length=(machine.reg_read(reg) for reg in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2))
  assert index==7 and length==205
  machine.mem_write(target,bytes(record[:length]))
  machine.reg_write(UC_ARM_REG_PC,machine.reg_read(UC_ARM_REG_LR))
 hook=gps.hook_add(UC_HOOK_CODE,gps_read_aprs_record,begin=0x08012110,end=0x08012110)
 gps_index=struct.unpack_from('<I',stock,0x52e0)[0]
 gps.mem_write(gps_index+4,b'\0');gps.mem_write(gps_index+967,b'\7')
 gps.reg_write(UC_ARM_REG_R9,0)
 gps_run(0x08007f04,0x08007f56)
 assert gps.mem_read(0x20017010,20)==position
 gps.hook_del(hook)
 print('OEM 0.29 GPS distance: factory reset and failed CRC defaults, APRS RX writer, last and indexed target branches PASS')
 rng=random.Random(950)
 for case in range(64):
  payload=bytes(rng.randrange(256) for _ in range(case*139//63))
  answers=[]
  for machine,pc in [(u,symbols['oem_crc_probe']),(oracle,0x0800d478)]:
   if payload:machine.mem_write(0x20010000,payload)
   machine.reg_write(UC_ARM_REG_R0,0x20010000);machine.reg_write(UC_ARM_REG_R1,len(payload));machine.reg_write(UC_ARM_REG_SP,0x20017000);machine.reg_write(UC_ARM_REG_LR,0x080ff001)
   machine.emu_start(pc|1,0x080ff000,count=200000)
   assert machine.reg_read(UC_ARM_REG_PC)==0x080ff000
   answers.append(machine.reg_read(UC_ARM_REG_R0))
  assert answers[0]==answers[1],(case,answers)
 print('OEM control checksum C++ vs raw 0.29: 64 payloads PASS; not AX.25 FCS')
 # Run the OEM FM signal wrapper with only its transfer subroutine replaced.
 calls=[]
 def fm_transfer(machine,address,size,_):
  regs=[machine.reg_read(reg) for reg in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3)]
  length,command,reply_length,reply=regs
  calls.append((bytes(machine.mem_read(command,length)),reply_length))
  if reply_length:
   packet={0x14: bytes([0x81]), 0x41: bytes([0x80]), 0x22: bytes([0x80,0,0x27,0x2e,48,19,0]),
           0x23: bytes([0x80,0,1,0x80,48,19,0,0]),
           0x43: bytes([0x80,0,1,0,42,18])}[calls[-1][0][0]]
   assert len(packet)==reply_length
   machine.mem_write(reply,packet)
  machine.reg_write(UC_ARM_REG_PC,machine.reg_read(UC_ARM_REG_LR))
 oracle.hook_add(UC_HOOK_CODE,fm_transfer,begin=0x0802a3c6,end=0x0802a3c6)
 oracle.reg_write(UC_ARM_REG_SP,0x20017000);oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
 oracle.emu_start(0x08011981,0x080ff000,count=20000)
 assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
 assert calls==[(bytes([0x23,1]),8)],calls
 assert bytes(oracle.mem_read(0x2000a388+27,2))==bytes([48,19])
 print('OEM 0.29 FM RSQ: ARM command 23 01, 8-byte response, RSSI/SNR offsets 4/5 PASS')
 oracle.mem_write(0x2000a949+67,bytes([1]))
 calls.clear()
 oracle.reg_write(UC_ARM_REG_SP,0x20017000);oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
 oracle.emu_start(0x08011981,0x080ff000,count=20000)
 assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
 assert calls==[(bytes([0x43,1]),6)],calls
 assert bytes(oracle.mem_read(0x2000a388+27,2))==bytes([42,18])
 oracle.mem_write(0x2000a949+67,bytes(1))
 print('OEM 0.29 AM RSQ: command 43 01, six-byte response and RSSI/SNR offsets PASS')


 # Stub delays and audio routing while executing the OEM tune wrappers.
 def return_from_hook(machine,address,size,_):
  machine.reg_write(UC_ARM_REG_PC,machine.reg_read(UC_ARM_REG_LR))
 for address in (0x0800d906,0x08011930):
  oracle.hook_add(UC_HOOK_CODE,return_from_hook,begin=address,end=address)
 calls.clear()
 oracle.reg_write(UC_ARM_REG_R0,7100)
 oracle.reg_write(UC_ARM_REG_SP,0x20017000);oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
 oracle.emu_start(0x08006861,0x080ff000,count=20000)
 assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
 assert calls==[(bytes([0x40,0x40,0x1b,0xbc,0,1]),0)],calls
 print('OEM 0.29 AM tune: 7100 kHz command and antenna capacitor PASS')
 calls.clear()
 oracle.reg_write(UC_ARM_REG_R0,1);oracle.reg_write(UC_ARM_REG_R1,1)
 oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
 oracle.emu_start(0x08006831,0x080ff000,count=20000)
 assert calls==[(bytes([0x41,0x0c]),1)],calls
 print('OEM 0.29 AM seek: executed command 41 0C, upwards with wrap PASS')
 properties=[]
 def capture_am_property(machine,address,size,_):
  properties.append((address,machine.reg_read(UC_ARM_REG_R0),machine.reg_read(UC_ARM_REG_R1)))
  return_from_hook(machine,address,size,_)
 hooks=[oracle.hook_add(UC_HOOK_CODE,capture_am_property,begin=address,end=address)
        for address in (0x0802a33e,0x0802a420)]
 oracle.mem_write(0x2000a949+67,b'\x01')
 for band,low,high in ((0,153,279),(1,520,1710),(2,2300,30000)):
  properties.clear()
  oracle.mem_write(0x2000a388+1,bytes([band]))
  oracle.reg_write(UC_ARM_REG_SP,0x20017000);oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
  oracle.emu_start(0x08011ac1,0x080ff000,count=2000)
  assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
  assert properties==[(0x0802a33e,low,high),(0x0802a420,0x3402,5 if band==2 else 9)],properties
  calls.clear()
  oracle.mem_write(0x2000019c,bytes([band]))
  oracle.reg_write(UC_ARM_REG_R0,low)
  oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
  oracle.emu_start(0x08006861,0x080ff000,count=20000)
  assert calls==[(bytes([0x40,0x40,low>>8,low&255,0,1]),0)],calls
 for hook in hooks:oracle.hook_del(hook)
 oracle.mem_write(0x2000a949+67,bytes(1))
 print('OEM 0.29 LW/MW/SW: executed band boundaries, spacing and antenna command PASS')
 calls.clear()
 for address in (0x08011b40,0x080150f0,0x080119d4):
  oracle.reg_write(UC_ARM_REG_R0,10030)
  oracle.reg_write(UC_ARM_REG_SP,0x20017000)
  oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
  if address==0x08011b40: # Enter the FM branch after the enclosing PUSH {r4, lr}.
   oracle.mem_write(0x20017000,struct.pack('<II',0,0x080ff001))
  oracle.emu_start(address|1,0x080ff000,count=20000)
  assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
 assert calls==[(bytes.fromhex(cmd),length) for cmd,length in (
  ('12 00 14 00 19 64',0),('12 00 14 01 2a 30',0),('20 03 27 2e 00',0),
  ('14',1),('22 03',7))],calls
 assert bytes(oracle.mem_read(0x2000a388+2,2))==struct.pack('<H',10030)
 print('OEM 0.29 FM ARM: band edges, tune 20 03, GET_INT_STATUS, tune status 22 03 / 7 bytes PASS')

 # The writer takes a prepared tuning word and dispatches two register writes.
 writes=[]
 def capture_rf(machine,address,size,_):
  if address==0x080fe000:
   writes.append((machine.reg_read(UC_ARM_REG_R0),machine.reg_read(UC_ARM_REG_R1)))
  machine.reg_write(UC_ARM_REG_PC,machine.reg_read(UC_ARM_REG_LR))
 oracle.mem_write(0x20000c78,struct.pack('<I',0x080fe001))
 oracle.hook_add(UC_HOOK_CODE,capture_rf,begin=0x080fe000,end=0x080fe000)
 mode_stub=oracle.hook_add(UC_HOOK_CODE,capture_rf,begin=0x0801ed50,end=0x0801ed50)
 tuning_word=44608125
 oracle.reg_write(UC_ARM_REG_R0,tuning_word)
 oracle.reg_write(UC_ARM_REG_SP,0x20017000);oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
 oracle.emu_start(0x0801f0f1,0x080ff000,count=20000)
 assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
 assert writes==[(0x38,tuning_word&65535),(0x39,tuning_word>>16)],writes
 print('OEM 0.29 voice tuning word: low/high halves via registers 38/39 PASS')

 # Capture the OEM write pins with GPIO and the short delay replaced.
 pins={0x100: True, 0x8000: True, 0x400: False, 0x800: False}
 wire=[]
 def capture_voice_gpio(machine,address,size,_):
  port=machine.reg_read(UC_ARM_REG_R0)
  mask=machine.reg_read(UC_ARM_REG_R1)
  assert port==0x40011800 and mask in pins,(hex(port),hex(mask))
  pins[mask]=address==0x080156e6
  if mask==0x400 and pins[mask]:
   assert pins[0x100]!=pins[0x8000]
   wire.append(int(pins[0x800]))
  return_from_hook(machine,address,size,_)
 for address in (0x080156e2,0x080156e6):
  oracle.hook_add(UC_HOOK_CODE,capture_voice_gpio,begin=address,end=address)
 oracle.hook_add(UC_HOOK_CODE,return_from_hook,begin=0x0800d922,end=0x0800d922)
 for address in (0x0801f440,0x0801f494):
  wire.clear()
  oracle.reg_write(UC_ARM_REG_R0,0x38)
  oracle.reg_write(UC_ARM_REG_R1,0x1234)
  oracle.reg_write(UC_ARM_REG_SP,0x20017000)
  oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
  oracle.emu_start(address|1,0x080ff000,count=20000)
  assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
  assert ''.join(map(str,wire))==format(0x381234,'024b')
  assert pins[0x100] and pins[0x8000] and not pins[0x400]
 print('OEM 0.29 voice serial: both selects and 24-bit MSB-first write verified in ARM')

 # Run stock receiver startup with only chip selection and register I/O mocked.
 import re
 startup=[]
 def capture_init_write(machine,address,size,_):
  startup.append((machine.reg_read(UC_ARM_REG_R0),machine.reg_read(UC_ARM_REG_R1)))
  return_from_hook(machine,address,size,_)
 def mock_init_read(machine,address,size,_):
  register=machine.reg_read(UC_ARM_REG_R0)
  assert register in (0x31,0x40,0x52,0x7d)
  machine.reg_write(UC_ARM_REG_R0,{0x31:0x33,0x40:0xa800,0x52:0x6fff,0x7d:0xe912}[register])
  return_from_hook(machine,address,size,_)
 oracle.hook_add(UC_HOOK_CODE,return_from_hook,begin=0x0800d930,end=0x0800d930)
 oracle.hook_add(UC_HOOK_CODE,mock_init_read,begin=0x080fe100,end=0x080fe100)
 oracle.hook_add(UC_HOOK_CODE,capture_init_write,begin=0x080fe104,end=0x080fe104)
 oracle.mem_write(0x20000c74,struct.pack('<II',0x080fe101,0x080fe105))
 oracle.mem_write(0x20000c91,bytes([0]))
 oracle.reg_write(UC_ARM_REG_R0,0)
 oracle.reg_write(UC_ARM_REG_SP,0x20017000);oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
 oracle.emu_start(0x0800a029,0x080ff000,count=20000)
 assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
 table=(r/'voice_rx.hpp').read_text()
 def fixed(name):
  block=table.split(name+'[]={',1)[1].split('};',1)[0]
  return [(int(reg,16),int(value,16)) for reg,value in re.findall(r'{(0x[0-9a-f]+),(0x[0-9a-f]+|0)}',block)]
 expected=fixed('rx_startup')+[(0x40,0xa4d2)]+fixed('rx_finish')
 assert startup[:-1]==expected and startup[-1][0]==0x48,(startup,expected)
 print('OEM 0.29 voice startup: register order and preserved register 40 bits PASS')

 # Resolve the band-to-RX-route path from the same raw binary.
 routes=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
 routes.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
 routes.mem_map(0x08000000,0x100000)
 routes.mem_map(0x20000000,0x18000)
 routes.mem_write(0x08003000,stock)
 changes=[]
 def capture_route(machine,address,size,_):
  changes.append((machine.reg_read(UC_ARM_REG_R0),machine.reg_read(UC_ARM_REG_R1),address==0x080156e6))
  return_from_hook(machine,address,size,_)
 for address in (0x080156e2,0x080156e6):
  routes.hook_add(UC_HOOK_CODE,capture_route,begin=address,end=address)
 def run_route(address,first,second):
  for reg,value in ((UC_ARM_REG_R0,first),(UC_ARM_REG_R1,second),
                    (UC_ARM_REG_SP,0x20017000),(UC_ARM_REG_LR,0x080ff001)):
   routes.reg_write(reg,value)
  routes.emu_start(address|1,0x080ff000,count=2000)
  assert routes.reg_read(UC_ARM_REG_PC)==0x080ff000
 for mhz,expected in ((145,0),(446,1)):
  run_route(0x080160cc,mhz*10,0)
  route=routes.mem_read(0x2000a468+0x10a,1)[0]
  assert route==expected
  changes.clear()
  run_route(0x0801d5a4,route,1)
  expected_changes=[(0x40011800,0x2000,False)]
  expected_changes+=([(0x40010c00,1,False),(0x40011800,0x1000,True)] if route else
                     [(0x40011800,0x1000,False),(0x40010c00,1,True)])
  assert changes==expected_changes,(mhz,changes)
 print('OEM 0.29 RX route: VHF and UHF GPIO sequence PASS')
 for alternate,expected in ((0,(4,1)),(1,(0,0))):
  routes.mem_write(0x2000a3fc+98,bytes([alternate]))
  run_route(0x080160cc,270,0)
  cb_rx_route=routes.mem_read(0x2000a468+0x10a,1)[0]
  cb_rx_chip=routes.mem_read(0x2000a468+0x10d,1)[0]
  assert (cb_rx_route,cb_rx_chip)==expected
  changes.clear()
  run_route(0x0801d5a4,cb_rx_route,1)
  if alternate:
   assert changes==[(0x40011800,0x2000,False),(0x40011800,0x1000,False),(0x40010c00,1,True)]
  else:
   assert changes==[(0x40011800,0x1000,False),(0x40010c00,1,False),(0x40011800,0x2000,True)]
 routes.mem_write(0x2000a3fc+98,bytes(1))
 print('OEM 0.29 27 MHz RX: both route/chip selections and GPIO sequences PASS')
 for hz,accepted in ((17999990,0),(18000000,1),(25000000,1),(27185000,1),(29999990,1),(63999990,1),(64000000,0)):
  run_route(0x0800bfdc,hz//10,0)
  assert routes.reg_read(UC_ARM_REG_R0)==accepted,(hz,accepted)
 print('OEM 0.29 low RF gate: 18 MHz inclusive to 64 MHz exclusive with low-route flag PASS')
 for alternate in (0,1):
  routes.mem_write(0x2000a3fc+98,bytes([alternate]))
  previous=None
  intervals=[]
  for frequency in range(10,301):
   routes.mem_write(0x2000a468+0x10a,bytes([255]*4))
   run_route(0x080160cc,frequency,0)
   route=routes.mem_read(0x2000a468+0x10a,4)
   pair=(route[0],route[3])
   if pair!=previous:
    intervals.append((frequency,pair))
    previous=pair
  print(f'OEM 0.29 1-30 MHz RX route transitions (0.1 MHz, alternate={alternate}): {intervals}')
 routes.mem_write(0x2000a3fc+98,bytes(1))

 # The same stock AF selector is used for mute and for opening voice audio.
 for mode,expected in ((0,0x6042),(1,0x6142)):
  startup.clear()
  oracle.reg_write(UC_ARM_REG_R0,mode)
  oracle.reg_write(UC_ARM_REG_SP,0x20017000);oracle.reg_write(UC_ARM_REG_LR,0x080ff001)
  oracle.emu_start(0x0801efb1,0x080ff000,count=2000)
  assert oracle.reg_read(UC_ARM_REG_PC)==0x080ff000
  assert startup[0]==(0x47,expected),startup
 print('OEM 0.29 voice AF selector: register 47 mute and demodulated audio PASS')

 # Compare the new TX value calculations against stock routines with
 # synthetic calibration data and a mocked serial register interface.
 def call_tx(machine,pc,*values,budget=60000):
  for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),values):
   machine.reg_write(reg,value&0xffffffff)
  machine.reg_write(UC_ARM_REG_SP,0x20017000)
  machine.reg_write(UC_ARM_REG_LR,0x080ff001)
  try:machine.emu_start(pc|1,0x080ff000,count=budget)
  except UcError as error:raise AssertionError(f"{pc:#x} stopped at {machine.reg_read(UC_ARM_REG_PC):#x}") from error
  assert machine.reg_read(UC_ARM_REG_PC)==0x080ff000,(hex(pc),hex(machine.reg_read(UC_ARM_REG_PC)),budget)
  return machine.reg_read(UC_ARM_REG_R0)
 # Run the stock frequency counter and spectrum helpers against mocked RF reads.
 reads=[]
 def counter_read(machine,address,size,_):
  reg=machine.reg_read(UC_ARM_REG_R0)
  reads.append(reg)
  machine.reg_write(UC_ARM_REG_R0,counter_values[reg])
  return_from_hook(machine,address,size,_)
 oracle.mem_write(0x20000c74,struct.pack('<I',0x080fe109))
 counter_hook=oracle.hook_add(UC_HOOK_CODE,counter_read,begin=0x080fe108,end=0x080fe108)
 startup.clear()
 call_tx(oracle,0x0801e94c)
 call_tx(oracle,0x0801e93c)
 assert startup==[(0x32,0x8245),(0x32,0x8244)]
 counter_values={0xd:0x8100,0xe:0xfedc,0x65:0xfe55,0x67:0xfe34}
 assert call_tx(oracle,0x0801e5f0)==0 and reads==[0xd]
 counter_values[0xd]=0x31ab
 reads.clear()
 assert call_tx(oracle,0x0801e5f0)==0x1abfedc and reads==[0xd,0xe]
 reads.clear()
 assert call_tx(oracle,0x08023d70)==0x34 and call_tx(oracle,0x08023d5c)==0x55
 assert reads==[0x67,0x65]
 # Run state 1 for three VHF samples through the 500 Hz boundary and state 2 rounding.
 oracle.mem_write(0x2000a3fd,bytes([4]))
 state=0x2000b208
 oracle.mem_write(state,bytes([1])+bytes(39))
 startup.clear()
 for word in (14550005,14550055,14550006):
  counter_values.update({0xd:word>>16,0xe:word&0xffff})
  call_tx(oracle,0x08022510)
 assert startup==[(0x32,0x8244),(0x32,0x8245)]*2+[(0x32,0x8244)]
 assert oracle.mem_read(state,1)==bytes([2])
 assert struct.unpack('<I',oracle.mem_read(state+4,4))[0]==14550030
 tuner=oracle.hook_add(UC_HOOK_CODE,return_from_hook,begin=0x0801e6f4,end=0x0801e6f4)
 drawing=oracle.hook_add(UC_HOOK_CODE,return_from_hook,begin=0x08022408,end=0x08022408)
 call_tx(oracle,0x08022510)
 assert struct.unpack('<I',oracle.mem_read(state+4,4))[0]==14550025
 oracle.mem_write(0x2000a3fd,bytes(1))
 oracle.hook_del(tuner);oracle.hook_del(drawing)
 oracle.hook_del(counter_hook)
 oracle.mem_write(0x20000c74,struct.pack('<I',0x080fe101))
 print('OEM 0.29 RX frequency counter: enable/disable, ready bit, 27-bit word, 500 Hz boundary and grid; spectrum RSSI and noise register masks PASS')
 # Battery classification and its TX consumer from 0.29.
 sample=[0]
 def battery_read(machine,address,size,data):
  machine.reg_write(UC_ARM_REG_R0,sample[0])
  return_from_hook(machine,address,size,data)
 battery_hook=oracle.hook_add(UC_HOOK_CODE,battery_read,begin=0x08016988,end=0x08016988)
 for factory,thresholds in enumerate((bytes([129,135,142,147,152,160,187]),
                                      bytes([127,133,145,154,162,170,187]))):
  oracle.mem_write(0x2000a328+5,thresholds)
  for sample[0] in range(256):
   call_tx(oracle,0x08009b84)
   allowed=call_tx(oracle,0x0800b91c)
   assert allowed==call_tx(u,symbols['battery_tx_limit_probe'],sample[0],factory),(sample[0],factory)
 oracle.hook_del(battery_hook)
 print('OEM 0.29 battery TX gate: all ADC values, factory and fallback thresholds PASS')
 oracle.mem_write(0x200000e4,bytes([0]+[80+2*i for i in range(1,10)]))
 oracle.mem_write(0x200000f1,bytes([0]+[73+2*i for i in range(1,10)]))
 oracle.mem_write(0x200000fe,bytes(range(3,16)))
 oracle.mem_write(0x2000010e,bytes(range(2,12)))
 for hz in (136000000,145500000,174000000,400000000,446081250,470000000):
  oracle.mem_write(0x20000c7c,struct.pack('<I',hz//10))
  for level in range(1,10):
   oracle.mem_write(0x20000c7c+22,bytes([level]))
   startup.clear()
   call_tx(oracle,0x0801edd0)
   assert startup[0]==(0x78,call_tx(u,symbols['voice_sql_probe'],hz,level)),(hz,level,startup)
 oracle.mem_write(0x20000c7c+22,bytes(1))
 print('OEM 0.29 SQL: 54 calibrated VHF/UHF threshold pairs PASS')
 oracle.mem_write(0x2000a8fc,bytes([3]))
 read_pointer=bytes(oracle.mem_read(0x20000c74,4))
 oracle.mem_write(0x20000c74,struct.pack('<I',0x080fe201))
 def sql_read(machine,address,size,data):
  assert machine.reg_read(UC_ARM_REG_R0)==0x0c
  machine.reg_write(UC_ARM_REG_R0,word)
  return_from_hook(machine,address,size,data)
 read_hook=oracle.hook_add(UC_HOOK_CODE,sql_read,begin=0x080fe200,end=0x080fe200)
 for word in (0,1,2,3,0x0281,0x0203,0x0400,0x8002,0xffff):
  active=call_tx(u,symbols['voice_sql_status_probe'],word)
  assert call_tx(oracle,0x0801d51c)==active
  assert call_tx(oracle,0x0801d548)==1-active
 oracle.hook_del(read_hook)
 oracle.mem_write(0x20000c74,read_pointer)
 print('OEM 0.29 SQL: register 0C bit 1 with enabled threshold PASS')

 for source in (0,1,2):
  for gain in (0,17,36,63,255):
   oracle.mem_write(0x2000a91c+18,bytes([source]))
   oracle.mem_write(0x2000011e,bytes([gain]))
   startup.clear()
   call_tx(oracle,0x0801e4d0)
   assert startup==[(0x7d,call_tx(u,symbols['voice_tx_mic_probe'],0xe912,source,gain))]
 for correction in (-1200,-31,0,31,1200):
  oracle.mem_write(0x200000d4+85,bytes([0xa0])+struct.pack('<h',correction))
  startup.clear()
  call_tx(oracle,0x0801e510,44608125)
  word=call_tx(u,symbols['voice_tx_freq_probe'],44608125,correction)
  assert startup[:2]==[(0x38,word&0xffff),(0x39,word>>16)],(correction,startup,word)
 for level in range(3):
  oracle.mem_write(0x20000c7c+23,bytes([level]))
  pointer=0x20012000+16*level
  oracle.mem_write(0x200000d4+104+4*level,struct.pack('<I',pointer))
  oracle.mem_write(pointer,bytes(range(16*level,16*level+8)))
  for hz in (400000000,409999990,410000000,446081250,470000000):
   oracle.mem_write(0x20000c7c+12,struct.pack('<I',hz//10))
   actual=call_tx(oracle,0x0801ea60)
   assert actual==16*level+call_tx(u,symbols['voice_tx_pa_probe'],hz),(level,hz,actual)
 oracle.mem_map(0x40007000,0x1000)
 call_tx(oracle,0x0801e2c4,0x5a)
 assert oracle.mem_read(0x4000741c,4)==struct.pack('<I',0x5a)
 print('OEM 0.29 voice TX: mic gain, signed correction, UHF PA lookup and DAC2 write PASS')

 # Replay the hardware route with writes captured, never emitted.
 route_pins={(0x40011800,0x80):0,(0x40011800,0x1000):1,
             (0x40011800,0x2000):2,(0x40011800,0x4000):3,
             (0x40010c00,1):4,(0x40010c00,2):5}
 for mode,count in ((0,6),(2,3)):
  changes.clear()
  run_route(0x0801d5a4,1,mode)
  assert len(changes)==count
  expected=0
  for port,pin,high in changes:
   expected=(expected<<4)|(route_pins[(port,pin)]<<1)|high
  assert expected==call_tx(u,symbols['voice_tx_route_probe'],mode)
 print('OEM 0.29 UHF TX: isolated and selected GPIO routes PASS')
 for frequency in (270,1450):
  run_route(0x080168c8,frequency,0)
  route,chip=routes.mem_read(0x2000a468+0x10b,2)
  for mode in (0,2):
   changes.clear()
   run_route(0x0801d5a4,route,mode)
   print(f'OEM 0.29 TX trace: {frequency/10:g} MHz route={route} chip={chip} mode={mode} GPIO={changes}')


 if a.calibration_dump:
  # Run the factory loader up to the end of the calibration reads.
  # The backup is used as a read-only SPI oracle; report no private bytes.
  dump=a.calibration_dump.read_bytes()
  assert len(dump)==0x400000
  factory=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
  factory.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
  factory.mem_map(0x08000000,0x100000)
  factory.mem_map(0x20000000,0x18000)
  factory.mem_write(0x08003000,stock)
  reads=[]
  def factory_read(machine,address,size,_):
   offset,destination,length=(machine.reg_read(reg) for reg in
                              (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2))
   assert 0<=offset<offset+length<=len(dump)
   reads.append((offset,destination,length))
   machine.mem_write(destination,dump[offset:offset+length])
   return_from_hook(machine,address,size,_)
  factory.hook_add(UC_HOOK_CODE,factory_read,begin=0x08024424,end=0x08024424)
  factory.reg_write(UC_ARM_REG_SP,0x20017000)
  factory.emu_start(0x080121fd,0x0801232c,count=10000)
  assert factory.reg_read(UC_ARM_REG_PC)==0x0801232c
  assert reads[0][0]==0xf0e0 and reads[0][2]==1
  for level in range(3):
   offset=0xf000+0x40*level
   dest=0x20000014+0x40*level
   assert (offset,dest,16) in reads
   assert factory.mem_read(dest,13)==dump[offset:offset+13]
   assert all(0<byte<255 for byte in dump[offset:offset+13])
  assert (0xf210,0x2000011e,16) in reads
  assert dump[0xf210]<=63
  print('OEM 0.29 factory loader: supplied backup maps to all three UHF PA tables and mic calibration PASS')

 call_tx(oracle,0x0801e2c4,0x5a)
 call_tx(oracle,0x0801d824)
 assert oracle.mem_read(0x4000741c,4)==bytes(4)
 print('OEM 0.29 voice TX release: DAC2 cleared through 0x0801D824 PASS')

 # Execute the compiled board adapter against a SPI2/GPIO register model.
 spi=0x40003800
 port=0x40010c00
 crm=0x4002101c
 for address in (0x40003000,0x40010000,0x40021000):
  u.mem_map(address,0x1000)
 def word_write(address,value):
  u.mem_write(address,struct.pack('<I',value))
 def word_read(address):
  return struct.unpack('<I',u.mem_read(address,4))[0]
 class FactoryDevice:
  def __init__(self):
   self.selected=False
   self.frames=[]
   self.timeout=False
  def write(self,machine,access,address,size,value,_):
   if address in (port+16,port+20):
    odr=word_read(port+12)
    word_write(port+12,(odr|value) if address==port+16 else (odr&~value))
    if value&0x1000:
     if address==port+20:
      assert not self.selected
      self.frames.append([])
      self.selected=True
     else:self.selected=False
   if address==spi+12:
    assert self.selected and word_read(spi)==0x34f,(self.selected,hex(word_read(spi)),self.frames)
    self.frames[-1].append(value)
  def read(self,machine,access,address,size,value,_):
   if address==spi+8:
    word_write(address,2 if self.timeout or not self.selected or not self.frames[-1] else 3)
   if address==spi+12:
    frame=self.frames[-1]
    reply=0
    if len(frame)>4:
     offset=int.from_bytes(bytes(frame[1:4]),'big')+len(frame)-5
     reply=(dump[offset] if a.calibration_dump else
            6 if offset==0xf0e0 else 55 if offset==0xf210 else
            0xc0-((offset-0xf000)//0x40)*0x20 if 0xf000<=offset<0xf08d else 0x64)
    word_write(address,reply)
 device=FactoryDevice()
 u.hook_add(UC_HOOK_MEM_WRITE,device.write)
 u.hook_add(UC_HOOK_MEM_READ,device.read)
 for address,value in ((port+4,0x44441234),(port+12,0x507a),(crm,4),(spi,0),
                       (spi+4,8),(spi+28,0x800)):
  word_write(address,value)
 read_board=next(value for name,value in symbols.items() if 'read_board_uhf_calibration' in name)
 assert call_tx(u,read_board,0x20012000,0)==1
 expected=((0xf0e0,1),(0xf000,13),(0xf040,13),(0xf080,13),(0xf210,16))
 assert [(int.from_bytes(bytes(f[1:4]),'big'),len(f)-4) for f in device.frames]==list(expected)
 assert all(f[0]==3 for f in device.frames)
 assert not device.selected
 assert (word_read(port+4),word_read(port+12),word_read(crm),word_read(spi),
         word_read(spi+4),word_read(spi+28))==(0x44441234,0x507a,4,0,8,0x800)
 device.timeout=True
 assert call_tx(u,read_board,0x20012000,0)==0
 assert not device.selected and word_read(port+4)==0x44441234 and word_read(crm)==4
 word_write(spi,0x40)
 before=len(device.frames)
 assert call_tx(u,read_board,0x20012000,0)==0 and len(device.frames)==before
 # The updater can leave SPI enabled with stale receive/busy state.
 def reset_spi(machine,access,address,size,value,_):
  if value&0x4000:
   for offset in (0,4,12,28):word_write(spi+offset,0)
   word_write(spi+8,2)
 reset_hook=u.hook_add(UC_HOOK_MEM_WRITE,reset_spi,begin=0x40021010,end=0x40021010)
 word_write(spi,0x34f)
 word_write(spi+4,0x43)
 word_write(spi+8,0x83)
 word_write(spi+28,0xc00)
 initialize=next(value for name,value in symbols.items() if 'initialize_factory_spi' in name)
 call_tx(u,initialize)
 assert word_read(0x40021010)&0x4000==0
 assert word_read(spi)==0 and word_read(spi+4)==0 and word_read(spi+28)==0
 device.timeout=False
 assert call_tx(u,read_board,0x20012000,0)==1
 u.hook_del(reset_hook)
 print('ARM factory SPI: inherited SPE/RXNE/BSY/I2S reset at startup; runtime ownership guard retained PASS')
 print('ARM voice factory board: SPI2 reads actual offsets, ownership, timeout and GPIO restore PASS')

 u.mem_map(0x40007000,0x1000)
 word_write(0x40007400,5)
 word_write(0x4000741c,0x80)
 word_write(0x40010800,0x44444444)
 prepare_dac=next(value for name,value in symbols.items() if 'prepare_board_pa_bias' in name)
 clear_dac=next(value for name,value in symbols.items() if 'clear_board_pa_bias' in name)
 call_tx(u,prepare_dac)
 assert word_read(crm)==(4|(1<<29)) and word_read(0x40007400)==0x10005
 assert word_read(0x4000741c)==0 and word_read(0x40010800)==0x44044444
 word_write(0x4000741c,0x80)
 call_tx(u,clear_dac)
 assert word_read(0x4000741c)==0
 print('ARM voice PA board: DAC2 initialized at zero, DAC1 preserved, zero on stop PASS')

 # Execute the OEM register sequence for untoned simplex UHF, chip 0,
 # using known synthetic calibration and mocked RF bus reads/writes.
 oracle.mem_write(0x20000c7c,bytes(32))
 oracle.mem_write(0x20000c7c+12,struct.pack('<I',44608125))
 oracle.mem_write(0x200000d4+85,bytes(3))
 oracle.mem_write(0x2000a91c+18,bytes([2]))
 oracle.mem_write(0x2000011e,bytes([55]))
 oracle.hook_del(mode_stub)
 delay_trace=[]
 def trace_delay(machine,address,size,_):
  delay_trace.append(machine.reg_read(UC_ARM_REG_R0))
 oracle.hook_add(UC_HOOK_CODE,trace_delay,begin=0x0800d922,end=0x0800d922)

 # The PTT caller passes the band selector derived from frequency / 10000.
 for frequency in (4300,4306,4460,4600):
  oracle.mem_write(0x2000a468+0x10b,b'\xff\xff')
  call_tx(oracle,0x080168c8,frequency)
  assert bytes(oracle.mem_read(0x2000a468+0x10b,2))==bytes([1,1]),frequency
 oracle.mem_write(0x200000d4+116,struct.pack('<3I',0x20000024,0x20000064,0x200000a4))
 oracle.mem_write(0x200000d4+92,struct.pack('<3I',0x2000016c,0x2000017c,0x2000018c))
 for chip,word,narrow in ((0,14550000,0),(0,14550000,1),(1,44608125,0),(1,44608125,1),(1,2718500,1)):
  oracle.mem_write(0x20000c7c+12,struct.pack('<I',word))
  oracle.mem_write(0x20000c7c,bytes([chip]))
  oracle.mem_write(0x20000c7c+21,bytes([narrow]))
  for selector in range(3):
   startup.clear();delay_trace.clear()
   oracle.mem_write(0x20000c7c+23,bytes([selector]))
   call_tx(oracle,0x0801f2a8)
   address=0x20012000
   count=call_tx(u,symbols['voice_tx_register_probe'],word,(narrow<<8)|selector,address)
   packed=struct.unpack('<'+'I'*count,u.mem_read(address,4*count))
   assert [(entry>>16,entry&0xffff) for entry in packed if entry>>16!=0xff]==startup,(chip,word,selector,startup)
   assert [entry&0xffff for entry in packed if entry>>16==0xff]==delay_trace
 for start,end,expected in ((260,279,bytes([4,1])),(1360,1740,b'\x00\x00'),(4000,4700,b'\x01\x01')):
  for frequency in range(start,end+1):
   call_tx(oracle,0x080168c8,frequency)
   assert bytes(oracle.mem_read(0x2000a468+0x10b,2))==expected,frequency
 print('OEM 0.29 VHF/UHF TX: routes and chips verified in 0.1 MHz steps; both register sequences PASS')

 # Board-level UHF PTT: simulate the two serial devices and capture all
 # PA and route writes. A DAC write alone is not evidence of RF power.
 ep=0x40011800
 u.mem_map(0x40011000,0x1000)
 word_write(ep+12,0xffff)
 word_write(port+12,0xffff)
 transfers=[]
 power=[]
 route=[]
 responses={0:(0xa800,0x0280,55),1:(0xa800,0xe912,0x6fff,0x33,0xa800)}
 words=[iter(responses[0]),iter(responses[1])]
 tx_registers=[{},{}]
 bad_readback=None
 pa_routes=[]
 expected_pa_route=None
 bits=[]
 selected_chip=None
 sample_bit=0
 sample_value=0
 def trace_voice_write(machine,access,address,size,value,_):
  global selected_chip,bits,sample_bit,sample_value
  if address==0x4000741c:
   power.append(value)
   if not value:pa_routes.clear()
   else:pa_routes.append((word_read(ep+12)&0x4080,word_read(port+12)&2))
  if address==0x4000741c and value and expected_pa_route is not None:
   assert pa_routes[-1]==expected_pa_route,(pa_routes,expected_pa_route)
  if address==ep+20 and value&0x7080 or address==port+20 and value&3:
   route.append((address,value,word_read(0x4000741c)))
  if address not in (ep+16,ep+20):return
  high=address==ep+16
  prior=word_read(ep+12)
  word_write(ep+12,prior|value if high else prior&~value)
  for chip,mask in enumerate((0x100,0x8000)):
   if not value&mask:continue
   if not high:
    assert selected_chip is None
    selected_chip=chip;bits=[];sample_bit=0
   elif selected_chip==chip:
    if len(bits)==24:
     value24=int(''.join(map(str,bits)),2)
     register,value16=value24>>16,value24&0xffff
     tx_registers[chip][register]=value16
     transfers.append((chip,register,value16,word_read(0x4000741c)))
    selected_chip=None
  if high and value&0x400 and selected_chip is not None:
   bits.append(int(bool(prior&0x800)))
 forbid_active_reads=False
 register_response=None
 def trace_voice_read(machine,access,address,size,value,_):
  global sample_bit,sample_value
  if address!=ep+8 or selected_chip is None:return
  assert not forbid_active_reads or not power or power[-1]==0,'serial read while PA active'
  if not sample_bit:
   register=int(''.join(map(str,bits[:8])),2)&0x7f
   regs=tx_registers[selected_chip]
   if register in (0x30,0x36,0x38,0x39) and regs.get(0x30)==0xc1fe:
    assert not power or power[-1]==0
    sample_value=regs[register] ^ int(register==bad_readback)
   elif register_response is None:sample_value=next(words[selected_chip])
   else:sample_value=register_response(register)
   sample_bit=16
  sample_bit-=1
  word_write(address,((sample_value>>sample_bit)&1)<<11)
 u.hook_add(UC_HOOK_MEM_WRITE,trace_voice_write)
 u.hook_add(UC_HOOK_MEM_READ,trace_voice_read)
 for hz,anchor,cancel in ((145512500,145000000,False),(433075000,433000000,False),(145512500,145000000,True)):
  transfers.clear();power.clear();route.clear()
  samples=iter((0x8000,hz//10>>16,hz//10>>16))
  def register_response(register):
   if register==0x0d:return next(samples)
   return {0x0c:0,0x40:0xa800,0x67:70,0x0e:hz//10&0xffff}[register]
  result=call_tx(u,symbols['frequency_copy_board_probe'],anchor,int(cancel),budget=6000000)
  assert result==(0 if cancel else hz),result
  counter_writes=[(chip,value,bias) for chip,reg,value,bias in transfers if reg==0x32]
  assert counter_writes==[(0,0x8245,0),(0,0x8244,0)]*(1 if cancel else 2),counter_writes
  assert not power and not any(addr==ep+20 and pins&0x4080 or addr==port+20 and pins&2
                               for addr,pins,_ in route),route
  if cancel:
   assert transfers.index((0,0x32,0x8244,0))<len(transfers)-1
   assert transfers[-1]==(0,0x30,0,0)
 register_response=None
 transfers.clear();power.clear();route.clear()
 print('ARM board frequency counter: VHF/UHF RX routes, chip 0, PA idle and stop before receiver release PASS')
 expected_pa_route=(0x80,0)
 result=call_tx(u,symbols['voice_board_tx_probe'],446081250,0,budget=16000000)
 assert result==0,result
 assert (1,0x30,0xc1fe,0) in transfers
 assert (1,0x36,0x57,0) in transfers
 assert all(not (chip==0 and reg==0x30 and val==0xc1fe) for chip,reg,val,_ in transfers)
 assert [v for v in power if v]==[34,68,102,136] and power[-1]==0
 assert not any(bias for _,_,bias in route)
 assert (1,0x30,0,0) in transfers[transfers.index((1,0x36,0x57,0))+1:]
 assert word_read(ep+12)&0x7080==0x7080 and word_read(port+12)&3==3
 print('ARM board UHF PTT: PE15, zero-bias readback, four-step calibrated DAC and safe release PASS')
 for bad_readback in (0x30,0x38,0x39,0x36):
  transfers.clear();power.clear();route.clear()
  words=[iter(responses[0]),iter(responses[1])]
  assert call_tx(u,symbols['voice_board_tx_probe'],446081250,0,budget=16000000)==2
  assert power and not any(power) and not any(bias for _,_,bias in route)
  assert (1,0x30,0,0) in transfers[transfers.index((1,0x36,0x57,0))+1:]
 bad_readback=None
 print('ARM board TX: each mismatched readback blocks PA and unkeys chip PASS')
 for selection,gain in enumerate((36,45,55),1):
  transfers.clear();power.clear();route.clear()
  words=[iter(responses[0]),iter(responses[1])]
  assert call_tx(u,symbols['voice_board_tx_probe'],446081250,selection<<24,budget=16000000)==0
  assert (1,0x7d,(0xe912&0xffe0)|gain,0) in transfers,(selection,transfers)
  assert 136 in power and power[-1]==0
 print('ARM board microphone: Low/Middle/High select OEM register words without changing PA bias PASS')
 for mode in (1,2,3,4):
  transfers.clear();power.clear();route.clear()
  words=[iter(responses[0]),iter(responses[1])]
  result=call_tx(u,symbols['voice_board_tx_probe'],446081250,mode<<20,budget=16000000)
  assert result==(0 if mode==1 else 2),(mode,result)
  assert power and power[-1]==0
  assert [v for v in power if v]==([10,21,31,42] if mode==1 else []),(mode,power)
 print('ARM measured PA code: exact DAC override; above-Low, wrong frequency and zero blocked before keying PASS')
 forbid_active_reads=True
 for tail_tone in (0,0x109,0x201):
  transfers.clear();power.clear();route.clear()
  words=[iter(responses[0]),iter(responses[1])]
  result=call_tx(u,symbols['voice_board_tx_probe'],446081250,tail_tone|0x20000,budget=16000000)
  assert result==0,(tail_tone,result)
  if tail_tone==0x201:
   assert (1,0x52,0x8fff,136) in transfers
  else:
   assert (1,0x51,0x902f,136) in transfers and (1,0x07,0x0471,136) in transfers
   assert (1,0x52,0x0292 if tail_tone else 0x0fff,136) in transfers
  assert power[-1]==0
 forbid_active_reads=False
 print('ARM UHF tail: cached register 52, no serial reads while PA active, 400 ms and safe release PASS')
 transfers.clear();power.clear();route.clear()
 words=[iter((0xa800,0x0280,55,0xa800,0xe912,0x6fff,0x33,0xa800)),iter(())]
 expected_pa_route=(0,2)
 result=call_tx(u,symbols['voice_board_tx_probe'],145500000,0,budget=16000000)
 assert result==0,result
 assert (0,0x30,0xc1fe,0) in transfers
 assert (0,0x36,0x57,0) in transfers
 assert 98 in power and power[-1]==0
 assert all(chip==0 for chip,_,_,_ in transfers)
 assert not any(bias for _,_,bias in route)
 assert (0,0x30,0,0) in transfers[transfers.index((0,0x36,0x57,0))+1:]
 print('ARM board VHF PTT: PE8, calibrated DAC, VHF route and release PASS')
 transfers.clear();power.clear();route.clear()
 words=[iter((0xa800,0x0280,55,0xa800,0xe912,0x6fff,0x33,0xa800)),iter(())]
 assert call_tx(u,symbols['voice_board_tx_probe'],145500000,0x10000,budget=16000000)==0
 assert (0,0x7d,0xe934,0) in transfers
 print('ARM board APRS TX: stock packet gain in register 7D and calibrated VHF key PASS')
 transfers.clear();power.clear();route.clear()
 words=[iter((0xa800,)),iter((0xa800,0x0280,55,0xa800,0xe912,0x6fff,0x33,0xa800,0xa800,0x0280,170))]
 expected_pa_route=(0x4000,0)
 result=call_tx(u,symbols['voice_board_tx_probe'],27185000,0,budget=16000000)
 assert result==0,(result,transfers[-16:],power)
 assert (1,0x30,0xc1fe,0) in transfers and (1,0x36,0x57,0) in transfers
 assert not any(bias for _,_,bias in route)
 assert 73 in power and power[-1]==0
 assert (1,0x30,0,0) in transfers[transfers.index((1,0x36,0x57,0))+1:]
 print('ARM board 27 MHz FM PTT: CB PA calibration, route 4, DAC zero on release and RX restore PASS')
 for hz,chip,bias in ((145500000,0,98),(27185000,1,73)):
  expected_pa_route=(0x4000,0) if chip else (0,2)
  for selection,gain in enumerate((36,45,55),1):
   transfers.clear();power.clear();route.clear()
   words=([iter((0xa800,0x0280,55,0xa800,0xe912,0x6fff,0x33,0xa800)),iter(())] if chip==0 else
          [iter((0xa800,)),iter((0xa800,0x0280,55,0xa800,0xe912,0x6fff,0x33,0xa800,0xa800,0x0280,170))])
   assert call_tx(u,symbols['voice_board_tx_probe'],hz,selection<<24,budget=16000000)==0
   assert (chip,0x7d,(0xe912&0xffe0)|gain,0) in transfers
   assert bias in power and power[-1]==0
 transfers.clear();power.clear();route.clear()
 words=[iter((0xa800,0x0280,55,0xa800,0xe912,0x6fff,0x33,0xa800)),iter(())]
 expected_pa_route=(0,2)
 assert call_tx(u,symbols['voice_board_tx_probe'],145500000,0x1010000,budget=16000000)==0
 assert (0,0x7d,0xe934,0) in transfers
 print('ARM microphone isolation: VHF/CB gain, acquired-owner lock and fixed APRS gain PASS')
 transfers.clear();power.clear();route.clear()
 words=[iter(responses[0]),iter(responses[1])]
 expected_pa_route=(0x80,0)
 assert call_tx(u,symbols['voice_board_tx_probe'],446081250,0x109,budget=16000000)==0
 assert (1,0x51,0x902f,0) in transfers
 assert (1,0x07,1827,0) in transfers
 assert (1,0x52,0x0292,0) in transfers
 assert power[-1]==0
 print('ARM board CTCSS TX: 88.5 Hz sequence before applying PA bias and safe release PASS')
 transfers.clear();power.clear();route.clear()
 words=[iter(responses[0]),iter(responses[1])]
 assert call_tx(u,symbols['voice_board_tx_probe'],446081250,0x201,budget=16000000)==0
 assert (1,0x51,0xa04b,0) in transfers
 assert (1,0x07,0x0ad7,0) in transfers
 dcs_words=[(reg,value) for chip,reg,value,bias in transfers if chip==1 and reg==8 and not bias]
 assert len(dcs_words)==2 and dcs_words[0][1]&0x1ff==0o23 and dcs_words[1][1]&0x8000
 assert power[-1]==0
 transfers.clear();power.clear();route.clear()
 words=[iter(responses[0]),iter(responses[1])]
 assert call_tx(u,symbols['voice_board_tx_probe'],446081250,0x26a,budget=16000000)==0
 assert (1,0x51,0x804b,0) in transfers
 assert [(reg,value) for chip,reg,value,bias in transfers if chip==1 and reg==8 and not bias]==dcs_words
 assert power[-1]==0
 print('ARM board DCS TX: 023N and 023I before PA bias and safe release PASS')
 # RX tone gating is linked in the standard TX-capable firmware only.
 main_symbols=run([a.prefix+'nm',o/'rt950-custom-ui.elf'])
 if 'read_board_vhf_calibration' in main_symbols:
  for tone,match,wrong in ((0x109,0x0400,0x0800),(0x201,0x4000,0x8000),(0x26a,0x8000,0x4000)):
   transfers.clear()
   words=[iter((0xa800,)+(wrong,170)*3+(match,170)*3),iter(())]
   assert call_tx(u,symbols['voice_board_rx_tone_probe'],tone,budget=6000000)==0,tone
   assert any(chip==0 and reg==0x51 and value for chip,reg,value,_ in transfers)
  print('ARM board CTCSS/DCS RX: wrong tone stays muted, three matches open audio, retune clears carrier PASS')
  transfers.clear()
  words=[iter((0xa800,)+(0x0800,170)*3+(0x0400,170)*3),iter(())]
  assert call_tx(u,symbols['voice_board_rx_tone_probe'],0x10109,budget=6000000)==0
  filters=[value for chip,reg,value,_ in transfers if chip==0 and reg==0x43]
  assert filters==[0x3028,0x4008,0x4008],filters
  assert (0,0x30,0x0200,0) in transfers
  print('ARM board FM RX: Wide, Narrow and CTCSS share the selected slot profile PASS')
  audio_pins=[]
  def capture_audio_pins(machine,access,address,size,value,_):
   if address in (ep+16,ep+20) and value==2:
    audio_pins.append(address==ep+16)
  audio_hook=u.hook_add(UC_HOOK_MEM_WRITE,capture_audio_pins)
  for packet in (False,True):
   transfers.clear();audio_pins.clear()
   words=[iter((0xa800,)+(0x0800,170)*3+(0x0400,170)*3),iter(())]
   assert call_tx(u,symbols['voice_board_rx_tone_probe'],0x109|(int(packet)<<17),budget=6000000)==0
   assert (0,0x47,0x6142,0) in transfers
   assert audio_pins==([] if packet else [False,True]),audio_pins
  audio_pins.clear()
  words=[iter((0xa800,0x0280,120,0x0280,150,0x0280,130,0x0280,120,0x0280,55)),iter(())]
  assert call_tx(u,symbols['voice_board_sql_probe'],budget=6000000)==0
  assert audio_pins==[False,True,False],audio_pins
  print('ARM voice SQL: hysteresis and SQL 0 audio without false carrier PASS')
  transfers.clear();audio_pins.clear()
  words=[iter((0xa800,0x0800,55)),iter(())]
  assert call_tx(u,symbols['voice_board_monitor_probe'],budget=6000000)==0
  # PE1 is active-low: clear opens the speaker, set mutes it.
  assert audio_pins==[False,True,False,True],audio_pins
  assert (0,0x47,0x6142,0) in transfers and (0,0x47,0x6042,0) in transfers
  print('ARM monitor audio: valid sample gate, SQL/tone bypass, PTT mute, resume and close/retune PASS')
  u.hook_del(audio_hook)
  print('ARM APRS audio route: AF active with speaker muted and normal voice audio restored PASS')
  for low_hz in (18000000,25000000,27185000,29500000):
   transfers.clear();route.clear()
   words=[iter((0xa800,0x0280,170)),iter((0xa800,0x0280,170))]
   result=call_tx(u,symbols['voice_board_cb_rx_probe'],low_hz,budget=6000000)
   assert result==0,(low_hz,result)
   for chip,hz in ((1,low_hz),(0,145500000)):
    assert (chip,0x38,(hz//10)&0xffff,0) in transfers
    assert (chip,0x39,(hz//10)>>16,0) in transfers
    assert (chip,0x30,0xbff1,0) in transfers
   assert (ep+20,0x1000,0) in route and (port+20,1,0) in route
  print('ARM board low FM RX: 18/25/27.185/29.5 MHz, TX gating and return to VHF PASS')




 # 0.29 menu 0x08019BA0 indexes strings copied by the scatter loader.
 menu=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
 menu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
 menu.mem_map(0x08000000,0x100000)
 menu.mem_map(0x20000000,0x18000)
 menu.mem_write(0x08003000,stock)
 menu.reg_write(UC_ARM_REG_SP,0x20015f78)
 menu.emu_start(0x08003185,0x08003280,count=3000000)
 assert menu.reg_read(UC_ARM_REG_PC)==0x08003280
 mic_pointers=struct.unpack('<3I',menu.mem_read(0x20000768,12))
 for pointer,label in zip(mic_pointers,(b'Low\0',b'Middle\0',b'High\0')):
  assert menu.mem_read(pointer,len(label))==label
 save_hook=menu.hook_add(UC_HOOK_CODE,return_from_hook,begin=0x0801ac38,end=0x0801ac38)
 for selection in range(3):
  menu.mem_write(0x2000e89c+3,bytes([selection]))
  call_tx(menu,0x08020edc)
  assert menu.mem_read(0x2000a91c+18,1)==bytes([selection])
 menu.hook_del(save_hook)
 print('OEM 0.29 Mic Gain: Low=0/Middle=1/High=2 menu strings and applied global field PASS')
 pointers=struct.unpack('<3I',menu.mem_read(0x200004e4,12))
 for level,label in enumerate((b'Low',b'Middle',b'High')):
  index=call_tx(u,symbols['voice_factory_selector_probe'],level)
  assert index<3 and menu.mem_read(pointers[index],len(label)+1)==label+bytes(1)
 assert call_tx(u,symbols['voice_factory_selector_probe'],3)==2
 assert call_tx(u,symbols['voice_factory_selector_probe'],4)==2
 assert call_tx(u,symbols['voice_factory_selector_probe'],5)==255
 print('OEM 0.29 Trans Power menu: High=0, Middle=1, Low=2 PASS')
 bandwidth=struct.unpack('<2I',menu.mem_read(0x20000370,8))
 for pointer,label in zip(bandwidth,(b'Wide\0',b'Narrow\0')):
  assert menu.mem_read(pointer,len(label))==label
 print('OEM 0.29 FM bandwidth menu: Wide=0, Narrow=1 PASS')
 vhf_pointers=struct.unpack('<3I',menu.mem_read(0x200000d4+116,12))
 assert vhf_pointers==(0x20000024,0x20000064,0x200000a4)
 for level,pointer in enumerate(vhf_pointers):
  menu.mem_write(0x20000c7c+23,bytes([level]))
  menu.mem_write(pointer,bytes(range(33-16*level,49-16*level)))
  for hz in range(136000000,174000001,125000):
   menu.mem_write(0x20000c7c+12,struct.pack('<I',hz//10))
   assert call_tx(menu,0x0801ea60)==call_tx(u,symbols['voice_vhf_pa_probe'],hz,level),(hz,level)
 for hz,level in ((135999990,0),(174000010,0),(145500000,3),(145500001,0)):
  assert call_tx(u,symbols['voice_vhf_pa_probe'],hz,level)==0
 print('OEM 0.29 VHF PA: factory table pointers and 915 frequency/power lookups PASS')


 for tone in (670,885,1000,1230,2541):
  for level in (0,47,112,127):
   for extra in (0,1):
    oracle.mem_write(0x2000011f,bytes([level]))
    oracle.mem_write(0x20000c7c+26,bytes([extra]))
    startup.clear()
    call_tx(oracle,0x0800a584,tone,0)
    address=0x20012000
    count=call_tx(u,symbols['voice_ctcss_tx_probe'],tone,level|(extra<<8),address)
    packed=struct.unpack('<'+'I'*count,u.mem_read(address,count*4))
    assert [(entry>>16,entry&0xffff) for entry in packed]==startup,(tone,level,extra,startup,packed)
 print('OEM 0.29 CTCSS TX: 40 register sequences, factory gain and optional adjustment PASS')

 cb_pointers=struct.unpack('<3I',menu.mem_read(0x200000d4+92,12))
 assert cb_pointers==(0x2000016c,0x2000017c,0x2000018c)
 for level,pointer in enumerate(cb_pointers):
  menu.mem_write(0x20000c7c+23,bytes([level]))
  menu.mem_write(pointer,bytes(range(33-16*level,49-16*level)))
  for hz in range(26000000,28000000,25000):
   menu.mem_write(0x20000c7c+12,struct.pack('<I',hz//10))
   assert call_tx(menu,0x0801ea60)==call_tx(u,symbols['voice_cb_pa_probe'],hz,level),(hz,level)
 print('OEM 0.29 CB PA: factory table pointers and 240 frequency/power lookups PASS')

 oracle.mem_write(0x20000c7c+26,b'\x00')
 for level in (15,75):
  oracle.mem_write(0x20000120,bytes([level]))
  for code in range(1,512):
   startup.clear()
   oracle.reg_write(UC_ARM_REG_R3,0)
   call_tx(oracle,0x0800d78c,code,0,0)
   address=0x20012000
   count=call_tx(u,symbols['voice_dcs_tx_probe'],code,level,address)
   packed=struct.unpack('<'+'I'*count,u.mem_read(address,count*4))
   assert [(entry>>16,entry&0xffff) for entry in packed]==startup,(code,level,startup,packed)
 print('OEM 0.29 DCS TX normal: 1022 register sequences and Golay codewords PASS')

 # The stock menu maps indices 106..210 to type 3 and the same DCS table.
 for index in range(1,211):
  mode=call_tx(oracle,0x08015b04,index)
  assert mode==(3 if index>105 else 2)
  code=struct.unpack_from('<H',stock,0x2994e+2*((index-1)%105))[0]
  oracle.mem_write(0x20000c7c+16,struct.pack('<I',code))
  oracle.mem_write(0x20000c7c+20,bytes([mode]))
  oracle.mem_write(0x20000c7c+25,bytes(2))
  oracle.mem_write(0x20000120,bytes([75]))
  startup.clear()
  call_tx(oracle,0x0800a850)
  address=0x20012000
  count=call_tx(u,symbols['voice_dcs_tx_probe'],code,75|((mode==3)<<8),address)
  packed=struct.unpack('<'+'I'*count,u.mem_read(address,count*4))
  assert [(entry>>16,entry&0xffff) for entry in packed]==startup[-4:],(index,startup,packed)
 print('OEM 0.29 DCS TX: 210 normal/inverse menu positions matched to TX register writes PASS')
 # 0x20000C7C+21 and the serial chip selector are distinct inputs.
 oracle.mem_write(0x200000d4+85,bytes(3))
 for chip in (0,1):
  oracle.mem_write(0x20000c70,bytes([chip]))
  for profile in (0,1):
   oracle.mem_write(0x20000c7c+21,bytes([profile]))
   startup.clear()
   call_tx(oracle,0x0801e510,2700000)
   assert startup==[(0x38,2700000&0xffff),(0x39,2700000>>16),
                    (0x43,0x4008 if profile else 0x3028)],(chip,profile,startup)
 print('OEM 0.29 27 MHz tuning: frequency halves and independent register 43 profile selector PASS')


 # 0.29 DDS consumes packed tone levels, one bit per 32 DAC samples.
 for levels in (0,255,0x55,0xa6):
  oracle.mem_write(0x20000230+12,bytes(4))
  oracle.mem_write(0x2000e12a,bytes([levels]))
  oracle.mem_write(0x2000e12a+514,struct.pack('<2H',0,8))
  address=0x20012000
  call_tx(oracle,0x08006380,address,256)
  call_tx(u,symbols['aprs_afsk_probe'],levels,address,8)
  assert oracle.mem_read(address,512)==u.mem_read(address,512),levels
 print('OEM 0.29 APRS DDS: 32 samples per symbol and 1024 DAC samples matched PASS')

 # Compare release signalling while the stock extra signalling flag is clear.
 oracle.mem_write(0x20000c74,struct.pack('<II',0x080fe101,0x080fe105))
 for kind in range(3):
  oracle.mem_write(0x20000c7c+20,bytes([kind]))
  oracle.mem_write(0x20000c7c+25,bytes(1))
  for level in range(256):
   oracle.mem_write(0x2000011f,bytes([level]))
   startup.clear()
   call_tx(oracle,0x0801d7b4,1)
   address=0x20012000
   count=call_tx(u,symbols['voice_tail_probe'],kind,level,address)
   if kind<2 and level&127==0:
    assert count==0
    continue
   packed=struct.unpack('<'+'I'*count,u.mem_read(address,4*count))
   assert [(entry>>16,entry&0xffff) for entry in packed]==startup,(kind,level,startup)
 print('OEM 0.29 tail: all 256 factory bytes use 7-bit level; zero level rejected, nonzero sequences matched PASS')

 # Isolate the OEM SysTick counter logic, stubbing its service calls.
 timer=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
 timer.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
 timer.mem_map(0x08000000,0x100000);timer.mem_map(0x20000000,0x20000)
 timer.mem_write(0x08003000,stock)
 timer.hook_add(UC_HOOK_CODE,return_from_hook,begin=0x0800a514,end=0x0800a514)
 for at in range(0x0800c9bc,0x0800cb18,2):
  first,second=struct.unpack('<HH',stock[at-0x08003000:at-0x08003000+4])
  if first&0xf800!=0xf000 or second&0xd000!=0xd000:continue
  sign=(first>>10)&1
  offset=(sign<<24)|((1^((second>>13&1)^sign))<<23)|((1^((second>>11&1)^sign))<<22)|((first&1023)<<12)|((second&2047)<<1)
  if sign:offset-=1<<25
  target=at+4+offset
  timer.hook_add(UC_HOOK_CODE,return_from_hook,begin=target,end=target)
 timer.mem_write(0x2000a403,struct.pack('<H',40))
 for tick in range(1,401):
  call_tx(timer,0x0800c9bc)
  remaining=struct.unpack('<H',timer.mem_read(0x2000a403,2))[0]
  assert remaining==40-tick//10,(tick,remaining)
 print('OEM 0.29 release counter: 40 decrements at 10 SysTick intervals = 400 ms PASS')


 for page in (0x40001000,0x40020000):
  u.mem_map(page,0x1000)
 def update_pb8(machine,access,address,size,value,_):
  if value&0x100:
   prior=word_read(port+12)
   word_write(port+12,prior|0x100 if address==port+16 else prior&~0x100)
 u.hook_add(UC_HOOK_MEM_WRITE,update_pb8,begin=port+16,end=port+20)
 for mode in range(5):
  result=call_tx(u,symbols['aprs_dac_probe'],mode,budget=1000000)
  assert result==0,(mode,result)
 print('ARM APRS DAC1: timer and DMA setup, sample buffer, completion, timeout, cancellation, busy ownership, overrun and PB8 restore PASS')

 for page in (0x40000000,0x40012000):
  u.mem_map(page,0x1000)
 def clear_dma1_flags(machine,access,address,size,value,_):
  word_write(0x40020000,word_read(0x40020000)&~value)
 u.hook_add(UC_HOOK_MEM_WRITE,clear_dma1_flags,begin=0x40020004,end=0x40020004)
 for mode in (0,1,2):
  result=call_tx(u,symbols['aprs_adc_probe'],mode,budget=5000000)
  assert result==0,(mode,result)
 print('ARM APRS ADC1: TIM2/CH2, DMA1/CH1 to AX.25, overrun and calibration timeout PASS')
 result=call_tx(u,symbols['mcu_diagnostics_probe'],budget=1000000)
 assert result==0,f'MCU diagnostics failed at C++ line {result}'
 print('ARM MCU diagnostics: datasheet temperature trend, ADC1 busy ownership, internal channels, restoration, cancellation and timeout PASS')

 u.mem_map(0x40013000,0x1000)
 result=call_tx(u,symbols['aprs_ble_probe'],budget=1000000)
 assert result==0,result
 print('ARM BLE KISS: USART1 setup, escaped bytes, queue bound, timeout and revocation PASS')
 print('ARM BLE GPS: five states, coordinate validity, private command and AX.25 priority PASS')

