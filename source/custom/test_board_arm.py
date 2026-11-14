"""Execute real ARM code with simulated GPIO/UART, not hardware validation.

Requires unicorn. Usage: python test_board_arm.py --prefix /path/arm-none-eabi-
  --stock /path/stock029.bin
The optional stock image is only an oracle for keypad scanning, never linked.
"""
from pathlib import Path
import argparse
import struct
import subprocess
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_PC, UC_ARM_REG_C1_C0_2, UC_ARM_REG_FPEXC, UC_ARM_REG_CPSR, UC_ARM_REG_PRIMASK, UC_CPU_ARM_CORTEX_M4

parser=argparse.ArgumentParser()
parser.add_argument('--prefix',default='arm-none-eabi-')
parser.add_argument('--stock',type=Path)
parser.add_argument('--bench',action='store_true')
parser.add_argument('--raw',type=Path,help='Raw image paired with the selected build ELF')
args=parser.parse_args()
root=Path(__file__).resolve().parent
build=root/('build-bench' if args.bench else 'build')
raw=(args.raw or build/'rt950-custom-ui.bin').read_bytes()
symbols={}
for line in subprocess.check_output([args.prefix+'nm','-n',str(build/'rt950-custom-ui.elf')],text=True).splitlines():
    parts=line.split()
    if len(parts)==3: symbols[parts[2]]=int(parts[0],16)
BASE=0x08003000
GPIO=[0x40010800,0x40010c00,0x40011000,0x40011400,0x40011800]
STOP=0x080ff000
KEYS=[10,14,18,13,6,3,9,2,12,16,20,21,11,15,19,17,7,8,4,5]

class Machine:
    def __init__(self,image,pressed=(),side=None):
        self.u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
        self.u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M4)
        for start,length in [(0x08000000,0x100000),(0x20000000,0x18000),(0x1ffff000,0x1000),
                             (0x40000000,0x10000),(0x40010000,0x3000),
                             (0x40020000,0x4000),(0xe000e000,0x2000)]:
            self.u.mem_map(start,length)
        self.u.reg_write(UC_ARM_REG_C1_C0_2,0xF00000)
        self.u.reg_write(UC_ARM_REG_FPEXC,1<<30)
        self.u.mem_write(BASE,image)
        if image==raw and symbols['_edata']>symbols['_sdata']:
            start=symbols['_sidata']-BASE;length=symbols['_edata']-symbols['_sdata']
            self.u.mem_write(symbols['_sdata'],image[start:start+length])
        self.odr={p:0xffff for p in GPIO}
        self.pressed=pressed;self.side=side;self.feeds=0;self.cut=False
        self.route_clear=[]
        self.u.hook_add(UC_HOOK_MEM_READ,self.read)
        self.u.hook_add(UC_HOOK_MEM_WRITE,self.write)
        self.u.hook_add(UC_HOOK_CODE,self.code)
    def code(self,u,address,size,data):
        if address==STOP:u.emu_stop()
    def read(self,u,access,address,size,value,data):
        if address==0x40004804: # USART3 DR read clears RXNE/error flags
            u.mem_write(0x40004800,bytes(4))
        if address not in [p+8 for p in GPIO]:return
        port=address-8;v=0xffff
        if port==GPIO[3]:
            for col,row in self.pressed:
                if col==0 or not(self.odr[GPIO[2]] & (1<<(col-1))):v &= ~(1<<(7-row))
        if port==GPIO[4]:
            v &= ~1 # power switch ON
            if self.side==0:v &= ~(1<<5)
            v &= ~getattr(self,"ptt_pins",0)
        if port==GPIO[0] and self.side==1:v &= ~(1<<12)
        u.mem_write(address,struct.pack('<I',v))
    def write(self,u,access,address,size,value,data):
        if address==0x40003000:
            self.feeds+=1
            if self.cut:u.emu_stop()
        for p in GPIO:
            if address==p+16:self.odr[p] |= value
            elif address==p+20:
                self.odr[p] &= ~value
                if p==GPIO[4] and value & 0x7080:self.route_clear.append(value)
                if p==GPIO[1] and value & 3:self.route_clear.append(value)
            elif address==p+12:self.odr[p]=value
            if address in (p+12,p+16,p+20):
                u.mem_write(p+12,struct.pack('<I',self.odr[p]&0xffff))
    def call(self,address,budget=250000):
        self.u.reg_write(UC_ARM_REG_CPSR,self.u.reg_read(UC_ARM_REG_CPSR)&~0x0600fc00)
        self.u.reg_write(UC_ARM_REG_SP,0x20017b00)
        self.u.reg_write(UC_ARM_REG_LR,STOP|1)
        self.u.emu_start(address|1,STOP,count=budget)
        return self.u.reg_read(UC_ARM_REG_R0)
    def function(self,name):return self.call(symbols[name])

startup=Machine(raw)
startup.u.mem_write(0x40021024,struct.pack('<I',(1<<29)|(1<<28)|3))
def stop_at_system_init(u,address,size,data):
    u.emu_stop()
startup.u.hook_add(UC_HOOK_CODE,stop_at_system_init,begin=symbols['SystemInit'],end=symbols['SystemInit'])
startup.function('Reset_Handler')
assert startup.u.reg_read(UC_ARM_REG_PC)==symbols['SystemInit']
assert bytes(startup.u.mem_read(symbols['custom_reset_flags'],1))==b'\x0c'
assert struct.unpack('<I',startup.u.mem_read(0x40021024,4))[0]==((1<<24)|1)
print('ARM reset telemetry: real Reset_Handler captures combined CRM flags before SystemInit PASS')

m=Machine(raw)
for size in (256,512,1024,65535):
    m.u.mem_write(0x1ffff7e0,struct.pack('<H',size))
    assert m.function('board_flash_kib')==size
assert m.function('board_storage_probe')==255
m.u.mem_write(0x1ffff7e0,struct.pack('<H',1024))
assert m.function('board_storage_probe')==0
m.u.mem_write(symbols['_storage_start']+2048,bytes(4))
assert m.function('board_storage_probe')==1
m.u.mem_write(symbols['_storage_start']+symbols['_storage_slot_size'],bytes(4))
assert m.function('board_storage_probe')==3
print('ARM flash geometry and read-only slot probe PASS')

cases=[()]+[((c,r),) for c in range(5) for r in range(4)]
cases += [((a//4,a%4),(b//4,b%4)) for a in range(20) for b in range(a+1,20)]
stock=args.stock.read_bytes() if args.stock else None
if stock:
    import hashlib
    assert hashlib.sha256(stock).hexdigest()=='025bc583802a96911836e274cdfdf3ce05d9919bc77451b7f7f1573fb5571907'
for keys in cases:
    m=Machine(raw,keys);actual=m.function('board_scan_key')
    assert m.odr[GPIO[2]]&15==15,'columns must return to idle'
    if len(keys)==0:assert actual==255
    elif len(keys)==1:assert actual==KEYS[keys[0][0]*4+keys[0][1]],(keys,actual)
    if stock:
        expected=Machine(stock,keys).call(0x08016160)
        assert actual==expected,(keys,actual,expected)
for side in (0,1):assert Machine(raw,((2,1),),side).function('board_scan_key')==side
for mask,pins in enumerate((0,1<<3,1<<2,(1<<2)|(1<<3))):
    m=Machine(raw)
    m.ptt_pins=pins
    assert m.function('board_ptt_mask')==mask
    assert m.function('board_scan_key')==255
assert Machine(raw,side=0).function('board_ptt_mask')==4
assert Machine(raw,side=1).function('board_ptt_mask')==0
print('ARM side inputs: independent PTT1/PTT2 levels, distinct from keypad scanner PASS')
print(f'ARM scanner: {len(cases)} key combinations and side priority PASS; OEM oracle={bool(stock)}')

# Electrical regression: unlike the original oracle, check actual GPIO mode.
# The first all-HIGH pass must release all columns WITH pull-ups (CNF=2,
# MODE=0). OD output HIGH alone cannot establish HIGH without a pull-up.
class Electrical(Machine):
    def read(self,u,access,address,size,value,data):
        if address==GPIO[3]+8:
            cfg=struct.unpack('<I',u.mem_read(GPIO[2],4))[0]
            for col in range(4):
                if self.odr[GPIO[2]] & (1<<col):
                    assert (cfg>>(col*4))&15==2, 'column must match OEM PP 2MHz'
        super().read(u,access,address,size,value,data)
for keys in cases:
    m=Electrical(raw,keys); m.function('board_init'); m.function('board_scan_key')
print('ARM GPIO configuration: columns match OEM push-pull PASS (electrical hypothesis only)')

m=Machine(raw);m.cut=True
shutdown_delays=[]
def shutdown_delay(u,address,size,data):
    assert not u.reg_read(UC_ARM_REG_PRIMASK),'SysTick delay must run with interrupts enabled'
    shutdown_delays.append(u.reg_read(UC_ARM_REG_R0));u.reg_write(UC_ARM_REG_PC,u.reg_read(UC_ARM_REG_LR))
m.u.hook_add(UC_HOOK_CODE,shutdown_delay,begin=symbols['delay_ms'],end=symbols['delay_ms'])
m.function('board_power_cut')
assert shutdown_delays==[100] and m.u.reg_read(UC_ARM_REG_PRIMASK)==1
assert not m.route_clear,'shutdown must never activate TX routing'
assert m.odr[GPIO[4]]&0x7080==0x7080
assert m.odr[GPIO[1]]&3==3
assert not m.odr[GPIO[0]]&(1<<11)
assert not m.odr[GPIO[1]]&(1<<9)
assert not m.odr[GPIO[2]]&(1<<6)
assert not m.odr[GPIO[1]]&(1<<3)
assert struct.unpack('<I',m.u.mem_read(0x4000480c,4))[0]==0
print('ARM shutdown: PA11/PB9, backlight, GPS UART and safe routing PASS')

m=Machine(raw)
m.u.reg_write(UC_ARM_REG_R0,1);m.function('board_gps_enable')
sentence=b'$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n'
for ch in sentence:
    m.u.mem_write(0x40004800,struct.pack('<II',32,ch))
    m.function('USART3_IRQHandler')
m.u.reg_write(UC_ARM_REG_R0,1000);m.function('board_gps_poll')
nav=m.function('nav_get')
assert bytes(m.u.mem_read(nav+16,2))==b'\x01\x08'
assert m.function('board_gps_bytes')==len(sentence)
assert m.function('board_gps_errors')==0
assert m.function('board_gps_dropped')==0
m.u.reg_write(UC_ARM_REG_R0,0);m.function('board_gps_enable')
assert bytes(m.u.mem_read(nav+16,2))==b'\0\0','disabled GPS must not retain a live fix'
print('ARM GPS: IRQ ring, NMEA fix, byte counters, disable/reset PASS')

m=Machine(raw)
m.u.mem_write(symbols['custom_watchdog_armed'],b'\x01')
m.u.mem_write(symbols['custom_heartbeat'],struct.pack('<I',0))
for _ in range(600):m.function('SysTick_Handler')
assert m.feeds==499,m.feeds
print('ARM watchdog: foreground stall stops tick feeding at 500ms PASS')

class EncoderMachine(Machine):
    state=3
    def read(self,u,access,address,size,value,data):
        super().read(u,access,address,size,value,data)
        if address==GPIO[1]+8:u.mem_write(address,struct.pack('<I',0xffcf|(self.state<<4)))
m=EncoderMachine(raw);m.function('encoder_init')
def sample(state):
    m.state=state
    result=[]
    for _ in range(2):m.function('encoder_tick');result.append(m.function('encoder_poll'))
    return result
cw=[]
for state in (2,0,1,3):cw+=sample(state)
assert cw.count(1)==2 and cw.count(255)==0,cw
ccw=[]
for state in (1,0,2,3):ccw+=sample(state)
assert ccw.count(0xffffffff)==2,ccw
assert m.function('encoder_edges')==8
assert m.function('encoder_invalid_edges')==0
sample(0)
assert m.function('encoder_invalid_edges')==1
m.function('encoder_init')
for _ in range(10):
    for state in (1,3,2,0):
        m.state=state
        for _ in range(2):m.function('encoder_tick')
assert m.function('encoder_overruns')==12
for _ in range(8):assert m.function('encoder_poll')==1
assert m.function('encoder_poll')==0
print('ARM encoder: CW/CCW alternating 00/11 detents, pull-up sampling, invalid transition counter PASS')

# ADC calibration/EOC are advanced explicitly by this peripheral model.
m=Machine(raw)
def poll_battery(now):
    m.u.reg_write(UC_ARM_REG_R0,now);m.function('board_battery_poll')
def adc(off,value):m.u.mem_write(0x40012800+off,struct.pack('<I',value))
def reg(off):return struct.unpack('<I',m.u.mem_read(0x40012800+off,4))[0]
m.u.reg_write(UC_ARM_REG_R0,0);m.function('board_battery_init')
assert reg(0x34)==1 and reg(0x10)&56==56
assert (struct.unpack('<I',m.u.mem_read(GPIO[0],4))[0]>>4)&15==0
poll_battery(1);assert reg(8)&8
adc(8,reg(8)&~8);poll_battery(2);assert reg(8)&4
adc(8,reg(8)&~4);poll_battery(3)
adc(0,2);adc(0x4c,0xa00);poll_battery(4)
adc(0,2);adc(0x4c,0xa08);poll_battery(5)
assert m.function('board_battery_raw')==160
assert m.function('board_battery_raw12')==0xa08
assert m.function('board_battery_mv')==7736
for now,age in ((5,0),(2004,1999),(3,0xfffffffe)):
    m.u.reg_write(UC_ARM_REG_R0,now)
    assert m.function('board_battery_sample_age')==age
for now,fresh in ((5,1),(2004,1),(2005,0)):
    m.u.reg_write(UC_ARM_REG_R0,now)
    assert m.function('board_battery_sample_fresh')==fresh
assert m.function('board_battery_errors')==0
if stock:
    oracle=Machine(stock);oracle.u.mem_write(0x40012800,struct.pack('<I',2));oracle.u.mem_write(0x4001284c,struct.pack('<I',0xa00))
    assert oracle.call(0x08016988)==160
    assert struct.unpack('<I',oracle.u.mem_read(0x40012834,4))[0]&31==1
poll_battery(1005);adc(0,0);poll_battery(1025)
assert m.function('board_battery_mv')==0 and m.function('board_battery_errors')==1
assert m.function('board_battery_sample_age')==0xffffffff
assert m.function('board_battery_sample_fresh')==0
poll_battery(2025);poll_battery(2026);poll_battery(2046)
assert m.function('board_battery_errors')==2
print('ARM battery: ADC2 PA1 sequence, dummy sample, OEM raw oracle, 12-bit 7.736V scaling, 2s freshness, EOC/calibration timeouts PASS')

m=Machine(raw)
m.u.reg_write(UC_ARM_REG_R0,0);m.function('board_battery_init')
poll_battery(1);adc(8,reg(8)&~8);poll_battery(2);adc(8,reg(8)&~4);poll_battery(3)
adc(0,2);poll_battery(4);adc(0,2);adc(0x4c,0xa00);poll_battery(5)
m.u.reg_write(UC_ARM_REG_R0,1);m.function('board_battery_fast')
adc(0,2);poll_battery(24);assert reg(0)&2
poll_battery(25);assert not reg(0)&2
adc(0,2);poll_battery(26);adc(0,2);adc(0x4c,188<<4);poll_battery(27)
assert m.function('board_battery_raw')==188
m.u.reg_write(UC_ARM_REG_R0,0);m.function('board_battery_fast')
adc(0,2);poll_battery(1026);assert reg(0)&2
poll_battery(1027);assert not reg(0)&2
print('ARM battery schedule: 20ms TX conversions, high sample and return to 1s standby PASS')


m=Machine(raw)
for mv,want in [(0,255),(6000,0),(6400,0),(7400,52),(7420,53),(7460,55),(8319,99),(8320,100),(8400,100),(9000,100)]:
    m.u.reg_write(UC_ARM_REG_R0,mv);assert m.function('board_battery_percent_from_mv')==want
previous=0
for mv in range(6400,8320,13):
    m.u.reg_write(UC_ARM_REG_R0,mv);value=m.function('board_battery_percent_from_mv')
    assert previous<=value<=100 and value==min(99,((mv-6400)*100+960)//1920);previous=value
print('Battery display estimate: 6.4..8.32V clamp, unknown sentinel, monotonic 1-percent steps PASS')

m=Machine(raw);m.function('board_init');m.function('board_display_init')
for level in range(1,5):
    m.u.reg_write(UC_ARM_REG_R0,level);m.function('board_display_brightness');high=0
    for _ in range(16):m.function('board_display_tick');high+=bool(m.odr[GPIO[2]]&(1<<6));assert bool(m.odr[GPIO[2]]&(1<<6))==bool(m.odr[GPIO[1]]&8)
    assert high==4*level,(level,high)
assert not m.odr[GPIO[1]]&(1<<9) # Bluetooth starts off
m.odr[GPIO[1]]|=1<<9 # Simulate the enabled module before sleeping.
m.u.mem_write(0x4000480c,struct.pack('<I',0x202c));m.function('board_display_sleep')
assert not m.odr[GPIO[2]]&(1<<6) and not m.odr[GPIO[1]]&8
assert m.odr[GPIO[0]]&(1<<11) and m.odr[GPIO[1]]&(1<<9)
assert struct.unpack('<I',m.u.mem_read(0x4000480c,4))[0]==0x202c
m.function('board_display_begin_wake');m.u.reg_write(UC_ARM_REG_R0,4);m.function('board_display_brightness');m.function('board_display_tick')
assert m.odr[GPIO[2]]&(1<<6)
for rx,tx,want in [(1,0,0x4000),(0,1,0x6000),(0,0,0)]:
    m.u.reg_write(UC_ARM_REG_R0,rx);m.u.reg_write(UC_ARM_REG_R1,tx);m.function('board_radio_led');assert m.odr[GPIO[2]]&0x6000==want
print('ARM display: four PWM duties, screen off/wake preserves GPS/power, green/red+green outputs PASS')

class Foreground(Machine):
    """Real main/application/input code; synthetic tick and text LCD sink."""
    def __init__(self):
        self.now=0;self.texts=[]
        super().__init__(raw)
        self.cut=True
        self.u.mem_write(0xe000ed10,struct.pack("<I",6))
        self.sequence=[14,12,15,10,11,15,6,3,6,9] # 435125, OK, ABC, pages
    def code(self,u,address,size,data):
        if bytes(u.mem_read(address,4))==b'\xbf\xf3\x6f\x8f':
            u.reg_write(UC_ARM_REG_PC,(address+4)|1);return
        if bytes(u.mem_read(address,2))==b'\x30\xbf':
            u.reg_write(UC_ARM_REG_PC,(address+2)|1);return
        if address==symbols['delay_ms']:
            self.now+=u.reg_read(UC_ARM_REG_R0)
            u.reg_write(UC_ARM_REG_PC,u.reg_read(UC_ARM_REG_LR));return
        if address==symbols['get_tick']:
            self.now+=1
            u.reg_write(UC_ARM_REG_R0,self.now)
            u.reg_write(UC_ARM_REG_CPSR,u.reg_read(UC_ARM_REG_CPSR)&~0x0600fc00)
            u.reg_write(UC_ARM_REG_PC,u.reg_read(UC_ARM_REG_LR));return
        if address in [symbols['lcd_draw_string'],symbols['lcd_draw_string_2x']]:
            ptr=u.reg_read(UC_ARM_REG_R2)
            buf=bytearray()
            for n in range(80):
                ch=bytes(u.mem_read(ptr+n,1))[0]
                if ch==0:break
                buf.append(ch)
            self.texts.append(buf.decode('utf-8'))
        if address in [symbols[n] for n in ('lcd_init','lcd_draw_string','lcd_draw_string_2x','lcd_fill_rect')]:
            u.reg_write(UC_ARM_REG_CPSR,u.reg_read(UC_ARM_REG_CPSR)&~0x0600fc00)
            u.reg_write(UC_ARM_REG_PC,u.reg_read(UC_ARM_REG_LR));return
        super().code(u,address,size,data)
    def read(self,u,access,address,size,value,data):
        self.pressed=()
        slot=(self.now-1000)//180
        if 0<=slot<len(self.sequence) and (self.now-1000)%180<80:
            i=KEYS.index(self.sequence[slot]);self.pressed=((i//4,i%4),)
        super().read(u,access,address,size,value,data)
        if address==GPIO[4]+8 and self.now>=4000:
            v=struct.unpack('<I',u.mem_read(address,4))[0]|1
            u.mem_write(address,struct.pack('<I',v))

m=Foreground();m.call(symbols['main'],budget=17000000)
assert m.now>=4050 and m.feeds>=1,'foreground must reach debounced power cut'
assert any('435.12500' in line for line in m.texts),'numeric entry not committed/rendered'
assert 'ACTIVE' in m.texts,'slot cards not rendered'
assert 'Radio' in m.texts and 'GPS' in m.texts,'menu did not render'
assert not m.odr[GPIO[0]]&(1<<11)
assert struct.unpack('<I',m.u.mem_read(0xe000ed10,4))[0]&6==0
assert m.odr[GPIO[2]]&0x6000==0,'standby must not assert RX/TX LEDs'
print('ARM main integration: digit input -> OK commit -> A/B/C -> OK menu -> Back -> shutdown PASS (LCD/tick simulated)')

messages=Foreground()
messages.sequence=[6,8,8,8,6,8,6] # Settings -> APRS -> Messages
messages.call(symbols['main'],budget=17000000)
assert 'APRS messages' in messages.texts and '0/3' in messages.texts
assert '1/3' not in messages.texts
print('ARM APRS messages UI: empty inbox displays 0/3 PASS')


if args.bench:
    class NoFactoryPtt(Foreground):
        def __init__(self):
            super().__init__()
            self.sequence=[14,14,16,17,19,10,11,15,6] # 44608125 + OK
            self.rf_reads=0
            self.ptt_reads=0
            self.dac=[]
        def read(self,u,access,address,size,value,data):
            super().read(u,access,address,size,value,data)
            if address!=GPIO[4]+8:return
            if not self.odr[GPIO[4]]&0x100:
                sample=self.rf_reads//16
                reg=0xa800 if sample==0 else 0x0280 if sample%2 else 55
                bit=(reg>>(15-self.rf_reads%16))&1
                self.rf_reads+=1
                u.mem_write(address,struct.pack('<I',bit<<11))
            elif 2900<=self.now<3200:
                flags=struct.unpack('<I',u.mem_read(address,4))[0]
                u.mem_write(address,struct.pack('<I',flags&~8))
                self.ptt_reads+=1
        def write(self,u,access,address,size,value,data):
            super().write(u,access,address,size,value,data)
            if address==0x4000741c:self.dac.append(value)
    guard=NoFactoryPtt()
    guard.call(symbols['main'],budget=50000000)
    assert guard.now>=4050 and guard.feeds>=1
    assert any('446.08125' in line for line in guard.texts)
    assert guard.rf_reads>16 and guard.ptt_reads>10
    assert 'TX NO CALIBRATION' in guard.texts
    assert not guard.dac,'PTT must not arm DAC when calibration SPI fails'
    assert not any(mask&0x4080 or mask==2 for mask in guard.route_clear)
    print('ARM bench main: valid UHF slot and pressed PTT cannot key after factory SPI failure PASS')


# Model Bank 1 control accesses for the manual-save entry point.
class FlashModel(Machine):
    def __init__(self):
        super().__init__(raw)
        self.u.mem_map(0xe0001000,0x1000)
        self.programmed=[];self.erased=[];self.controller_writes=[]
        self.u.mem_write(0x1ffff7e0,struct.pack('<H',1024))
        self.put(0x40022010,128)
    def put(self,address,value):self.u.mem_write(address,struct.pack('<I',value))
    def get(self,address):return struct.unpack('<I',self.u.mem_read(address,4))[0]
    def write(self,u,access,address,size,value,data):
        if 0x40022000<=address<0x40022020:self.controller_writes.append(address)
        if address==0x40022004 and value==0xcdef89ab:
            self.put(0x40022010,0)
        if address==0x40022010 and value==66:
            target=self.get(0x40022014)
            assert target%2048==0 and 0x08010000<=target<0x08014000
            self.erased.append(target)
            self.u.mem_write(target,b'\xff'*2048)
            self.put(0x4002200c,32)
        if 0x08000000<=address<0x08100000:
            assert size==4 and 0x08010000<=address<0x08014000
            assert self.get(0x40022010)==1 and self.get(address)==0xffffffff
            self.programmed.append(address)
            self.put(0x4002200c,32)
        super().write(u,access,address,size,value,data)

fm=FlashModel()
def flash_call(name,*values):
    for register,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),values):fm.u.reg_write(register,value)
    return fm.call(symbols[name],budget=4000000)
operation=next(name for name,address in symbols.items()
               if 'flash_operation' in name and 0x20000000<=address<0x20018000)
class BusyFlashModel(FlashModel):
    def __init__(self):
        super().__init__()
        self.cycles=0
        self.edges=[]
        self.u.hook_add(UC_HOOK_MEM_READ,self.cycle,begin=0xe0001004,end=0xe0001004)
    def cycle(self,u,access,address,size,value,data):
        self.put(address,self.cycles)
        self.cycles+=120000
        if self.cycles==9*120000:self.put(0x4002200c,32)
    def write(self,u,access,address,size,value,data):
        super().write(u,access,address,size,value,data)
        if address==0x40022010 and value==66:self.put(0x4002200c,1)
        if address in (GPIO[2]+16,GPIO[2]+20,GPIO[1]+16,GPIO[1]+20):
            self.edges.append((address,value))

fm=BusyFlashModel()
flash_call('board_display_init');flash_call('board_display_brightness',2);flash_call('board_display_tick')
fm.edges.clear()
assert flash_call(operation,0x08010000,0,1)
off=[(GPIO[2]+20,64),(GPIO[1]+20,8)]
on=[(GPIO[2]+16,64),(GPIO[1]+16,8)]
assert fm.edges==off+on+off+on
assert fm.odr[GPIO[2]]&64 and fm.odr[GPIO[1]]&8
print('ARM flash busy: backlight PWM continues and resumes at the original output state PASS')
fm=FlashModel()
for address,erase in ((0x08000000,1),(0x08002ffc,0),(0x08003000,1),(0x0800fffc,0),
                      (0x08010001,0),(0x08010004,1),(0x08013ffc,1),(0x08014000,0),
                      (0x08014000,1),(0xffffffff,0)):
    assert not flash_call(operation,address,0,erase)
assert not fm.controller_writes and not fm.erased and not fm.programmed
print('ARM RAM guard: ten forbidden erase/program addresses rejected before any register write PASS')
def read_string(addr):
    return bytes(fm.u.mem_read(addr,32)).split(b'\0')[0]
app=0x20010000
fresh=0x20012000
ctor='_ZN5rt95011ApplicationC1Eb'
save='_ZN5rt95013save_settingsERKNS_11ApplicationEb'
load='_ZN5rt95013load_settingsERNS_11ApplicationE'
set_frequency='_ZN5rt95011Application13set_frequencyEm'
flash_call(ctor,app,1)
assert read_string(flash_call(save,app,0))==b'CHECK BATTERY' and not fm.programmed
flash_call('board_battery_init',0)
flash_call('board_battery_poll',1)
fm.put(0x40012808,fm.get(0x40012808)&~8);flash_call('board_battery_poll',2)
fm.put(0x40012808,fm.get(0x40012808)&~4);flash_call('board_battery_poll',3)
for now in (4,5):
    fm.put(0x40012800,2);fm.put(0x4001284c,0xa00);flash_call('board_battery_poll',now)
for base,freq in ((0x08010000,145600000),(0x08012000,145625000),(0x08010000,145650000)):
    assert flash_call(set_frequency,app,freq)
    assert read_string(flash_call(save,app,0))==b'SETTINGS SAVED'
    assert fm.programmed[-1]==base+20 and fm.get(base+20)==0x54494d43
assert fm.erased and all(0x08010000<=addr<0x08012000 for addr in fm.erased)
programmed=len(fm.programmed)
flash_call(ctor,fresh,1);flash_call(load,fresh)
assert read_string(flash_call(save,fresh,0))==b'SETTINGS SAVED'
assert len(fm.programmed)==programmed
assert fm.get(0x40022010)==128
print('ARM storage controller model: low battery guard, A/B rotation, bounded sector erase, commit last and reload PASS')

auto_begin=next(name for name in symbols if 'SettingsAutosave5begin' in name)
auto_poll=next(name for name in symbols if 'SettingsAutosave4poll' in name)
auto=0x20014000
def poll_auto(now,shutdown=0,ptt=0):
    fm.put(0x20017b00,ptt) # AAPCS fifth argument
    from unicorn.arm_const import UC_ARM_REG_R3
    fm.u.reg_write(UC_ARM_REG_R3,shutdown)
    return flash_call(auto_poll,auto,fresh,now)
flash_call(auto_begin,auto,fresh,0)
assert not poll_auto(2500) and len(fm.programmed)==programmed
assert flash_call(set_frequency,fresh,145675000)
assert not poll_auto(2750) and not poll_auto(4500)
assert not poll_auto(4750,ptt=1) and len(fm.programmed)==programmed
assert read_string(poll_auto(5000))==b'SETTINGS SAVED'
programmed=len(fm.programmed)
assert flash_call(set_frequency,fresh,145700000)
assert not poll_auto(5250) and len(fm.programmed)==programmed
assert read_string(poll_auto(5300,shutdown=1))==b'SETTINGS SAVED'
assert len(fm.programmed)>programmed
flash_call(ctor,app,1);flash_call(load,app)
programmed=len(fm.programmed)
assert read_string(flash_call(save,app,0))==b'SETTINGS SAVED' and len(fm.programmed)==programmed
print('ARM autosave: no startup write, debounce, PTT guard, shutdown flush and reload PASS')

# Mixed legacy/compact storage: a newer invalid payload must never cause
# the last usable old slot to be erased during migration.
import zlib
legacy_dump=next(n for n in symbols if 'save_portable_snapshot' in n)
assert flash_call(legacy_dump,fresh,0x20015000,8192)==8008
legacy_size=7992 # Immutable prefix; the unused legacy helper is now linker-GC'd.
legacy_payload=bytes(fm.u.mem_read(0x20015000,legacy_size))
def storage_record(payload,sequence):
    header=struct.pack('<4I',0x53303952,1,sequence,len(payload))
    return header+struct.pack('<2I',zlib.crc32(header+payload),0x54494d43)+payload
legacy_record=storage_record(legacy_payload,10)
bad_record=storage_record(struct.pack('<5I',0x32535452,1,0,0,0),11)
def seed_records():
    fm.u.mem_write(0x08010000,legacy_record+b'\xff'*(8192-len(legacy_record)))
    fm.u.mem_write(0x08012000,bad_record+b'\xff'*(8192-len(bad_record)))
seed_records();old_slot=bytes(fm.u.mem_read(0x08010000,8192))
future_record=storage_record(struct.pack('<5I',0x32535452,99,0,0,0),11)
fm.u.mem_write(0x08012000,future_record+b'\xff'*(8192-len(future_record)))
old_erases=len(fm.erased);old_programmed=len(fm.programmed)
assert read_string(flash_call(save,fresh,0))==b'STORAGE FORMAT NEWER'
assert len(fm.erased)==old_erases and len(fm.programmed)==old_programmed
seed_records()
flash_call(ctor,fresh,0);flash_call(load,fresh)
assert flash_call(legacy_dump,fresh,0x20015000,8192)==legacy_size+16
assert bytes(fm.u.mem_read(0x20015000,legacy_size))==legacy_payload
fm.erased.clear();fm.programmed.clear()
assert read_string(flash_call(save,fresh,0))==b'SETTINGS SAVED'
assert fm.erased and all(0x08012000<=address<0x08014000 for address in fm.erased)
assert bytes(fm.u.mem_read(0x08010000,8192))==old_slot
assert fm.get(0x08012000+24)==0x32535452 and fm.get(0x08012000+12)<3000
assert fm.programmed[-1]==0x08012000+20
flash_call(ctor,fresh,0);flash_call(load,fresh)
assert flash_call(legacy_dump,fresh,0x20015000,8192)==legacy_size+16
assert bytes(fm.u.mem_read(0x20015000,legacy_size))==legacy_payload
# Cut the migrated write before its commit, then boot a fresh MCU model.
seed_records();fm.erased.clear();fm.programmed.clear();cut_writes=[]
def cut_migration(u,access,address,size,value,data):
    if 0x08012000<=address<0x08014000:
        cut_writes.append(address)
        if len(cut_writes)==10:u.emu_stop()
hook=fm.u.hook_add(UC_HOOK_MEM_WRITE,cut_migration)
flash_call(save,fresh,0)
assert len(cut_writes)==10 and fm.get(0x08012000+20)==0xffffffff
assert bytes(fm.u.mem_read(0x08010000,8192))==old_slot
interrupted_slots=bytes(fm.u.mem_read(0x08010000,16384))
fm=FlashModel();fm.u.mem_write(0x08010000,interrupted_slots)
flash_call(ctor,fresh,0);flash_call(load,fresh)
assert flash_call(legacy_dump,fresh,0x20015000,8192)==legacy_size+16
assert bytes(fm.u.mem_read(0x20015000,legacy_size))==legacy_payload
print('ARM compact migration: unknown version write guard, legacy fallback, invalid newer record overwritten first, old copy preserved, commit last and power-cut reboot PASS')

# Enter and exit the FM diagnostic using the compiled board code.
fm_hw=Machine(raw)
fm_poll=next(name for name in symbols if 'BoardProbe4poll' in name)
fm_object=0x20012000
def probe_poll(open_page,now):
    for register,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),(fm_object,open_page,now)):
        fm_hw.u.reg_write(register,value)
    fm_hw.function(fm_poll)
def crl(port):return struct.unpack('<I',fm_hw.u.mem_read(port,4))[0]
fm_hw.u.mem_write(GPIO[0],struct.pack('<I',0x44000000))
fm_hw.u.mem_write(GPIO[1],struct.pack('<I',0x44000000))
fm_hw.u.mem_write(GPIO[2],struct.pack('<I',0x00040000))
probe_poll(1,0)
assert crl(GPIO[2])>>16&15==2 and not fm_hw.odr[GPIO[2]]&16
assert crl(GPIO[0])>>24&15==2 and crl(GPIO[1])>>24==0x82
assert not fm_hw.odr[GPIO[0]]&64
probe_poll(1,10)
assert fm_hw.odr[GPIO[0]]&64
probe_poll(0,20)
assert crl(GPIO[0])==0x44000000 and crl(GPIO[1])==0x44000000
assert crl(GPIO[2])==0x00040000
print('ARM FM GPIO model: PC4 low, PA6 reset, PB6 push-pull, PB7 input pull-up and restore on exit PASS')

class VoiceMachine(Machine):
    def __init__(self):
        super().__init__(raw)
        self.reply_words=iter((0xa800,0x0281,55,0x0203,171))
        self.reply_bits=0
    def read(self,u,access,address,size,value,data):
        super().read(u,access,address,size,value,data)
        if address!=GPIO[4]+8 or self.odr[GPIO[4]]&0x100:return
        if not self.reply_bits:
            self.reply=self.reply_words.__next__()
            self.reply_bits=16
        self.reply_bits-=1
        bit=(self.reply>>self.reply_bits)&1
        u.mem_write(address,struct.pack('<I',bit<<11))

voice=VoiceMachine()
voice.function('board_init')
voice.route_clear.clear()
voice_object=0x20012000
def voice_call(method,a=0,b=0,c=0):
    for register,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),(voice_object,a,b,c)):
        voice.u.reg_write(register,value)
    name=next(name for name in symbols if 'BoardReceiver'+method in name and '.' not in name)
    voice.function(name)
# The direct-call model starts with zeroed RAM, so apply the C++ defaults.
voice.u.mem_write(0x20017b00,struct.pack('<II',1,3))
voice_call('11set_profile')
voice.u.mem_write(0x20017b00,bytes(8))
voice_call('6enable',1)
for now in range(46):voice_call('4poll',now,0)
voice_call('4tune',446081250,100)
voice_call('4poll',101,0)
assert not voice.odr[GPIO[1]]&1
assert not voice.odr[GPIO[1]]&0x100
voice_call('4poll',131,0)
assert not voice.odr[GPIO[1]]&0x100
voice_call('4poll',132,0,1)
assert voice.odr[GPIO[1]]&0x100 and voice.odr[GPIO[4]]&0x10 and not voice.odr[GPIO[4]]&2
voice_call('4poll',133,1,1)
assert not voice.odr[GPIO[1]]&0x100
voice_call('4poll',146,0)
assert voice.odr[GPIO[1]]&0x100
voice_call('4poll',147,1)
assert not voice.odr[GPIO[1]]&0x100 and voice.odr[GPIO[4]]&2
voice_call('6enable',0)
assert voice.odr[GPIO[1]]&3==3 and voice.odr[GPIO[4]]&0x7080==0x7080
assert voice.route_clear==[0x2000,1],voice.route_clear
print('ARM voice board: startup, RX route only, carrier audio, PTT mute, shutdown PASS')

voice=VoiceMachine()
voice.reply_words=iter((0xa800,0x0203,120,0x0203,120))
voice.function('board_init')
voice.u.mem_write(0x20017b00,struct.pack('<II',1,3))
voice_call('11set_profile')
voice.u.mem_write(0x20017b00,bytes(8))
voice_call('6enable',1)
for now in range(46):voice_call('4poll',now,0)
# AAPCS passes now, narrow and SQL in the stack slots of tune_profile.
voice.u.mem_write(0x20017b00,struct.pack('<III',100,0,3))
voice_call('12tune_profile',145500000,0,0)
voice.u.mem_write(0x20017b00,bytes(12))
voice_call('4poll',101,0)
voice_call('4poll',131,0)
assert voice.odr[GPIO[1]]&0x100
voice.u.mem_write(0x20017b00,struct.pack('<II',0,9))
voice_call('11set_profile',0,0,132)
assert not voice.odr[GPIO[1]]&0x100
voice.u.mem_write(0x20017b00,bytes(8))
voice_call('4poll',132,0)
assert not voice.odr[GPIO[1]]&0x100
print('ARM Watch RF profile: combined tune, immediate SQL mute and fresh sample at the higher threshold PASS')

voice=VoiceMachine()
voice.reply_words=iter((0xa800,2,1,0,511,2,1))
voice.function('board_init');voice.route_clear.clear()
voice.u.mem_write(0x20017b00,struct.pack('<II',1,3))
voice_call('11set_profile')
voice.u.mem_write(0x20017b00,bytes(8))
voice_call('6enable',1)
for now in range(46):voice_call('4poll',now,0)
voice_call('4tune',162550000,200)
voice.u.mem_write(0x20017b00,struct.pack('<II',0,1)) # packet audio off, WX muted
voice_call('4poll',201,0)
voice_call('4poll',231,0)
assert voice.route_clear==[0x2000,0x1000],voice.route_clear
assert not voice.odr[GPIO[1]]&0x100
voice.u.mem_write(0x20017b00,struct.pack('<II',0,2)) # WX speaker
voice_call('4poll',232,0)
assert voice.odr[GPIO[1]]&0x100 and not voice.odr[GPIO[4]]&2
voice_call('4poll',246,0)
assert not voice.odr[GPIO[1]]&0x100 # high RSSI alone does not open WX audio
voice_call('4poll',261,1)
assert not voice.odr[GPIO[1]]&0x100 # PTT mutes despite open SQL
voice_call('4poll',262,0)
assert voice.odr[GPIO[1]]&0x100
voice_call('6enable',0)
assert not voice.odr[GPIO[1]]&0x100 and voice.odr[GPIO[4]]&0x7000==0x7000
print('ARM NOAA RF: VHF route, muted scan, hardware SQL over RSSI, PTT mute and stop PASS')

# Render the new menu page in both languages using real compiled UI calls.
# Record primitive bounds before the LCD sink skips hardware operations.
class MenuBounds(Foreground):
    def call(self,address,budget=250000):
        # Each call is an isolated ABI entry; the LCD sink redirects PC and
        # clears ITSTATE. Unicorn may reuse translated Thumb blocks with stale
        # conditional state across those redirects. A fresh translation cache
        # prevents a skipped stack-address instruction on the next UI entry.
        # This changes the test model only, not the bytes being executed.
        self.u.ctl_flush_tb()
        return super().call(address,budget)
    def code(self,u,address,size,data):
        if address==symbols['lcd_fill_rect']:
            x,y,w,h=[u.reg_read(r) for r in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3)]
            assert x+w<=240 and y+h<=320,(x,y,w,h)
        if address in (symbols['lcd_draw_string'],symbols['lcd_draw_string_2x']):
            x,y,ptr=[u.reg_read(r) for r in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2)]
            text=bytearray()
            while u.mem_read(ptr,1)[0]:text.extend(u.mem_read(ptr,1));ptr+=1
            scale=2 if address==symbols['lcd_draw_string_2x'] else 1
            assert x+len(text.decode('utf-8'))*8*scale<=240 and y+8*scale<=320
        super().code(u,address,size,data)

import re
key_enum=(root/'application.hpp').read_text(encoding='utf-8').split('enum class Key : uint8_t {',1)[1].split('};',1)[0]
logical_keys={name.strip():i for i,name in enumerate(key_enum.split(','))}
menu_ui=MenuBounds();menu_app=0x20012000
ctor=next(n for n in symbols if n.startswith('_ZN5rt95011ApplicationC1Eb'))
app_key=next(n for n in symbols if n.startswith('_ZN5rt95011Application3keyE'))
draw=next(n for n in symbols if n.startswith('_ZN5rt9507ui_draw'))
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
def menu_key(name):
    menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,logical_keys[name]);menu_ui.function(app_key)
def menu_draw():
    menu_ui.texts.clear()
    for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),(menu_app,0x20015000,0)):menu_ui.u.reg_write(reg,value)
    menu_ui.function('ui_invalidate' if 'ui_invalidate' in symbols else next(n for n in symbols if 'ui_invalidate' in n))
    for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),(menu_app,0x20015000,0)):menu_ui.u.reg_write(reg,value)
    menu_ui.function(draw)
menu_key('Menu');menu_key('Up');menu_key('Up');menu_key('Ok');menu_key('Down');menu_key('Down');menu_key('Ok')
menu_draw()
assert all(label in menu_ui.texts for label in ('Menu timeout','OFF','10 seconds','30 seconds','60 seconds','120 seconds'))
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('MenuTimeout');menu_draw()
assert all(label in menu_ui.texts for label in ('Выход из меню','10 секунд','30 секунд','60 секунд','120 секунд'))
print('ARM menu timeout UI: System navigation, complete RU/EN lists and LCD bounds PASS')

# A fresh model selects the numeric target through the actual radio menu.
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
set_frequency=next(n for n in symbols if n.startswith('_ZN5rt95011Application13set_frequencyE'))
def menu_frequency(hz):
    menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,hz);menu_ui.function(set_frequency)
menu_frequency(446000000)
menu_ui.u.mem_write(0x20015000,struct.pack('<I',0x20015e00)) # bench_ptt -> empty text
menu_key('Menu');menu_key('Ok')
for _ in range(4):menu_key('Down')
menu_key('Ok');menu_draw()
assert '10 mW target' in menu_ui.texts and '0.5 W target' in menu_ui.texts
for _ in range(4):menu_key('Down')
menu_key('Ok');menu_key('PowerCalibration');menu_draw()
assert all(label in menu_ui.texts for label in ('TX power calibration','0.5 W target','DAC code','Recorded'))
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('PowerCalibration');menu_draw()
assert 'Калибровка мощности TX' in menu_ui.texts and 'Цель 0,5 Вт' in menu_ui.texts
menu_key('Left');menu_draw()
assert 'СТЕРЕТЬ ЗАМЕРЫ МОЩНОСТИ?' in menu_ui.texts
print('ARM numeric power UI: 10 mW/0.5 W choices, RU/EN calibration and clear confirmation bounds PASS')
menu_frequency(446081250)
menu_key('BandProfile');menu_draw()
assert 'Профили LPD / PMR (RU)' in menu_ui.texts and 'PMR: цель 0,5 Вт' in menu_ui.texts
assert '446.08125' in menu_ui.texts
menu_key('Right');menu_draw()
assert 'LPD: цель 10 мВт' in menu_ui.texts
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('BandProfile');menu_draw()
assert 'LPD / PMR profiles (RU)' in menu_ui.texts and 'PMR: 0.5 W target' in menu_ui.texts
print('ARM RU LPD/PMR picker UI: both bands, channel frequencies and bilingual LCD bounds PASS')
menu_key('PowerStatus');menu_draw()
assert '1/4 Battery readings' in menu_ui.texts and 'TX DISABLED ON THIS PAGE' in menu_ui.texts
menu_key('Down');menu_draw();assert '2/4 Last TX supply' in menu_ui.texts and 'NO TX HISTORY' in menu_ui.texts
menu_key('Down');menu_draw()
assert '3/4 Available sensors' in menu_ui.texts and 'NO DATA' in menu_ui.texts and 'NOT VERIFIED' in menu_ui.texts
menu_key('Down');menu_draw();assert '4/4 Activity time' in menu_ui.texts and 'GPS on s' in menu_ui.texts and 'Reset flags' in menu_ui.texts
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('PowerStatus');menu_draw()
assert '1/4 Измерения питания' in menu_ui.texts
menu_key('Down');menu_draw();assert '2/4 Питание при TX' in menu_ui.texts
menu_key('Down');menu_draw();assert '3/4 Доступные датчики' in menu_ui.texts and 'НЕ ПОДТВ.' in menu_ui.texts
menu_key('Down');menu_draw();assert '4/4 Время работы' in menu_ui.texts and 'Приём APRS, с' in menu_ui.texts
print('ARM power diagnostics UI: four read-only pages, activity time, RU/EN and LCD bounds PASS')
menu_key('MicGain');menu_draw()
assert all(label in menu_ui.texts for label in ('Чувствительность микрофона','Низкая','Средняя','Высокая (завод.)'))
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('MicGain');menu_draw()
assert all(label in menu_ui.texts for label in ('Mic Gain','Low','Middle','High (factory)'))
print('ARM microphone UI: complete RU/EN lists and LCD bounds PASS')
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_key('Menu');menu_key('Up');menu_key('Up');menu_key('Ok');menu_key('Up');menu_draw()
assert 'Lock keypad' in menu_ui.texts
menu_key('Ok');menu_draw()
assert 'LOCK' in menu_ui.texts and 'HOLD # TO UNLOCK' in menu_ui.texts
menu_key('MicGain');menu_key('Up');menu_draw()
assert 'LOCK' in menu_ui.texts and 'Mic Gain' not in menu_ui.texts
menu_key('KeypadLock');menu_draw();assert 'LOCK' not in menu_ui.texts
menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('KeypadLock');menu_draw()
assert 'ЗАМОК' in menu_ui.texts and 'УДЕРЖ. # ДЛЯ РАЗБЛОК.' in menu_ui.texts
menu_key('KeypadLock');menu_draw();assert 'ЗАМОК' not in menu_ui.texts
print('ARM keypad lock UI: System action, persistent lock hint, blocked navigation, unlock and RU/EN LCD bounds PASS')
menu_key('Monitor');menu_draw()
assert all(label in menu_ui.texts for label in ('Монитор','ВКЛ','ВЫКЛ','ОБХОД SQL И RX-СУБТОНОВ'))
menu_key('Down');menu_key('Ok');menu_key('Monitor');menu_draw()
assert 'ЗВУК АКТИВНОГО СЛОТА' in menu_ui.texts
menu_key('Back');menu_key('VoiceRx');menu_draw()
assert 'ПРОСМОТР / PTT ОТКЛЮЧЕНА' in menu_ui.texts
assert 'OK: ОБХОД   >: МОНИТОР' not in menu_ui.texts
before=menu_ui.texts[:]
menu_key('Right');menu_key('Ok');menu_key('NextSlot');menu_draw()
assert menu_ui.texts==before
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('Monitor');menu_draw()
assert all(label in menu_ui.texts for label in ('Monitor','ON','OFF','ACTIVE SLOT AUDIO'))
menu_key('Back');menu_key('VoiceRx');menu_draw()
assert 'READ ONLY / PTT DISABLED' in menu_ui.texts and 'READ ONLY / BACK RETURN' in menu_ui.texts
print('ARM monitor UI: Radio OFF/ON list, read-only diagnostics, no hidden control hints and RU/EN bounds PASS')

# Inject only the hardware-provided battery and factory code limit, using
# DWARF offsets from this ELF rather than hard-coded Application layout.
debug_text=subprocess.check_output([args.prefix+'gdb','-batch',str(build/'rt950-custom-ui.elf'),
    '-ex','p/x (unsigned long)&((rt950::Application*)0)->power_limit_',
    '-ex','p/x (unsigned long)&((rt950::Application*)0)->power_battery_mv_'],text=True,stderr=subprocess.DEVNULL)
field_offsets=[int(value,16) for value in re.findall(r'\$\d+ = (0x[0-9a-f]+)',debug_text)]
assert len(field_offsets)==2
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_ui.u.mem_write(menu_app+field_offsets[0],bytes([95]))
menu_ui.u.mem_write(menu_app+field_offsets[1],struct.pack('<H',8000))
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,446018750);menu_ui.function(set_frequency)
menu_key('BandProfile');menu_key('Ok');menu_key('PowerCalibration');menu_draw()
assert 'SET VFO TO 446.00000' in menu_ui.texts
menu_frequency(446000000);menu_key('PowerCalibration');menu_draw()
assert 'CODE IS NOT POWER' in menu_ui.texts and '50 OHM LOAD + RF METER' in menu_ui.texts
menu_key('Right');menu_draw()
assert '-- mW' in menu_ui.texts and '-- W' in menu_ui.texts and '1 mW = 0.001 W' in menu_ui.texts
app_digit=next(n for n in symbols if n.startswith('_ZN5rt95011Application5digitE'))
def menu_number(number):
    for digit in str(number):
        menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,int(digit));menu_ui.function(app_digit)
menu_key('Ok');menu_draw();assert 'ENTER MEASURED POWER' in menu_ui.texts
menu_number(0);menu_key('Ok');menu_draw();assert 'MEASUREMENT MUST BE POSITIVE' in menu_ui.texts
menu_key('Erase');menu_number(470);menu_draw()
assert '470 mW' in menu_ui.texts and '0.470 W' in menu_ui.texts
menu_key('Ok');menu_key('Right');menu_number(500);menu_draw()
assert '500 mW' in menu_ui.texts and '0.500 W' in menu_ui.texts
menu_key('Back');menu_key('Right');menu_number(501);menu_key('Ok');menu_draw()
assert 'MEASUREMENT ABOVE TARGET' in menu_ui.texts
menu_key('Back');menu_key('Back');menu_key('BandProfile');menu_key('Right');menu_key('Ok')
menu_frequency(433000000)
menu_key('PowerCalibration');menu_key('Right');menu_number(9);menu_draw()
assert '9 mW' in menu_ui.texts and '0.009 W' in menu_ui.texts
menu_key('Back');menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok')
menu_key('PowerCalibration');menu_key('Right');menu_number(9);menu_draw()
assert all(label in menu_ui.texts for label in ('Ввод показания прибора','9 мВт','0,009 Вт','1 мВт = 0,001 Вт'))
menu_key('Ok');menu_draw();assert '9 мВт' in menu_ui.texts
menu_key('Back');menu_key('PowerCalibration');menu_draw()
assert '9 мВт' in menu_ui.texts and 'КОД НЕ РАВЕН МОЩНОСТИ' in menu_ui.texts
print('ARM power units UI: explicit mW/W, 470=0.470, 500=0.500, 9=0.009, blank/zero/over-target, save and RU/EN bounds PASS')

menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,1);menu_ui.function(ctor)
menu_frequency(435125000);menu_key('Save');menu_number(2);menu_draw()
assert 'Channel number' in menu_ui.texts
menu_key('Ok');menu_draw()
assert all(label in menu_ui.texts for label in ('Replace this channel?','CH 2','446.01875','435.12500'))
menu_key('Back');menu_key('Back');menu_key('Mode');menu_number(64);menu_key('Ok');menu_draw()
assert 'CH 64' in menu_ui.texts or any('64' in label for label in menu_ui.texts)
menu_key('LanguageSelect');menu_key('Down');menu_key('Ok')
menu_key('Save');menu_key('Ok');menu_draw()
assert 'ЗАМЕНИТЬ ЭТОТ КАНАЛ?' in menu_ui.texts
print('ARM channel workflow UI: number entry, overwrite preview, cancel, recall and RU/EN bounds PASS')

# Verify the auto-lock duration list in both languages with actual ARM UI.
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_key('Back');menu_key('AutoLock');menu_draw()
assert all(label in menu_ui.texts for label in ('Auto keypad lock','OFF','10 seconds','30 seconds','60 seconds','120 seconds'))
menu_key('Down');menu_key('Ok');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('AutoLock');menu_draw()
assert all(label in menu_ui.texts for label in ('Автоблокировка клавиш','10 секунд','30 секунд','60 секунд','120 секунд'))
print('ARM auto lock UI: full RU/EN duration lists, apply and LCD bounds PASS')

# Fresh application; all UI assertions use actual compiled drawing primitives.
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_key('FmPresets');menu_draw()
assert 'FM presets' in menu_ui.texts and menu_ui.texts.count('EMPTY')==8
menu_key('Left');menu_draw();assert 'SAVE FREQUENCY HERE?' in menu_ui.texts and '100.30' in menu_ui.texts
menu_key('Ok');menu_draw();assert '100.30' in menu_ui.texts and 'PRESET SAVED' in menu_ui.texts
menu_key('Erase');menu_draw();assert 'DELETE THIS PRESET?' in menu_ui.texts
menu_key('Back');menu_key('Back');menu_key('AmPresets');menu_draw()
assert 'AM presets' in menu_ui.texts and menu_ui.texts.count('EMPTY')==8
menu_key('Left');menu_draw();assert '7100' in menu_ui.texts and 'kHz' in menu_ui.texts
menu_key('Ok');menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('FmPresets');menu_draw()
assert 'Память FM' in menu_ui.texts and '100.30' in menu_ui.texts and menu_ui.texts.count('ПУСТО')==7
menu_key('Left');menu_draw();assert 'СОХРАНИТЬ ЧАСТОТУ?' in menu_ui.texts
menu_key('Back');menu_key('Erase');menu_draw();assert 'УДАЛИТЬ СТАНЦИЮ?' in menu_ui.texts
menu_key('Back');menu_key('Back');menu_key('AmPresets');menu_draw()
assert 'Память AM' in menu_ui.texts and '7100' in menu_ui.texts
menu_key('Back');menu_key('About');menu_draw()
firmware_version=(root/'firmware_version.hpp').read_text(encoding='utf-8').split('"')[1]
assert all(text in menu_ui.texts for text in ('О прошивке',firmware_version,'ТЕСТОВАЯ / НЕ РЕЛИЗ','by wizado','ШТАТНЫЙ ЗАГРУЗЧИК СОХРАНЁН'))
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('About');menu_draw()
assert all(text in menu_ui.texts for text in ('About firmware',firmware_version,'TEST BUILD / NOT RELEASE','C++17 / GPL-3.0-or-later'))
print('ARM broadcast banks UI: eight FM/AM rows, explicit units, confirmations, empty states, RU/EN and LCD bounds PASS')
print('ARM about UI: current version, acceptance status, author/license, flash, RU/EN and LCD bounds PASS')

menu_key('Back');menu_key('FmStep');menu_draw()
assert all(text in menu_ui.texts for text in ('FM tuning step','50 kHz','100 kHz','200 kHz','MANUAL TUNING ONLY'))
menu_key('Down');menu_key('Ok');menu_key('AmStep');menu_draw()
assert all(text in menu_ui.texts for text in ('AM tuning step','1 kHz','5 kHz','9 kHz','10 kHz'))
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('FmStep');menu_draw()
assert 'Шаг настройки FM' in menu_ui.texts and 'ТОЛЬКО РУЧНАЯ НАСТРОЙКА' in menu_ui.texts
menu_key('Back');menu_key('AmStep');menu_draw();assert 'Шаг настройки AM' in menu_ui.texts
menu_key('Back');menu_key('CoordinateFormat');menu_draw()
assert all(text in menu_ui.texts for text in ('Градусы и минуты','Десятичные градусы','Градусы, минуты, секунды'))
menu_key('Down');menu_key('Down');menu_key('Ok');menu_key('CoordinateFormat');menu_draw()
assert 'Градусы, минуты, секунды' in menu_ui.texts
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('CoordinateFormat');menu_draw()
assert all(text in menu_ui.texts for text in ('Degrees + minutes','Decimal degrees','Degrees minutes seconds'))
print('ARM tuner steps UI: complete FM/AM lists, defaults/current mark, RU/EN and LCD bounds PASS')
print('ARM GPS DMS UI: three coordinate formats, apply, RU/EN and LCD bounds PASS')

# Real NMEA -> parser -> DMS UI, including stale-position markings.
menu_key('Back');menu_key('Gps');menu_ui.function('nav_reset')
payload=b'GPRMC,120000.00,A,5545.123456,S,03759.999999,W,1.00,90.0,061126,,,A'
checksum=0
for byte in payload:checksum^=byte
for byte in b'$'+payload+b'*'+f'{checksum:02X}'.encode()+b'\r\n':
    menu_ui.u.reg_write(UC_ARM_REG_R0,byte);menu_ui.u.reg_write(UC_ARM_REG_R1,1000);menu_ui.function('nav_feed')
def gps_draw_at(now):
    menu_ui.texts.clear()
    menu_ui.function('ui_invalidate' if 'ui_invalidate' in symbols else next(n for n in symbols if 'ui_invalidate' in n))
    for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),(menu_app,0x20015000,now)):menu_ui.u.reg_write(reg,value)
    menu_ui.function(draw)
gps_draw_at(1000)
assert 'LAT S 55d45\'07.40"' in menu_ui.texts and 'LON W 037d59\'59.99"' in menu_ui.texts
gps_draw_at(5000)
assert 'LAST LAT S 55d45\'07.40"' in menu_ui.texts and 'LAST LON W 037d59\'59.99"' in menu_ui.texts
assert 'LAT S 55d45\'07.40"' not in menu_ui.texts
print('ARM GPS DMS position UI: parsed S/W, fresh FIX, retained stale labels and LCD bounds PASS')

# GPS navigation is isolated from radio TX and uses only the receiver's FIX.
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_ui.function('nav_reset');menu_key('Distance');menu_draw()
assert 'NO SAVED GPS POINT' in menu_ui.texts and menu_ui.texts.count('--')>=2
menu_key('Back');menu_key('DistanceUnits');menu_draw()
assert all(label in menu_ui.texts for label in ('Distance units','Kilometers (km)','Miles (mi)','Nautical miles (nm)'))
menu_key('Down');menu_key('Ok');menu_key('Waypoint');menu_key('Ok');menu_draw()
assert 'FRESH GPS FIX REQUIRED' in menu_ui.texts
app_tick=next(n for n in symbols if n.startswith('_ZN5rt95011Application4tickE'))
def nav_payload(payload,now):
    checksum=0
    for byte in payload:checksum^=byte
    for byte in b'$'+payload+b'*'+f'{checksum:02X}'.encode()+b'\r\n':
        menu_ui.u.reg_write(UC_ARM_REG_R0,byte);menu_ui.u.reg_write(UC_ARM_REG_R1,now);menu_ui.function('nav_feed')
    menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,now);menu_ui.function(app_tick)
nav_payload(b'GPRMC,120000.00,A,0000.000000,N,00000.000000,E,1.00,90.0,071126,,,A',1000)
menu_key('Ok');menu_draw();assert 'SAVE CURRENT GPS POINT?' in menu_ui.texts
menu_key('Back');menu_key('Ok');menu_key('Ok');menu_draw()
assert all(label in menu_ui.texts for label in ('SAVED TARGET','N 0.000000','E 0.000000','GPS POINT SAVED'))
menu_key('Back');menu_key('Distance');gps_draw_at(1000)
assert '0.000 mi' in menu_ui.texts
nav_payload(b'GPRMC,120001.00,A,0100.000000,N,00000.000000,E,1.00,90.0,071126,,,A',1100)
gps_draw_at(1100)
assert '69.093 mi' in menu_ui.texts and '180.0 deg' in menu_ui.texts
gps_draw_at(4200)
assert '69.093 mi' not in menu_ui.texts and '180.0 deg' not in menu_ui.texts and 'FRESH GPS FIX REQUIRED' in menu_ui.texts
menu_key('Back');menu_key('DistanceUnits');menu_key('Up');menu_key('Ok');menu_key('Distance');gps_draw_at(1200)
assert '111.195 km' in menu_ui.texts
set_option=next(n for n in symbols if n.startswith('_ZN5rt95011Application10set_optionE'))
for on in (0,1):
    for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),(menu_app,1,on)):menu_ui.u.reg_write(reg,value)
    menu_ui.function(set_option);gps_draw_at(1200)
    if not on:assert '111.195 km' not in menu_ui.texts and 'FRESH GPS FIX REQUIRED' in menu_ui.texts
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('DistanceUnits');menu_draw()
assert all(label in menu_ui.texts for label in ('Единицы расстояния','Километры (km)','Мили (mi)','Морские мили (nm)'))
menu_key('Back');menu_key('Waypoint');menu_key('Down');menu_key('Ok');menu_draw()
assert 'УДАЛИТЬ GPS-ТОЧКУ?' in menu_ui.texts
menu_key('Back');menu_key('Back');menu_key('Distance');gps_draw_at(1200)
assert 'Расстояние до точки' in menu_ui.texts and '111.195 km' in menu_ui.texts
menu_key('Back');menu_key('Waypoint');menu_key('Down');menu_key('Ok');menu_key('Ok');menu_key('Back');menu_key('Distance');gps_draw_at(1200)
assert 'GPS-ТОЧКА НЕ ЗАДАНА' in menu_ui.texts and '111.195 km' not in menu_ui.texts
print('ARM GPS navigation UI: km/mi/nm lists, fresh point capture, confirm/cancel/clear, range/bearing, stale/GPS-off suppression, RU/EN and bounds PASS')

menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_key('FlashInfo')
menu_ui.u.mem_write(0x1ffff7e0,struct.pack('<H',1024));menu_draw()
assert all(label in menu_ui.texts for label in ('Data bytes','Free bytes','Raw bytes','COMPACT V2 / AUTO SAVE','8008'))
assert '2028' in menu_ui.texts and '6140' in menu_ui.texts
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('FlashInfo');menu_draw()
assert all(label in menu_ui.texts for label in ('Данные, Б','Свободно, Б','Без сжатия','КОМПАКТ V2 / АВТОЗАПИСЬ'))
print('ARM storage format UI: actual compact/canonical bytes, free slot capacity, read-only RU/EN and LCD bounds PASS')

# UART4 driver and the exact production export service, with RX IRQ delivery.
class BackupMachine(Machine):
    def __init__(self):
        self.wire=[];self.flash_writes=[]
        super().__init__(raw)
        self.u.mem_map(0x40013000,0x1000)
    def read(self,u,access,address,size,value,data):
        if address==0x40004c04:
            status=struct.unpack('<I',u.mem_read(0x40004c00,4))[0]
            u.mem_write(0x40004c00,struct.pack('<I',status&~47))
        super().read(u,access,address,size,value,data)
    def write(self,u,access,address,size,value,data):
        if address==0x40004c04:
            self.wire.append(value&255)
            status=struct.unpack('<I',u.mem_read(0x40004c00,4))[0]
            u.mem_write(0x40004c00,struct.pack('<I',status&~192))
        if 0x40022000<=address<0x40022020:self.flash_writes.append(address)
        super().write(u,access,address,size,value,data)
backup_machine=BackupMachine()
backup_object=symbols['_ZZ4mainE15settings_export']
backup_begin=next(n for n in symbols if 'SettingsExport' in n and '5begin' in n)
backup_poll=next(n for n in symbols if 'SettingsExport' in n and '4poll' in n)
backup_cancel=next(n for n in symbols if 'SettingsExport' in n and '6cancel' in n)
def backup_call(name,*values):
    for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),values):backup_machine.u.reg_write(reg,value)
    return backup_machine.call(symbols[name],budget=2000000)
def backup_reg(address):return struct.unpack('<I',backup_machine.u.mem_read(address,4))[0]
def backup_receive(data,status=32):
    for byte in data:
        backup_machine.u.mem_write(0x40004c00,struct.pack('<II',status,byte))
        backup_machine.function('UART4_IRQHandler')
backup_crh=0x4a897456;backup_odr=0xa55a
backup_machine.u.mem_write(GPIO[2]+4,struct.pack('<I',backup_crh))
backup_machine.odr[GPIO[2]]=backup_odr;backup_machine.u.mem_write(GPIO[2]+12,struct.pack('<I',backup_odr))
backup_machine.u.mem_write(0x4002101c,struct.pack('<I',1<<18))
backup_machine.u.mem_write(0x4000480c,struct.pack('<I',0x2024))
backup_machine.u.mem_write(0x4001380c,struct.pack('<I',0x2008))
backup_call(ctor,0x20012000,1);backup_call(backup_begin,backup_object,0x20012000,0)
assert backup_reg(0x40004c08)==521 and backup_reg(0x40004c0c)==0x202c
assert (backup_reg(GPIO[2]+4)>>8)&255==0x8a
assert backup_reg(0x08003000+4*(16+52))==(symbols['UART4_IRQHandler']|1)
# The bounded ring drops excess bytes and rejects a byte with UART errors.
backup_receive(b'X'*80);backup_receive(b'R',status=40)
backup_call(backup_poll,backup_object,1,1);backup_call(backup_poll,backup_object,2,1)
assert not backup_machine.wire
backup_receive(b'RT950CFG\n')
for now in range(3,9000):
    backup_machine.u.mem_write(0x40004c00,struct.pack('<I',128))
    backup_call(backup_poll,backup_object,now,1)
    if len(backup_machine.wire)==8056:break
assert len(backup_machine.wire)==8056 and backup_reg(0x40004c0c)!=0 # wait for TC
backup_machine.u.mem_write(0x40004c00,struct.pack('<I',192));backup_call(backup_poll,backup_object,now+1,1)
assert not backup_reg(0x40004c0c) and backup_reg(GPIO[2]+4)==backup_crh
assert backup_machine.odr[GPIO[2]]==backup_odr
assert backup_reg(0x4000480c)==0x2024 and backup_reg(0x4001380c)==0x2008
assert not backup_machine.flash_writes
import importlib.util
backup_spec=importlib.util.spec_from_file_location('settings_backup',root/'settings_backup.py')
backup_module=importlib.util.module_from_spec(backup_spec);backup_spec.loader.exec_module(backup_module)
assert backup_module.inspect_backup(bytes(backup_machine.wire))['occupied_channels']==64
backup_call(backup_begin,backup_object,0x20012000,10000)
backup_call(backup_poll,backup_object,10001,0)
assert not backup_reg(0x40004c0c) and backup_reg(GPIO[2]+4)==backup_crh
print('ARM backup UART: actual service/IRQ/vector/115200, bounded ring/errors, complete PC-validated frame, TC drain, pin restore, GPS/BLE isolation and no flash writes PASS')

if args.stock:
    class BackupOracle(Machine):
        def __init__(self):self.gpio=[];self.uart=[];super().__init__(args.stock.read_bytes())
        def code(self,u,address,size,data):
            if address in (0x080156ea,0x080259a8):
                u.mem_write(u.reg_read(UC_ARM_REG_R0),bytes(16 if address==0x080259a8 else 4))
            elif address==0x080151d4:
                self.gpio.append((u.reg_read(UC_ARM_REG_R0),bytes(u.mem_read(u.reg_read(UC_ARM_REG_R1),4))))
            elif address==0x080257e0:
                self.uart.append((u.reg_read(UC_ARM_REG_R0),bytes(u.mem_read(u.reg_read(UC_ARM_REG_R1),16))))
            elif address not in (0x080257a8,0x080256dc):
                return super().code(u,address,size,data)
            u.reg_write(UC_ARM_REG_PC,u.reg_read(UC_ARM_REG_LR))
    backup_oracle=BackupOracle();backup_oracle.call(0x08025b0c);backup_oracle.call(0x08025ae0)
    assert (0x40011000,struct.pack('<HBB',0x800,2,0x48)) in backup_oracle.gpio
    # This neighbouring helper configures PB10, not UART4 PC10. Do not use it
    # as evidence for the cable TX pin: the latter still needs hardware acceptance.
    assert (0x40010c00,struct.pack('<HBB',0x400,2,0x18)) in backup_oracle.gpio
    assert (0x40011000,struct.pack('<HBB',0x400,2,0x18)) not in backup_oracle.gpio
    assert backup_oracle.uart[0][0]==0x40004c00 and struct.unpack_from('<I',backup_oracle.uart[0][1])[0]==115200
    print('OEM 0.29 backup UART: executed PC11 RX / UART4 115200; nearby AF helper proven PB10, not PC10; cable TX pending hardware acceptance PASS')

# Device UI is bilingual; status injection uses offsets from this exact ELF.
backup_offsets_text=subprocess.check_output([args.prefix+'gdb','-batch',str(build/'rt950-custom-ui.elf'),
    '-ex','p/x (unsigned long)&((rt950::InputStatus*)0)->backup_phase',
    '-ex','p/x (unsigned long)&((rt950::InputStatus*)0)->backup_sent',
    '-ex','p/x (unsigned long)&((rt950::InputStatus*)0)->backup_total'],text=True,stderr=subprocess.DEVNULL)
backup_offsets=[int(v,16) for v in re.findall(r'\$\d+ = (0x[0-9a-f]+)',backup_offsets_text)]
assert len(backup_offsets)==3
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_key('Menu');menu_key('Up');menu_key('Up');menu_key('Ok')
for _ in range(4):menu_key('Down')
menu_key('Ok');menu_draw();assert 'Export settings' in menu_ui.texts
for phase,label in enumerate(('Export not started','Waiting for computer','Exporting settings','Backup exported','Export cancelled','Export timed out','Export error')):
    menu_ui.u.mem_write(0x20015000+backup_offsets[0],bytes([phase]))
    menu_ui.u.mem_write(0x20015000+backup_offsets[1],struct.pack('<I',32))
    menu_ui.u.mem_write(0x20015000+backup_offsets[2],struct.pack('<I',8056))
    menu_draw();assert label in menu_ui.texts and '32/8056' in menu_ui.texts
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('Backup');menu_draw()
assert all(label in menu_ui.texts for label in ('Экспорт настроек','ОШИБКА ЭКСПОРТА','PTT ОТМЕНЯЕТ ЭКСПОРТ','ТОЛЬКО ЧТЕНИЕ / БЕЗ ЗАПИСИ'))
print('ARM backup UI: System entry, every session state, progress, manual start/cancel hints, RU/EN and LCD bounds PASS')

# Both destructive operations opt in, and cancel starts selected in RU/EN.
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
for russian in (False,True):
 if russian:menu_key('LanguageSelect');menu_key('Down');menu_key('Ok')
 for action,title,retained in (
  ('ResetSettings','Сброс настроек' if russian else 'Reset settings','КАНАЛЫ / БАНКИ СОХРАНЯТСЯ' if russian else 'CHANNELS / BANKS KEPT'),
  ('ResetAll','Сброс данных' if russian else 'Reset all user data','64 ИСХОДНЫХ КАНАЛА' if russian else '64 DEFAULT CHANNELS')):
  menu_key(action);menu_draw()
  assert title in menu_ui.texts and retained in menu_ui.texts
  assert ('Отмена' if russian else 'Cancel') in menu_ui.texts
  assert ('Подтвердить сброс' if russian else 'Confirm reset') in menu_ui.texts
  menu_key('Down');menu_draw();menu_key('Back')
print('ARM reset UI: two reset scopes, explicit retained-data labels, cancel/confirm rows, RU/EN and LCD bounds PASS')

# Run the PC uploader against the actual import service, UART4 IRQ and driver.
import_frame=bytes(backup_machine.wire)
backup_machine=BackupMachine()
backup_machine.u.mem_write(GPIO[2]+4,struct.pack('<I',backup_crh))
backup_machine.odr[GPIO[2]]=backup_odr;backup_machine.u.mem_write(GPIO[2]+12,struct.pack('<I',backup_odr))
backup_machine.u.mem_write(0x4002101c,struct.pack('<I',1<<18))
backup_machine.u.mem_write(0x4000480c,struct.pack('<I',0x2024))
backup_machine.u.mem_write(0x4001380c,struct.pack('<I',0x2008))
import_object=symbols['_ZZ4mainE15settings_import']
import_begin=next(n for n in symbols if 'SettingsImport' in n and '5begin' in n)
import_poll=next(n for n in symbols if 'SettingsImport' in n and '4poll' in n)
import_apply=next(n for n in symbols if 'SettingsImport' in n and '5apply' in n)
backup_call(ctor,0x20012000,0);backup_call(set_frequency,0x20012000,145500000)
backup_call(app_key,0x20012000,logical_keys['Import'])
snapshot_symbol=next(n for n in symbols if 'save_portable_snapshot' in n)
def import_snapshot_bytes():
 length=backup_call(snapshot_symbol,0x20012000,0x20010000,8192)
 return bytes(backup_machine.u.mem_read(0x20010000,length))
import_before=import_snapshot_bytes()
backup_call(import_begin,import_object,0)
class RadioUploadSerial:
 def __init__(self):self.now=1;self.at=0
 def clock(self):return self.now/1000
 def write(self,data):
  self.now+=40 # Simulate a busy LCD foreground between bounded blocks.
  backup_receive(data)
  return len(data)
 def read(self,size):
  for _ in range(8):
   self.now+=1;backup_machine.u.mem_write(0x40004c00,struct.pack('<I',192))
   backup_call(import_poll,import_object,self.now,1)
  end=min(self.at+size,len(backup_machine.wire));result=bytes(backup_machine.wire[self.at:end]);self.at=end
  return result
radio_upload=RadioUploadSerial();backup_module.upload_backup(radio_upload,import_frame,clock=radio_upload.clock)
assert not backup_reg(0x40004c0c) and backup_reg(GPIO[2]+4)==backup_crh
assert backup_machine.odr[GPIO[2]]==backup_odr
assert backup_reg(0x4000480c)==0x2024 and backup_reg(0x4001380c)==0x2008
assert import_snapshot_bytes()==import_before and not backup_machine.flash_writes
assert not backup_call(import_apply,import_object,0x20012000,radio_upload.now)
# main synchronizes this flag after CHECKED; use the exact DWARF member offset.
import_offsets_text=subprocess.check_output([args.prefix+'gdb','-batch',str(build/'rt950-custom-ui.elf'),
 '-ex','p/x (unsigned long)&((rt950::Application*)0)->import_ready_',
 '-ex','p/x (unsigned long)&((rt950::InputStatus*)0)->import_phase',
 '-ex','p/x (unsigned long)&((rt950::InputStatus*)0)->import_received',
 '-ex','p/x (unsigned long)&((rt950::InputStatus*)0)->import_channels'],text=True,stderr=subprocess.DEVNULL)
import_offsets=[int(v,16) for v in re.findall(r'\$\d+ = (0x[0-9a-f]+)',import_offsets_text)]
assert len(import_offsets)==4
backup_machine.u.mem_write(0x20012000+import_offsets[0],bytes([1]))
backup_call(app_key,0x20012000,logical_keys['Down']);backup_call(app_key,0x20012000,logical_keys['Ok'])
assert backup_call(import_apply,import_object,0x20012000,radio_upload.now)
assert import_snapshot_bytes()!=import_before and not backup_machine.flash_writes
print('ARM import UART: actual service/IRQ with PC uploader, 40ms foreground stalls, staging without mutation, full validation/confirm, GPIO restore and no direct flash writes PASS')

menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
for russian in (False,True):
 if russian:menu_key('LanguageSelect');menu_key('Down');menu_key('Ok')
 menu_key('Import')
 for phase,label in ((0,'Import not started'),(1,'Waiting for computer'),(2,'Receiving settings'),(5,'Import cancelled'),(6,'Import timed out'),(7,'Import rejected')):
  menu_ui.u.mem_write(0x20015000+import_offsets[1],bytes([phase]))
  menu_ui.u.mem_write(0x20015000+import_offsets[2],struct.pack('<I',8056))
  menu_draw()
  assert backup_module is not None
  assert (('Импорт настроек' if russian else 'Import settings') in menu_ui.texts)
  if not russian:assert label in menu_ui.texts
 menu_ui.u.mem_write(menu_app+import_offsets[0],bytes([1]))
 menu_ui.u.mem_write(0x20015000+import_offsets[3],struct.pack('<H',64));menu_draw()
 assert ('Отмена' if russian else 'Cancel') in menu_ui.texts
 assert ('Применить импорт' if russian else 'Apply import') in menu_ui.texts
 menu_key('Down');menu_draw();menu_key('Back')
print('ARM import UI: receive/error/timeout states, manual cancel-default confirmation, channel count, RU/EN and LCD bounds PASS')

menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
for russian in (False,True):
 if russian:menu_key('LanguageSelect');menu_key('Down');menu_key('Ok')
 for key in ('Pf1Short','Pf1Hold','Pf2Short','Pf2Hold'):
  menu_key(key);menu_draw()
  assert all(label in menu_ui.texts for label in (('ВЫКЛ','Монитор','FM радио','Состояние GPS') if russian else ('OFF','Monitor','FM radio','GPS status')))
  menu_key('Down');menu_draw();menu_key('Back')
print('ARM user keys UI: four full action lists/current marks, RU/EN and LCD bounds PASS')

menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_key('Pf1Short');menu_key('Up');menu_draw()
assert 'Scan zone channels' in menu_ui.texts and 'Scan VFO' in menu_ui.texts
menu_key('Back');menu_frequency(145600000);menu_key('TxFrequency');menu_key('Left');menu_draw()
assert 'LEFT: TX = RX / RIGHT: CLEAR' in menu_ui.texts and '14560000' in menu_ui.texts
menu_key('Right');menu_draw();assert '0' in menu_ui.texts
print('ARM quick actions UI: scrolled PF list and TX draft shortcuts PASS')

menu_key('ChannelScan');menu_draw()
assert 'NO CHANNELS TO SCAN' in menu_ui.texts
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok')
menu_key('ChannelScan');menu_draw()
assert 'НЕТ КАНАЛОВ ДЛЯ ОБХОДА' in menu_ui.texts
print('ARM channel scan UI: empty scan status RU/EN and LCD bounds PASS')

menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,1);menu_ui.function(ctor)
power_status_field=subprocess.check_output([args.prefix+'gdb','-batch',str(build/'rt950-custom-ui.elf'),
    '-ex','p/x (unsigned long)&((rt950::InputStatus*)0)->power_status'],text=True,stderr=subprocess.DEVNULL)
power_status_offset=int(re.search(r'\$1 = (0x[0-9a-f]+)',power_status_field).group(1),16)
for offset in (0,power_status_offset):
    menu_ui.u.mem_write(0x20015000+offset,struct.pack('<I',0x20015e00))
menu_draw();assert 'Left: Watch slots' in menu_ui.texts
menu_key('Left');menu_draw();assert 'WATCH SLOTS' in menu_ui.texts
menu_key('Back')
observe_voice=next(n for n in symbols if 'observe_voice_slot' in n)
for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2),(menu_app,1,446018750)):
    menu_ui.u.reg_write(reg,value)
menu_ui.function(observe_voice);menu_draw()
assert 'LAST RX' in menu_ui.texts and 'Right: select RX B' in menu_ui.texts
menu_key('Right');menu_draw()
assert 'RX SLOT SELECTED' in menu_ui.texts
menu_key('LanguageSelect');menu_key('Down');menu_key('Ok');menu_key('Back');menu_draw()
assert 'ВЛЕВО: СЛОТЫ ОБХОДА' in menu_ui.texts
print('ARM Watch shortcuts UI: home entry, recent slot and explicit selection PASS')

tx_fields=subprocess.check_output([args.prefix+'gdb','-batch',str(build/'rt950-custom-ui.elf'),
    '-ex','p/x (unsigned long)&((rt950::InputStatus*)0)->voice_tx_slot',
    '-ex','p/x (unsigned long)&((rt950::Application*)0)->config_.vfo[0].tx_hz'],text=True,stderr=subprocess.DEVNULL)
tx_slot_offset,tx_frequency_offset=[int(value,16) for value in re.findall(r'\$\d+ = (0x[0-9a-f]+)',tx_fields)]
tx_slot_address=0x20015000+tx_slot_offset
menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,1);menu_ui.function(ctor)
menu_frequency(145600000)
menu_ui.u.mem_write(menu_app+tx_frequency_offset,struct.pack('<I',145000000))
menu_ui.u.mem_write(tx_slot_address,b'\xff')
menu_draw();assert 'SPLIT' in menu_ui.texts and '145.60000' in menu_ui.texts
menu_ui.u.mem_write(tx_slot_address,b'\x00')
menu_draw();assert 'TX' in menu_ui.texts and '145.00000' in menu_ui.texts and 'SPLIT' not in menu_ui.texts
menu_ui.u.mem_write(tx_slot_address,b'\xff')
menu_draw();assert '145.60000' in menu_ui.texts and 'SPLIT' in menu_ui.texts
print('ARM TX slot UI: split indication, keyed frequency and return to RX PASS')
menu_key('Channels');menu_draw();assert 'LEFT: ACTIONS / ABC: ZONES' in menu_ui.texts
menu_key('NextSlot');menu_draw();assert 'Select zone' in menu_ui.texts
menu_key('Back')
menu_key('Left');menu_draw()
assert all(label in menu_ui.texts for label in ('Rename channel','Move channel','Delete channel','Copy to VFO','Base RX MHz','Base TX MHz'))
menu_key('Back');menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok')
menu_key('Channels');menu_key('Left');menu_draw()
assert all(label in menu_ui.texts for label in ('Имя канала','Перенести канал','Удалить канал','Копировать в VFO','База RX','База TX'))
print('ARM channel actions UI: zone shortcut, VFO copy and base frequency preview RU/EN PASS')

# Change only a binding: autosave must notice it and survive a fresh MCU model.
fm=FlashModel();flash_call(ctor,fresh,0)
flash_call('board_battery_init',0);flash_call('board_battery_poll',1)
fm.put(0x40012808,fm.get(0x40012808)&~8);flash_call('board_battery_poll',2)
fm.put(0x40012808,fm.get(0x40012808)&~4);flash_call('board_battery_poll',3)
for now in (4,5):
 fm.put(0x40012800,2);fm.put(0x4001284c,0xa00);flash_call('board_battery_poll',now)
flash_call(auto_begin,auto,fresh,0)
flash_call(app_key,fresh,logical_keys['Pf1Hold'])
for _ in range(5):flash_call(app_key,fresh,logical_keys['Down'])
flash_call(app_key,fresh,logical_keys['Ok'])
assert not poll_auto(250) and not poll_auto(2000)
assert read_string(poll_auto(2250))==b'SETTINGS SAVED'
assert fm.get(0x08010000+28)==2
key_slots=bytes(fm.u.mem_read(0x08010000,16384))
fm=FlashModel();fm.u.mem_write(0x08010000,key_slots);flash_call(ctor,fresh,0);flash_call(load,fresh)
portable_dump=next(n for n in symbols if 'save_portable_snapshot' in n)
assert flash_call(portable_dump,fresh,0x20015000,8192)==8008
assert struct.unpack('<4I',bytes(fm.u.mem_read(0x20015000+7992,16)))==(0,5,2,0)
print('ARM user keys autosave: binding-only edit triggers debounce, compact v2 commit and fresh MCU reload preserves all four assignments PASS')

menu_ui.u.reg_write(UC_ARM_REG_R0,menu_app);menu_ui.u.reg_write(UC_ARM_REG_R1,0);menu_ui.function(ctor)
menu_key('Weather');menu_draw()
assert all(s in menu_ui.texts for s in ('NOAA','WX-1','162.55000','OK SCAN / BACK EXIT'))
menu_key('Down');menu_draw()
assert 'WX-10' in menu_ui.texts and '163.27500' in menu_ui.texts
menu_key('Back');menu_key('LanguageSelect');menu_key('Down');menu_key('Ok')
menu_key('WeatherAlert');menu_draw()
assert all(s in menu_ui.texts for s in ('Тревога NOAA','ПОИСК СИГНАЛА WX','OK СЛУШАТЬ / НАЗАД'))
menu_key('Ok');menu_draw()
assert 'Каналы NOAA' in menu_ui.texts and 'OK ПОИСК / НАЗАД' in menu_ui.texts
print('ARM NOAA UI: channel numbers, tuning, alert mode and LCD bounds RU/EN PASS')

menu_key('Back');menu_key('Timezone');menu_key('Left');menu_draw()
assert all(s in menu_ui.texts for s in ('ЧАСОВОЙ ПОЯС GPS','UTC +02:30','ВЛЕВО/ВПРАВО: 30 МИН','ABC: +1 ЧАС','OK СОХР. / НАЗАД ОТМЕНА'))
menu_key('Back');menu_key('LanguageSelect');menu_key('Up');menu_key('Ok')
menu_key('Timezone');menu_draw()
assert all(s in menu_ui.texts for s in ('GPS LOCAL TIMEZONE','UTC +03:00','LEFT/RIGHT: 30 MIN','ABC: +1 HOUR'))
menu_key('NextSlot');menu_key('Right');menu_draw()
assert 'UTC +04:30' in menu_ui.texts
menu_key('Ok');menu_key('Timezone');menu_draw()
assert 'UTC +04:30' in menu_ui.texts
print('ARM GPS timezone UI: cancelled draft, 30/60-minute shortcuts, applied offset and LCD bounds RU/EN PASS')
