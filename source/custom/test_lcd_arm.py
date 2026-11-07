"""LCD bus trace from the real ARM driver; controller/electrical timing not emulated."""
from pathlib import Path
import subprocess,struct,argparse
from unicorn import *
from unicorn.arm_const import *
p=argparse.ArgumentParser();p.add_argument('--prefix',required=True);p.add_argument('--raw',type=Path);p.add_argument('--stock',type=Path,required=True);a=p.parse_args()
r=Path(__file__).resolve().parent
syms={}
for line in subprocess.check_output([a.prefix+'nm','-n',str(r/'build/rt950-custom-ui.elf')],text=True).splitlines():
 t=line.split()
 if len(t)==3:syms[t[2]]=int(t[0],16)
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
for addr,size in [(0x08000000,0x100000),(0x20000000,0x18000),(0x40010000,0x3000),(0x40020000,0x4000)]:u.mem_map(addr,size)
u.mem_write(0x08003000,(a.raw or r/'build/rt950-custom-ui.bin').read_bytes())
ports={p:0xffff for p in (0x40010800,0x40010c00,0x40011000,0x40011400,0x40011800)}
trace=[];delays=[];led_on=[];bus_events=[];reset_delays=[];backlight_high=[]
def write(u,access,addr,size,value,data):
 if addr in (0x4001140c,0x40011410,0x40011414) and len(bus_events)<2048:bus_events.append((addr,value))
 if (addr==0x40011010 and value&64) or (addr==0x40010c10 and value&8):backlight_high.append((addr,value))
 if addr==0x40011010 and value&0x4000:led_on.append(value)
 for port in ports:
  if addr==port+12:ports[port]=value
  if addr==port+20:ports[port]&=~value
  if addr==port+16:
   old=ports[port];ports[port]|=value
   if port==0x40011400 and value&1 and not old&1 and not ports[port]&2:
    trace.append((bool(ports[port]&8),(ports[port]>>8)&255))

  if addr in (port+12,port+16,port+20):u.mem_write(port+12,struct.pack('<I',ports[port]&0xffff))

def delay(u,addr,size,data):
 delays.append(u.reg_read(UC_ARM_REG_R0));reset_delays.append((bool(ports[0x40011400]&4),u.reg_read(UC_ARM_REG_R0)));u.reg_write(UC_ARM_REG_PC,u.reg_read(UC_ARM_REG_LR))
u.hook_add(UC_HOOK_MEM_WRITE,write)
u.hook_add(UC_HOOK_CODE,delay,begin=syms['delay_ms'],end=syms['delay_ms'])
u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
u.emu_start(syms['lcd_init']|1,0x080ff000,count=60000000)
assert u.reg_read(UC_ARM_REG_PC)==0x080ff000,'init did not return'
cmd=[v for data,v in trace if not data]
assert cmd==[0x11,0xb2,0xb7,0xc0,0xc2,0xc3,0xc4,0xc6,0xd0,0xe8,0xe9,0xbb,0xe0,0xe1,0x36,0x3a,0x21,0x2a,0x2b,0x2c,0x29],cmd[:30]
start=trace.index((False,0x2c))+1;end=trace.index((False,0x29))
assert trace[start:end]==[(True,0)]*(240*320*2),'full black GRAM clear missing'
assert delays[:4]==[1,1,120,120]
assert reset_delays[:3]==[(True,1),(False,1),(True,120)]
assert not backlight_high,'init must remain dark until a complete foreground frame'
assert u.mem_read(0x40011404,4)==bytes.fromhex('22222222')
assert int.from_bytes(u.mem_read(0x40011400,4),'little')&0xffff==0x2222
custom_init=list(trace)
print('ARM LCD: reset delays, command order, 76800 black RGB565 pixels before display ON PASS')

# The panel driver must never enable the green LED (PC14).
# Initial port state is unknown in this standalone model; check writes directly.
assert not led_on,'LCD init must not set the green LED'
print('ARM LCD: status LED ownership, no PC14 enable PASS')

# LCD bus receives one complete glyph per Cyrillic code point in both scales.
for name,scale in (('lcd_draw_string',1),('lcd_draw_string_2x',2)):
 trace.clear()
 u.mem_write(0x20016000,bytes.fromhex('d090d091d19100')) # A, Be, lowercase Yo
 u.mem_write(0x20017000,bytes(4))
 for register,value in ((UC_ARM_REG_R0,0),(UC_ARM_REG_R1,0),(UC_ARM_REG_R2,0x20016000),(UC_ARM_REG_R3,0xffff)):
  u.reg_write(register,value)
 u.reg_write(UC_ARM_REG_SP,0x20017000);u.reg_write(UC_ARM_REG_LR,0x080ff001)
 u.emu_start(syms[name]|1,0x080ff000,count=3000000)
 assert u.reg_read(UC_ARM_REG_PC)==0x080ff000
 starts=[i for i,item in enumerate(trace) if item==(False,0x2c)]
 assert len(starts)==3
 for i,start in enumerate(starts):
  pixels=trace[start+1:start+1+128*scale*scale]
  assert len(pixels)==128*scale*scale and all(is_data for is_data,_ in pixels)
  assert any(value for _,value in pixels)
  assert trace[start-9:start-5]==[(True,0),(True,8*scale*i),(True,0),(True,8*scale*(i+1)-1)]
print('ARM LCD: UTF-8 Cyrillic at both glyph sizes, including lowercase Yo PASS')


# Compare actual OEM init parameters, not a second hand-copied expected table.
import hashlib
stock=a.stock.read_bytes()
assert hashlib.sha256(stock).hexdigest()=='025bc583802a96911836e274cdfdf3ce05d9919bc77451b7f7f1573fb5571907'
def oracle():
 v=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
 for addr,size in [(0x08000000,0x100000),(0x20000000,0x18000),(0x40010000,0x3000),(0x40020000,0x4000)]:v.mem_map(addr,size)
 v.mem_write(0x08003000,stock)
 return v
v=oracle();oem_trace=[];oem_delays=[];oem_reset=[];oem_gpio=[]
def oem_code(v,address,size,data):
 if address in (0x0802a5d4,0x0802a634):oem_trace.append((address==0x0802a634,v.reg_read(UC_ARM_REG_R0)))
 elif address==0x0800d906:oem_delays.append(v.reg_read(UC_ARM_REG_R0))
 elif address==0x0802a584:assert v.reg_read(UC_ARM_REG_R0)==0
 elif address==0x080151d4:oem_gpio.append((v.reg_read(UC_ARM_REG_R0),bytes(v.mem_read(v.reg_read(UC_ARM_REG_R1),4))))
 elif address not in (0x0801d248,0x0801d210,0x0801d1f4,0x0801527c):return
 v.reg_write(UC_ARM_REG_PC,v.reg_read(UC_ARM_REG_LR))
def oem_write(v,access,address,size,value,data):
 if address in (0x40011410,0x40011414) and value==4:oem_reset.append(address==0x40011410)
v.hook_add(UC_HOOK_CODE,oem_code);v.hook_add(UC_HOOK_MEM_WRITE,oem_write)
def invoke(v,address):
 v.reg_write(UC_ARM_REG_CPSR,v.reg_read(UC_ARM_REG_CPSR)&~0x0600fc00)
 v.reg_write(UC_ARM_REG_SP,0x20017000);v.reg_write(UC_ARM_REG_LR,0x080ff001)
 v.emu_start(address|1,0x080ff000,count=1000000)
 assert v.reg_read(UC_ARM_REG_PC)==0x080ff000
# The sole intentional extra command block clears the visible GRAM.
clear_start=custom_init.index((False,0x2a));clear_end=custom_init.index((False,0x29))
init_trace=custom_init[:clear_start]+custom_init[clear_end:]
invoke(v,0x0802a770)
assert init_trace==oem_trace,'OEM/custom init parameters differ'
assert oem_delays==[1,1,120,120] and oem_reset==[True,False,True]
invoke(v,0x08016a84)
assert (0x40011400,struct.pack('<HBB',0xff0f,2,0x10)) in oem_gpio
for x1,y1,x2,y2 in ((0,0,239,319),(239,319,239,319),(0,255,23,319),(15,16,103,199)):
 trace.clear();oem_trace.clear()
 for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),(x1,y1,x2,y2)):u.reg_write(reg,value)
 invoke(u,syms['lcd_set_window'])
 for reg,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),(x1,x2+1,y1,y2+1)):v.reg_write(reg,value)
 invoke(v,0x0801f82a)
 assert trace==oem_trace
for color in (0,0xf800,0x07e0,0x001f,0xffff,0x1234):
 oem_trace.clear();v.reg_write(UC_ARM_REG_R0,color);invoke(v,0x080266a8)
 assert oem_trace==[(True,color>>8),(True,color&255)]
print('ARM LCD window: actual OEM inclusive/exclusive conversion, four edge rectangles and RGB565 high-byte-first oracle PASS')
print('ARM LCD OEM 0.29: exact complete init command/data stream, reset polarity/delays, GPIO speed 2 and full GRAM clear addition PASS')
# Probe every byte in command/data mode and require the same control ordering.
# Canonical bus events: CS low, DC mode, WR low, data ODR, WR high, CS high.
bus_oracle=oracle();oem_bus=[]
def bus_write(v,access,address,size,value,data):
 if address in (0x4001140c,0x40011410,0x40011414):
  oem_bus.append((address,value&0xffff))
  old=int.from_bytes(v.mem_read(0x4001140c,4),'little')
  if address!=0x4001140c:v.mem_write(0x4001140c,struct.pack('<I',(old|value) if address==0x40011410 else old&~value))
bus_oracle.hook_add(UC_HOOK_MEM_WRITE,bus_write)
for data_mode,name in ((False,'lcd_write_command'),(True,'lcd_write_data')):
 for byte in range(256):
  initial=ports[0x40011400]
  bus_events.clear();trace.clear();u.reg_write(UC_ARM_REG_R0,byte)
  invoke(u,syms[name])
  assert trace==[(data_mode,byte)]
  assert bus_events[0]==(0x40011414,2)
  assert bus_events[1]==(0x40011410 if data_mode else 0x40011414,8)
  assert bus_events[2]==(0x40011414,1)
  assert bus_events[3][0]==0x4001140c and (bus_events[3][1]>>8)&255==byte
  assert bus_events[4:]==[(0x40011410,1),(0x40011410,2)]
  oem_bus.clear();bus_oracle.mem_write(0x4001140c,struct.pack('<I',initial))
  bus_oracle.reg_write(UC_ARM_REG_R0,byte);invoke(bus_oracle,0x0802a634 if data_mode else 0x0802a5d4)
  assert [(a,v&0xffff) for a,v in bus_events]==oem_bus
print('ARM LCD bus: all 256 command/data values, CS/DC/WR/data order, RGB565 window/GRAM and preserved matrix pins PASS')
# Foreground owns brightness/reveal; sleep/wake must leave panel registers alone.
trace.clear();backlight_high.clear();invoke(u,syms['board_display_init'])
for _ in range(16):invoke(u,syms['board_display_tick'])
assert not backlight_high and not trace
u.reg_write(UC_ARM_REG_R0,1);invoke(u,syms['board_display_brightness'])
for _ in range(8):invoke(u,syms['board_display_tick'])
assert backlight_high
trace.clear();invoke(u,syms['board_display_sleep']);invoke(u,syms['board_display_begin_wake'])
assert not trace and not ports[0x40011000]&64 and not ports[0x40010c00]&8
u.reg_write(UC_ARM_REG_R0,2);invoke(u,syms['board_display_brightness'])
for _ in range(8):invoke(u,syms['board_display_tick'])
invoke(u,syms['board_display_shutdown']);backlight_high.clear()
# Even a stale foreground brightness call cannot rearm shutdown PWM.
u.reg_write(UC_ARM_REG_R0,4);invoke(u,syms['board_display_brightness'])
for _ in range(32):invoke(u,syms['board_display_tick'])
assert not backlight_high and not trace
print('ARM LCD power: dark init, saved-level reveal, backlight-only sleep/wake and latched-off PWM after shutdown PASS')
