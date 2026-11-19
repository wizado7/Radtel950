"""Render production ARM home/quick UI with its packed glyphs and GPIO-free LCD sink."""
import argparse,struct,re,subprocess,sys,hashlib
from pathlib import Path
from PIL import Image,ImageDraw
p=argparse.ArgumentParser()
p.add_argument('--prefix',required=True)
p.add_argument('--raw',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
root=Path(__file__).resolve().parent
source=(root/'test_board_arm.py').read_text(encoding='utf-8')
sys.argv=[str(root/'test_board_arm.py'),'--prefix',a.prefix,'--raw',str(a.raw)]
env={'__file__':str(root/'test_board_arm.py')}
exec(source[:source.index('startup=Machine')],env)
exec(source[source.index('screen_text=next(n for n in symbols'):source.index('m=Foreground();')],env)
Machine,Foreground,symbols=env['Machine'],env['Foreground'],env['symbols']
from unicorn import UC_HOOK_CODE
from unicorn.arm_const import *
def rgb(c):return ((c>>11&31)*255//31,(c>>5&63)*255//63,(c&31)*255//31)
screen_text=next(n for n in symbols if n.startswith('_ZN5rt9506screen4textE'))
ui_draw=next(n for n in symbols if n.startswith('_ZN5rt9507ui_draw'))
app_ctor=next(n for n in symbols if n.startswith('_ZN5rt95011ApplicationC1Eb'))
app_key=next(n for n in symbols if n.startswith('_ZN5rt95011Application3keyE'))
app=0x20012000;status=0x20015000
key_list=(root/'application.hpp').read_text(encoding='utf-8').split('enum class Key : uint8_t {',1)[1].split('};',1)[0]
keys={name.strip():i for i,name in enumerate(key_list.split(','))}
class Preview(Foreground):
 def __init__(self):
  self.canvas=Image.new('RGB',(240,320));self.primitives=[];super().__init__()
 def code(self,u,address,size,data):
  self._code(u,address,size,data)
  if u.reg_read(UC_ARM_REG_PC)!=address:
   self.resume=u.reg_read(UC_ARM_REG_PC);u.emu_stop()
 def _code(self,u,address,size,data):
  if address==symbols['lcd_fill_rect']:
   x,y,w,h=[u.reg_read(r) for r in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3)]
   c=int.from_bytes(u.mem_read(u.reg_read(UC_ARM_REG_SP),4),'little')
   assert x+w<=240 and y+h<=320,(x,y,w,h)
   self.primitives.append((x,y,w,h))
   if w and h:ImageDraw.Draw(self.canvas).rectangle((x,y,x+w-1,y+h-1),fill=rgb(c))
  if address==symbols[screen_text]:
   x,y,ptr,font=[u.reg_read(r) for r in (UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3)]
   ink,bg,limit=struct.unpack('<III',bytes(u.mem_read(u.reg_read(UC_ARM_REG_SP),12)))
   raw=bytearray()
   while u.mem_read(ptr,1)[0]:raw+=u.mem_read(ptr,1);ptr+=1
   text=raw.decode('utf-8');self.texts.append(text)
   table,pixels,count,em=struct.unpack('<IIII',bytes(u.mem_read(font,16)))
   def metric(v):return ((v*em+6)//13 if v>=0 else -((-v*em+6)//13)) if em else v
   glyphs={}
   for i in range(count):
    code,offset,w,h,advance,left,top=struct.unpack('<HHBBBbb',bytes(u.mem_read(table+10*i,9)))
    glyphs[chr(code)]=(offset,w,h,advance,left,top)
   for c in text:
    if c not in glyphs:continue
    offset,w,h,advance,left,top=glyphs[c]
    advance=metric(advance)
    if x+advance>limit:break
    left,top=metric(left),metric(top)
    width,height=metric(w),metric(h)
    if width and height and x+left>=0 and y+top>=0 and x+left+width<=limit and y+top+height<=320:
     data=bytes(u.mem_read(pixels+offset,(w*h+3)//4))
     colors=[rgb((((ink>>11)*level+(bg>>11)*(3-level)+1)//3)<<11 |
                 ((((ink>>5)&63)*level+((bg>>5)&63)*(3-level)+1)//3)<<5 |
                 (((ink&31)*level+(bg&31)*(3-level)+1)//3)) for level in range(4)]
     for iy in range(height):
      for ix in range(width):
       at=((iy*h+h//2)//height)*w+(ix*w+w//2)//width
       self.canvas.putpixel((x+left+ix,y+top+iy),colors[data[at//4]>>(2*(at%4))&3])
    x+=advance
   u.reg_write(UC_ARM_REG_CPSR,u.reg_read(UC_ARM_REG_CPSR)&~0x0600fc00)
   u.reg_write(UC_ARM_REG_PC,u.reg_read(UC_ARM_REG_LR));u.ctl_flush_tb();return
  super().code(u,address,size,data)
 def call(self,address,budget=3000000):
  self.u.ctl_flush_tb();self.resume=None
  value=super().call(address,budget)
  for _ in range(10000):
   if self.resume is None:break
   resume,self.resume=self.resume,None
   self.u.ctl_flush_tb()
   try:self.u.emu_start(resume|1,env['STOP'],count=budget)
   except Exception:
    print('fault',hex(self.u.reg_read(UC_ARM_REG_PC)),hex(self.u.reg_read(UC_ARM_REG_SP)),self.texts[-12:]);raise
  assert self.u.reg_read(UC_ARM_REG_PC)==env['STOP'],hex(self.u.reg_read(UC_ARM_REG_PC))
  return self.u.reg_read(UC_ARM_REG_R0)
 def invoke(self,name,*args):
  for reg,val in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),args):self.u.reg_write(reg,val)
  return self.call(symbols[name])
 def key(self,name):self.invoke(app_key,app,keys[name])
 def draw(self,now,reset=True):
  invalidate=next(n for n in symbols if 'ui_invalidate' in n)
  if reset:self.invoke(invalidate)
  self.texts.clear();self.primitives.clear();self.invoke(ui_draw,app,status,now)
m=Preview();m.invoke(app_ctor,app,1)
# Initialize pointer-bearing status fields without changing the application.
fields=['&((rt950::InputStatus*)0)->bench_ptt','&((rt950::InputStatus*)0)->power_status','&((rt950::InputStatus*)0)->voice_tx_slot']
dwarf=subprocess.check_output([a.prefix+'gdb','-batch',str(root/'build/rt950-custom-ui.elf'),
 *sum((['-ex','p/x (unsigned long)'+f] for f in fields),[])],text=True,stderr=subprocess.DEVNULL)
offsets=[int(v,16) for v in re.findall(r'\$\d+ = (0x[0-9a-f]+)',dwarf)]
m.u.mem_write(0x20016f00,b'\0')
for off in offsets[:2]:m.u.mem_write(status+off,struct.pack('<I',0x20016f00))
m.u.mem_write(status+offsets[2],b'\xff')
home=Image.new('RGB',(720,640));quick=Image.new('RGB',(720,640))
for light in (False,True):
 if light:
  m.key('Theme');m.key('Up');m.key('Ok');m.key('Back')
 for selected in range(3):
  if selected:m.key('NextSlot')
  m.draw(100)
  home.paste(m.canvas,(240*selected,320*int(light)))
  m.key('QuickSettings')
  m.draw(100)
  assert all(label in m.texts for label in ('Mode','RX tone','TX tone','Offset','Power','Step','Squelch','Busy lock','Reverse'))
  quick.paste(m.canvas,(240*selected,320*int(light)))
  m.key('Back')
 m.key('NextSlot')
a.output.mkdir(parents=True,exist_ok=True)
home.save(a.output/'home.png');quick.save(a.output/'quick.png')

# Repeat the production renderer in Russian and at both ratio boundaries.
m.key('LanguageSelect');m.key('Down');m.key('Ok');m.key('Back')
ru_home=Image.new('RGB',(720,640));ru_quick=Image.new('RGB',(720,640))
for light in (False,True):
 m.key('Theme');m.key('Up');m.key('Ok');m.key('Back')
 for selected in range(3):
  if selected:m.key('NextSlot')
  m.draw(100);ru_home.paste(m.canvas,(240*selected,320*int(light)))
  m.key('QuickSettings');m.draw(100);ru_quick.paste(m.canvas,(240*selected,320*int(light)))
  m.key('Back')
 m.key('NextSlot')
ru_home.save(a.output/'home-ru.png');ru_quick.save(a.output/'quick-ru.png')
for name,direction in (('equal','Down'),('maximum','Up')):
 m.key('SlotRatio')
 for _ in range(100):m.key(direction)
 m.key('Back');m.key('Back')
 edge=Image.new('RGB',(720,320))
 for selected in range(3):
  if selected:m.key('NextSlot')
  m.draw(100);edge.paste(m.canvas,(240*selected,0))
 m.key('NextSlot');edge.save(a.output/('home-'+name+'.png'))
# Verify enlargement and theme selection through the actual application.
m.key('SlotRatio')
for _ in range(94):m.key('Down')
for _ in range(34):m.key('Up')
m.key('Ok');m.key('Back')
m.key('Menu')
for _ in range(5):m.key('Down')
m.key('Ok');m.draw(100)
assert 'Экран и питание' in m.texts and '1/8' in m.texts
m.canvas.save(a.output/'display-settings-ru.png')
for _ in range(6):m.key('Down')
m.draw(100)
assert 'Размер шрифта' in m.texts and '7/8' in m.texts
m.key('Ok');m.draw(100)
assert 'Обычный' in m.texts
m.key('Right');m.draw(100)
assert 'Крупный' in m.texts
m.canvas.save(a.output/'font-size-ru.png')
m.key('Ok');m.key('Down');m.draw(100)
assert 'Экономия батареи' in m.texts and '8/8' in m.texts and 'ВЫКЛ' in m.texts
m.key('Ok');m.draw(101);assert 'ВКЛ' in m.texts
m.key('Ok');m.draw(102);assert 'АВТО 20%' in m.texts
m.key('Ok');m.key('Back');m.key('Back')
print('ARM Low Power menu: additional page, Russian labels and Off/On/Auto control PASS')
for light in (True,False):
 suffix='light' if light else 'dark'
 if not light:
  m.key('Theme');m.key('Up');m.key('Ok');m.key('Back')
 m.invoke(next(n for n in symbols if 'ui_boot_screen' in n),app)
 assert 'RADTEL' in m.texts and 'ЗАГРУЗКА...' in m.texts
 assert m.canvas.getpixel((0,0))==rgb(0xffff if light else 0x0882)
 m.canvas.save(a.output/('boot-'+suffix+'.png'))
 m.draw(100);m.canvas.save(a.output/('home-large-'+suffix+'.png'))
 m.key('QuickSettings');m.draw(100);m.canvas.save(a.output/('quick-large-'+suffix+'.png'))
 m.key('Back');m.key('Menu');m.draw(100)
 m.canvas.save(a.output/('settings-large-'+suffix+'.png'))
 m.key('Back')
# Check the real incremental paths without invalidating the display.
m.invoke(app_ctor,app,1)
m.key('Menu');m.draw(0)
assert 'System' in m.texts and 'Diagnostics' not in m.texts and 'Page 1/2  ▼' in m.texts
m.canvas.save(a.output/'menu-page-1.png')
for _ in range(7):m.key('Down')
m.draw(40,reset=False)
assert 'Diagnostics' in m.texts and 'Page 2/2  ▲' in m.texts
m.canvas.save(a.output/'menu-page-2.png')
m.key('Down');m.draw(80,reset=False)
assert 'Radio' in m.texts and 'Page 1/2  ▼' in m.texts
m.key('Back');m.draw(100)
m.key('NextSlot')
panel=rgb(0x10e4)
frames=[]
for timestamp in (140,180,220,260):
 m.draw(timestamp,reset=False)
 frames.append(m.canvas.copy())
assert [next(y for y in range(20,300) if frame.getpixel((0,y))==panel)
        for frame in frames]==[143,125,108,90]
frames[0].save(a.output/'abc-transition.gif',save_all=True,append_images=frames[1:],duration=[40,40,40,600],loop=0)
m.key('QuickSettings');m.key('Right');m.draw(300)
m.key('Right');m.draw(340,reset=False)
assert 'Mode' in m.texts and 'RX tone' in m.texts
assert all(8<=x and x+w<=155 and 126<=y and y+h<=159 for x,y,w,h in m.primitives),m.primitives
m.key('Ok');m.draw(380,reset=False)
assert '67.0' in m.texts and 'RX tone' not in m.texts
assert m.primitives==[(86,142,68,16)],m.primitives
print('ARM UI updates: seven rows, page wrap, ABC boundary frames, two dirty focus tiles and one value strip PASS')
print('ARM UI reference: all A/B/C positions, dark/light palettes, RU/EN, ratio boundaries, 3x3 grid, large font and themed startup PASS')
# Render the status transitions without forcing header invalidation.
set_option=next(n for n in symbols if 'set_option' in n)
option_names=(root/'application.hpp').read_text(encoding='utf-8').split('enum class Option : uint8_t {',1)[1].split('};',1)[0]
options={name.strip():i for i,name in enumerate(option_names.split(','))}
def set_flag(name,value):
 assert m.invoke(set_option,app,options[name],int(value))
def icon(x):
 return m.canvas.crop((x,1,x+14,18)).tobytes()
for light in (False,True):
 m.invoke(app_ctor,app,1)
 if light:
  m.key('Theme');m.key('Up');m.key('Ok');m.key('Back')
 m.invoke('board_gps_enable',0)
 m.draw(100)
 idle_gps,off_ble=icon(8),icon(28)
 assert not m.invoke('board_gps_powered')
 set_flag('Gps',False);m.draw(101,reset=False)
 off_gps=icon(8)
 assert idle_gps!=off_gps
 set_flag('Ble',True);m.draw(102,reset=False)
 on_ble=icon(28)
 assert on_ble!=off_ble and icon(8)==off_gps
 set_flag('Export',True);m.draw(103,reset=False)
 assert icon(28)==on_ble and icon(8)==off_gps
 set_flag('Gps',True);m.draw(104,reset=False)
 assert icon(8)==idle_gps and icon(28)==on_ble
 m.invoke('board_gps_enable',1);m.draw(105,reset=False)
 searching_gps=icon(8)
 assert searching_gps not in (off_gps,idle_gps)
 for byte in b'$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n':
  m.invoke('nav_feed',byte,106)
 m.draw(106,reset=False)
 assert icon(8) not in (searching_gps,idle_gps,off_gps)
 m.invoke('board_gps_enable',0);m.draw(107,reset=False)
 assert icon(8)==idle_gps and icon(28)==on_ble
 if light:
  m.key('LanguageSelect');m.key('Down');m.key('Ok');m.key('Back')
 m.key('Menu');m.key('Down');m.key('Down');m.key('Ok');m.draw(108)
 assert ('Приёмник GPS' if light else 'GPS receiver') in m.texts
 assert ('Авто' if light else 'Auto') in m.texts
 m.key('Down');m.key('Ok');m.draw(109)
 assert ('ВКЛ' if light else 'ON') in m.texts
 m.key('Ok');m.draw(110)
 assert ('ВЫКЛ' if light else 'OFF') in m.texts
print('ARM GPS header: OFF, permitted Auto idle, searching and fresh fix; BLE independent, cached update and RU/EN menu PASS')
print('RAW SHA256',hashlib.sha256(a.raw.read_bytes()).hexdigest())
