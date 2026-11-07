"""Render real ui.cpp calls executed by ARM integration into pixel previews."""

import sys,re,runpy

from pathlib import Path

from PIL import Image,ImageDraw

root=Path(__file__).resolve().parent

scope=runpy.run_path(str(root/'test_board_arm.py'),run_name='preview_model')

Foreground=scope['Foreground'];symbols=scope['symbols']

from unicorn.arm_const import UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3,UC_ARM_REG_SP

lcd_source=(root/'../src/drivers/lcd.c').read_text()
def font_bytes(name):
 font=lcd_source.split(f'static const uint8_t {name}[] = {{')[1].split('};')[0]
 font=re.sub(r'/\*.*?\*/','',font,flags=re.S)
 return bytes(int(v,16) for v in re.findall(r'0x([0-9A-Fa-f]+)',font))
glyphs=font_bytes('font8x8')
cyrillic=font_bytes('font_cyrillic')
def rgb(c):return ((c>>11&31)*255//31,(c>>5&63)*255//63,(c&31)*255//31)

class Preview(Foreground):

 def __init__(self):

  self.canvas=Image.new('RGB',(240,320));self.saved=set();super().__init__()

 def code(self,u,address,size,data):

  if address==symbols['get_tick']:

   for at,name in [(400,'boot'),(2300,'radio'),(2640,'menu')]:

    if self.now>=at and name not in self.saved:

     self.canvas.save(root/'../../validation'/('v0.42-'+name+'.png'));self.saved.add(name)

  if address==symbols['lcd_fill_rect']:

   x,y,w,h=[u.reg_read(r) for r in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]]

   color=int.from_bytes(u.mem_read(u.reg_read(UC_ARM_REG_SP),4),'little')

   assert x+w<=240 and y+h<=320,(x,y,w,h)

   ImageDraw.Draw(self.canvas).rectangle((x,y,x+w-1,y+h-1),fill=rgb(color))

  if address in [symbols['lcd_draw_string'],symbols['lcd_draw_string_2x']]:

   x,y,ptr,fg=[u.reg_read(r) for r in [UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3]]

   bg=int.from_bytes(u.mem_read(u.reg_read(UC_ARM_REG_SP),4),'little');scale=2 if address==symbols['lcd_draw_string_2x'] else 1

   raw=bytearray()
   while True:
    byte=u.mem_read(ptr,1)[0];ptr+=1
    if not byte:break
    raw.append(byte)
   for character in raw.decode('utf-8'):
    ch=ord(character.upper()) if ord(character)>127 else ord(character)
    if 0x410<=ch<=0x42f: font,index=cyrillic,ch-0x410
    elif ch==0x401: font,index=cyrillic,32
    else: font,index=glyphs,(ch if 32<=ch<=126 else 63)-32
    assert x+8*scale<=240 and y+8*scale<=320,(x,y,ch)

    for row in range(8):

     bits=font[index*8+row]

     for col in range(8):

      color=rgb(fg if bits&(128>>col) else bg)

      ImageDraw.Draw(self.canvas).rectangle((x+col*scale,y+row*scale,x+(col+1)*scale-1,y+(row+1)*scale-1),fill=color)

    x+=8*scale

  super().code(u,address,size,data)

m=Preview();m.call(symbols['main'],budget=10000000)

a=Image.open(root/'../../validation/v0.42-radio.png');b=Image.open(root/'../../validation/v0.42-menu.png')

out=Image.new('RGB',(492,320),(8,12,18));out.paste(a,(0,0));out.paste(b,(252,0));out.resize((984,640),Image.Resampling.NEAREST).save(root/'../../validation/v0.42-preview.png')

print('UI previews from ARM calls saved; all drawn regions within 240x320 PASS')



# Exercise newly added pages with the same real ARM renderer.

class PagePreview(Preview):

 def code(self,u,address,size,data):

  if address==symbols[next(n for n in symbols if n.startswith('_ZN5rt9507ui_draw'))] and not hasattr(self,'app_ptr'):

   self.app_ptr=u.reg_read(UC_ARM_REG_R0);u.emu_stop();return

  super().code(u,address,size,data)

m=PagePreview();m.call(symbols['main'],budget=1000000)

from unicorn.arm_const import UC_ARM_REG_LR

def invoke(name,*args):

 for reg,value in zip([UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3],args):m.u.reg_write(reg,value)

 m.u.reg_write(UC_ARM_REG_SP,0x20016000);m.u.reg_write(UC_ARM_REG_LR,0x080ff001)

 try: m.u.emu_start(symbols[name]|1,0x080ff000,count=2000000)

 except Exception as exc: raise RuntimeError((name,hex(m.u.reg_read(scope['UC_ARM_REG_PC'])))) from exc

 assert m.u.reg_read(scope['UC_ARM_REG_PC'])==0x080ff000

app=m.app_ptr

key=next(n for n in symbols if n.startswith('_ZN5rt95011Application3keyE'))

setfreq=next(n for n in symbols if n.startswith('_ZN5rt95011Application13set_frequencyE'))

store=next(n for n in symbols if n.startswith('_ZN5rt95011Application5storeE'))

# Explicit ADC model input, not a physical voltage measurement.

import struct

invoke('board_battery_init',0);invoke('board_battery_poll',1)

cr=int.from_bytes(m.u.mem_read(0x40012808,4),'little');m.u.mem_write(0x40012808,struct.pack('<I',cr&~8));invoke('board_battery_poll',2)

cr=int.from_bytes(m.u.mem_read(0x40012808,4),'little');m.u.mem_write(0x40012808,struct.pack('<I',cr&~4));invoke('board_battery_poll',3)

for at in (4,5):

 m.u.mem_write(0x40012800,struct.pack('<I',2));m.u.mem_write(0x4001284c,struct.pack('<I',154<<4));invoke('board_battery_poll',at)

invoke(key,app,6);invoke(key,app,11)

m.u.mem_write(0x20015000,b'\xff\xff'+bytes(30))

draw=next(n for n in symbols if n.startswith('_ZN5rt9507ui_draw'))

invoke(draw,app,0x20015000,500)

m.canvas.save(root/'../../validation/v0.42-channels.png')

invoke(key,app,5);invoke(key,app,12);invoke(draw,app,0x20015000,501)

m.canvas.save(root/'../../validation/v0.42-battery-page.png')

print('Channels and battery pages executed and captured; bounds PASS')



invoke(key,app,5);invoke(setfreq,app,435125000);invoke(key,app,7);invoke(draw,app,0x20015000,502)

m.canvas.save(root/'../../validation/v0.42-save-channel.png')

print('Default named presets, explicit replacement page, and modeled 7.4V estimate captured PASS')



def feed_sentence(body,now):

 crc=0

 for v in body.encode():crc^=v

 for value in ('$'+body+'*'+format(crc,'02X')+'\r\n').encode():invoke('nav_feed',value,now)

feed_sentence('GPRMC,231500.00,A,5545.0000,N,03737.0000,E,2.50,90.0,021126,,,A',510)

invoke(key,app,8);invoke(draw,app,0x20015000,510)

m.canvas.save(root/'../../validation/v0.42-gps.png')

invoke(key,app,5);invoke(key,app,6);invoke(key,app,14);invoke(key,app,3);invoke(draw,app,0x20015000,511)

m.canvas.save(root/'../../validation/v0.42-brightness.png')

invoke(key,app,5);invoke(key,app,15);invoke(draw,app,0x20015000,512)

m.canvas.save(root/'../../validation/v0.42-timeout.png')

invoke(key,app,5);invoke(key,app,16);invoke(draw,app,0x20015000,513)

m.canvas.save(root/'../../validation/v0.42-timezone.png')

print('GPS course/time/date/coordinates and display/timezone editors rendered with bounds PASS (synthetic NMEA)')



# Drive the new page with the ARM renderer. No text or border may be drawn.

invoke(key,app,5);invoke(key,app,18);invoke(draw,app,0x20015000,514)

assert m.canvas.getextrema()==((0,0),(0,0),(0,0))

invoke(key,app,4);invoke(draw,app,0x20015000,515)

assert all(lo==hi and lo>0 for lo,hi in m.canvas.getextrema())

invoke(key,app,5);invoke(draw,app,0x20015000,516)

assert any(lo!=hi for lo,hi in m.canvas.getextrema())

print('LCD field test: uniform full-screen black and gray, Back redraw PASS')



# Render every GPS option row with the real ARM UI.

for action,name in ((23,'coordinate-options'),(26,'speed-options'),(27,'altitude-options')):

 invoke(key,app,5);invoke(key,app,action);invoke(draw,app,0x20015000,520)

 m.canvas.save(root/'../../validation'/('v0.42-'+name+'.png'))

print('GPS coordinate/speed/altitude lists drawn within bounds PASS')




# Switch languages through the real state machine and render representative pages.
invoke(key,app,5);invoke(key,app,28);invoke(draw,app,0x20015000,530)
assert 'Русский' in m.texts
invoke(key,app,3);invoke(key,app,4);invoke(draw,app,0x20015000,531)
assert 'Настройки' in m.texts
for action,name in ((8,'gps'),(23,'coordinate'),(26,'speed'),(27,'altitude'),(29,'vfo-step'),(9,'aprs'),(10,'debug'),(12,'battery'),(18,'lcd'),(19,'beacon-interval'),(22,'callsign'),(14,'brightness'),(15,'timeout'),(16,'timezone')):
 invoke(key,app,5);invoke(key,app,action);invoke(draw,app,0x20015000,532)
 m.canvas.save(root/'../../validation'/('v0.42-ru-'+name+'.png'))
invoke(key,app,5);invoke(key,app,5);invoke(draw,app,0x20015000,533)
m.canvas.save(root/'../../validation/v0.42-ru-radio.png')
assert 'RX ONLY' in m.texts and 'ЛИШЬ RX' not in m.texts
assert 'ЕДИНИЦЫ СКОРОСТИ GPS' in m.texts and 'ШАГ НАСТРОЙКИ VFO' in m.texts
print('Russian GPS/APRS/settings and bilingual VFO step rendered within bounds PASS')

# Enter Radio from a new menu session, then open all five profile settings.
invoke(key,app,6);invoke(key,app,4)
invoke(key,app,3);invoke(key,app,3)
for name in ('sql','bandwidth','power','busy-lock','tot','rx-ctcss','tx-ctcss','rx-dcs','tx-dcs'):
 invoke(key,app,4);invoke(draw,app,0x20015000,540)
 m.canvas.save(root/'../../validation'/('v0.42-ru-'+name+'.png'))
 invoke(key,app,5);invoke(key,app,3)
assert 'Шумоподавление (SQL)' in m.texts and 'Лимит передачи (TOT)' in m.texts
assert 'CTCSS приёма' in m.texts and 'DCS передачи' in m.texts
assert '67.0' in m.texts and 'D023N' in m.texts
print('All Russian radio profile editors and tone labels drawn within bounds PASS')

# The preceding loop leaves selection on the Repeater item.
invoke(key,app,4);invoke(draw,app,0x20015000,540)
assert any(line.startswith('RX ') and ' TX ' in line for line in m.texts)
m.canvas.save(root/'../../validation/v0.42-ru-repeater-summary.png')
for name in ('tx-frequency','shift','offset','reverse','tx-inhibit'):
 invoke(key,app,4);invoke(draw,app,0x20015000,541)
 m.canvas.save(root/'../../validation'/('v0.42-ru-'+name+'.png'))
 invoke(key,app,5);invoke(key,app,3)
assert 'Частота передачи' in m.texts and 'Величина сдвига' in m.texts
print('Russian repeater submenu and five editors drawn within bounds PASS')

for action,name in ((33,'rename-channel'),(34,'delete-channel'),(35,'select-zone'),(36,'rename-zone'),(37,'move-channel')):
 invoke(key,app,action);invoke(draw,app,0x20015000,550)
 if action!=35:invoke(key,app,4);invoke(draw,app,0x20015000,551)
 m.canvas.save(root/'../../validation'/('v0.42-ru-'+name+'.png'))
assert 'Удалить этот канал?' in m.texts and 'Зона назначения' in m.texts
print('Five Russian channel and zone workflows rendered within bounds PASS')
