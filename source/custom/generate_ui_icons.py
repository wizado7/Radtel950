"""Rasterize the reference SVG paths into native 2-bit LCD glyphs (fonttools)."""
from pathlib import Path
from xml.etree import ElementTree as ET
from fontTools.pens.basePen import BasePen
from fontTools.svgLib.path import parse_path
class Outline(BasePen):
 def __init__(self):super().__init__(None);self.contours=[]
 def _moveTo(self,p):self.contours.append([p])
 def _lineTo(self,p):self.contours[-1].append(p)
 def _curveToOne(self,a,b,c):
  origin=self.contours[-1][-1]
  for i in range(1,25):
   t=i/24;s=1-t
   self._lineTo(tuple(s**3*origin[j]+3*s*s*t*a[j]+3*s*t*t*b[j]+t**3*c[j] for j in (0,1)))
 def _closePath(self):self._lineTo(self.contours[-1][0])
 def _endPath(self):pass
root=Path(__file__).resolve().parent
paths=ET.parse(root/'assets/ui-reference-icons.svg').getroot()
out=['// Generated from assets/ui-reference-icons.svg.','#pragma once']
for size,name in ((12,'Icons12'),(10,'Icons10')):
 pixels=[];glyphs=[]
 for code,path in zip('gbms',paths):
  pen=Outline();parse_path(path.attrib['d'],pen)
  edges=[(a,b) for contour in pen.contours for a,b in zip(contour,contour[1:])]
  levels=[]
  for y in range(size):
   for x in range(size):
    coverage=0
    for sy in range(4):
     for sx in range(4):
      px=(x+(sx+.5)/4)*24/size;py=(y+(sy+.5)/4)*24/size;winding=0
      for (ax,ay),(bx,by) in edges:
       if (ay<=py<by) or (by<=py<ay):
        cross=ax+(py-ay)*(bx-ax)/(by-ay)
        if cross>px:winding+=1 if by>ay else -1
      coverage+=winding!=0
    levels.append((coverage*3+8)//16)
  glyphs.append((ord(code),len(pixels),size,size,size,0,0))
  for i in range(0,len(levels),4):pixels.append(sum(v<<(2*j) for j,v in enumerate(levels[i:i+4])))
 out+=['inline constexpr uint8_t '+name+'Pixels[]={'+','.join(map(str,pixels))+'};',
 'inline constexpr Glyph '+name+'Glyphs[]={'+','.join('{'+','.join(map(str,g))+'}' for g in glyphs)+'};',
 'inline constexpr Font '+name+'={'+name+'Glyphs,'+name+'Pixels,4};']
(root/'ui_icon_data.hpp').write_text('namespace rt950::screen {\n'+'\n'.join(out)+'\n}\n',encoding='utf-8')
