// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui_text.hpp"
extern "C" {
#include "drivers/lcd.h"
}
namespace rt950::screen {
uint16_t color(unsigned c){return uint16_t(((c>>19)&31u)<<11|((c>>10)&63u)<<5|((c>>3)&31u));}
namespace {
unsigned codepoint(const char *&p) {
    const auto first=uint8_t(*p++);
    if(first<128)return first;
    if((first&0xe0)==0xc0 && *p){const unsigned n=uint8_t(*p++)&63;return ((first&31u)<<6)|n;}
    if((first&0xf0)==0xe0 && p[0] && p[1]) {
        const unsigned high=uint8_t(*p++)&63,low=uint8_t(*p++)&63;
        return ((first&15u)<<12)|(high<<6)|low;
    }
    return '?';
}
const Glyph *glyph(const Font &font,unsigned c) {
    // The UI atlases begin with contiguous ASCII and Cyrillic ranges.
    // Icons and numeric fonts only contain the characters they display.
    unsigned start=0;
    if(font.glyphs==Sans13Glyphs || font.glyphs==Sans11Glyphs) {
        if(c>=32 && c<=126)return &font.glyphs[c-32];
        if(c>=0x410 && c<=0x44f)return &font.glyphs[95+c-0x410];
        start=159;
    }
    for(unsigned i=start;i<font.count;++i)if(font.glyphs[i].code==c)return &font.glyphs[i];
    return nullptr;
}
int metric(int n,const Font &font) {
    return font.em ? (n*int(font.em)+(n<0?-6:6))/13 : n;
}
uint16_t blend(uint16_t ink,uint16_t bg,unsigned level) {
    const unsigned r=((ink>>11)*level+(bg>>11)*(3u-level)+1u)/3u;
    const unsigned g=(((ink>>5)&63u)*level+((bg>>5)&63u)*(3u-level)+1u)/3u;
    const unsigned b=((ink&31u)*level+(bg&31u)*(3u-level)+1u)/3u;
    return uint16_t(r<<11|g<<5|b);
}
}
unsigned text_width(const char *s,const Font &font) {
    unsigned width=0;
    while(*s){const auto *g=glyph(font,codepoint(s));if(g)width+=metric(g->advance,font);}
    return width;
}
const Font &fit(const char *s,const Font &preferred,unsigned width) {
    if(text_width(s,preferred)<=width)return preferred;
    if(&preferred!=&Sans10 && text_width(s,Sans11)<=width)return Sans11;
    return Sans10;
}
void fill(int x,int y,unsigned width,unsigned height,uint16_t c) {
    if(x<0 || y<0 || x>=240 || y>=320)return;
    lcd_fill_rect(uint16_t(x),uint16_t(y),uint16_t(width),uint16_t(height),c);
}
void border(int x,int y,unsigned w,unsigned h,uint16_t c) {
    if(!w || !h)return;
    fill(x,y,w,1,c);fill(x,y+int(h)-1,w,1,c);
    fill(x,y,1,h,c);fill(x+int(w)-1,y,1,h,c);
}
void text(int x,int y,const char *s,const Font &font,uint16_t ink,uint16_t bg,unsigned limit) {
    while(*s) {
        const Glyph *g=glyph(font,codepoint(s));
        if(!g)continue;
        const int advance=metric(g->advance,font);
        if(x+advance>int(limit))return;
        const int left=x+metric(g->left,font),top=y+metric(g->top,font);
        const unsigned width=metric(g->width,font),height=metric(g->height,font);
        if(width && height && left>=0 && top>=0 && left+int(width)<=int(limit) && top+int(height)<=320) {
            lcd_set_window(uint16_t(left),uint16_t(top),uint16_t(left+width-1),uint16_t(top+height-1));
            lcd_gram_write();
            // Blend the four possible coverage values once per glyph.
            uint16_t colors[4]={bg,blend(ink,bg,1),blend(ink,bg,2),ink};
            for(unsigned row=0;row<height;++row) {
                const unsigned start=((row*g->height+g->height/2)/height)*g->width;
                for(unsigned col=0;col<width;++col) {
                    const unsigned i=start+(col*g->width+g->width/2)/width;
                    const unsigned level=(font.pixels[g->offset+i/4u]>>((i%4u)*2u))&3u;
                    const uint16_t c=colors[level];
                    lcd_write_data(uint8_t(c>>8));lcd_write_data(uint8_t(c));
                }
            }
        }
        x+=advance;
    }
}
}
