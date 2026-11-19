// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
#include "ui_font_data.hpp"
namespace rt950::screen {
uint16_t color(unsigned hex);
const Font &fit(const char *,const Font &,unsigned width);
unsigned text_width(const char *,const Font &);
void text(int x,int y,const char *,const Font &,uint16_t ink,uint16_t background,unsigned limit=240);
void fill(int x,int y,unsigned width,unsigned height,uint16_t);
void border(int x,int y,unsigned width,unsigned height,uint16_t);
}
