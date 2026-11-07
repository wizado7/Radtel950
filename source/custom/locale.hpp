// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <stdint.h>
namespace rt950 {
enum class Language : uint8_t { English, Russian };
const char *translate(Language language,const char *english);
}
