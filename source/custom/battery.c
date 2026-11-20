// SPDX-License-Identifier: GPL-3.0-or-later
#include "board.h"
#include "drivers/gpio.h"
/* OEM 0.29 ADC2 init 08025B80, sample PA1 08016988, voltage 0800E948.
 * Own asynchronous state machine: no peripheral wait blocks the UI. */
#define REG(off) (*(volatile uint32_t *)(0x40012800u+(off)))
static uint32_t since,last_good,errors;
static uint16_t millivolts,raw12;
static uint8_t state,raw;
static bool fast,fast_sample;
enum { Control=0xe0000u,AdcEnable=1u };
static bool clocked(void) {return (CRM->APB2EN&CRM_APB2EN_ADC2EN)!=0;}
static void suspend_adc(void) {
    REG(8)=Control;
    (void)REG(8);
    CRM->APB2EN&=~CRM_APB2EN_ADC2EN;
}
static void wake_adc(uint32_t now) {
    CRM->APB2EN|=CRM_APB2EN_ADC2EN;
    (void)CRM->APB2EN;
    REG(8)=Control|AdcEnable;
    since=now;state=1;
}
void board_battery_fast(uint8_t enabled) {
    if(!enabled)fast_sample=false;
    fast=enabled!=0;
}
bool board_battery_fast_sample(void){return fast_sample;}
void board_battery_service_fast(void) {
    extern uint32_t get_tick(void);
    if(fast)board_battery_poll(get_tick());
}
bool board_battery_idle(void){return state==6 && !fast && !clocked();}
uint32_t board_battery_deadline(uint32_t now) {
    if(state==6 && fast==clocked())return since+(fast?BOARD_BATTERY_FAST_MS:1000u);
    if(state==7)return since+(fast?BOARD_BATTERY_FAST_MS:1000u);
    return now+1u;
}
void board_battery_init(uint32_t now) {
    CRM->CFGR=(CRM->CFGR&~0xc000u)|0x8000u; /* APB2 /6 */
    CRM->APB2EN|=CRM_APB2EN_ADC2EN;
    CRM->APB2RST|=0x400u;CRM->APB2RST&=~0x400u;
    gpio_config_pin(GPIOA,GPIO_PIN_1,GPIO_MODE_INPUT,GPIO_CNF_ANALOG);
    REG(4)=0;REG(0x2c)=0;REG(0x34)=1;
    REG(0x10)=(REG(0x10)&~56u)|56u; /* CH1, sample time 7 */
    wake_adc(now);millivolts=0;raw12=0;raw=0;
}
static void start(uint32_t now,uint8_t next) {
    REG(0)=~2u;REG(8)|=0x500000u;since=now;state=next;
}
void board_battery_poll(uint32_t now) {
    if(millivolts && (uint32_t)(now-last_good)>=5000u)millivolts=0;
    switch(state) {
    case 1:if((uint32_t)(now-since)>=1u){REG(8)|=8u;since=now;state=2;}break;
    case 2:if(!(REG(8)&8u)){REG(8)|=4u;since=now;state=3;}break;
    case 3:if(!(REG(8)&4u))start(now,4);break;
    case 4:case 5:
        if(REG(0)&2u) {
            uint16_t value=(uint16_t)REG(0x4c);REG(0)=~2u;
            if(state==4)start(now,5); /* discard first conversion, as OEM callers */
            else {
                raw12=value;
                raw=(uint8_t)((value>>4)&255u);
                if(value && value<4095u) {
                    millivolts=(uint16_t)((uint32_t)value*482u/160u);last_good=now;
                    fast_sample=fast;
                }
                else {millivolts=0;fast_sample=false;++errors;}
                since=now;state=6;
                if(!fast)suspend_adc();
            }
        }break;
    case 6:
        if(clocked()) {
            if(!fast)suspend_adc();
            else if((uint32_t)(now-since)>=BOARD_BATTERY_FAST_MS)start(now,4);
        } else if(fast || (uint32_t)(now-since)>=1000u)wake_adc(now);
        break;
    case 7:if((uint32_t)(now-since)>=(fast?BOARD_BATTERY_FAST_MS:1000u))board_battery_init(now);break;
    default:break;
    }
    if(state>=2 && state<=5 && (uint32_t)(now-since)>=BOARD_BATTERY_CONVERSION_TIMEOUT_MS) {
        ++errors;millivolts=0;fast_sample=false;suspend_adc();since=now;state=7;
    }
}
uint16_t board_battery_mv(void){return millivolts;}
uint8_t board_battery_sample_fresh(uint32_t now) {
    return millivolts!=0 && (uint32_t)(now-last_good)<2000u;
}
uint8_t board_battery_raw(void){return raw;}
uint16_t board_battery_raw12(void){return raw12;}
uint32_t board_battery_sample_age(uint32_t now){return millivolts?(uint32_t)(now-last_good):0xffffffffu;}
uint32_t board_battery_errors(void){return errors;}

/* Display policy only. 8320 mV is the observed full reading on the
 * reference radio. The ADC conversion and power protections are independent. */
uint8_t board_battery_percent_from_mv(uint16_t mv) {
    const uint32_t empty=6400u,full=8320u,span=full-empty;
    if(!mv)return 255;
    if(mv<=empty)return 0;
    if(mv>=full)return 100;
    const uint32_t percent=((mv-empty)*100u+span/2u)/span;
    return (uint8_t)(percent<100u?percent:99u);
}
