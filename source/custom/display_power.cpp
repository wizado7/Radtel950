// SPDX-License-Identifier: GPL-3.0-or-later
#include "board.h"
extern "C" {
#include "drivers/gpio.h"
#include "drivers/lcd.h"
}

namespace {

/* PC6 drives the screen backlight (TIM8 CH1), PB3 drives the keypad light (TIM2 CH2).
 * Both timers follow SYSCLK. Hardware PWM also runs with interrupts masked. */
volatile uint32_t &timer8_main_output() {
    return *reinterpret_cast<volatile uint32_t *>(TIM8_BASE+0x44u);
}
constexpr uint32_t PanelDelay=120;
unsigned period() {return clock_mhz(clock_mult())*40u;}
enum class Panel {Awake,Asleep,Waking,Stopped};
Panel panel=Panel::Stopped;
uint32_t transition_at;

void start_pwm(TIM_TypeDef *timer,uint32_t mode,uint32_t channel) {
    timer->CR1=0;timer->CR2=0;timer->SMCR=0;timer->DIER=0;
    timer->PSC=0;timer->ARR=period()-1;timer->CCR1=0;timer->CCR2=0;
    timer->CCMR1=mode;timer->CCER=channel;timer->EGR=1;timer->SR=0;
    timer->CR1=0x81;
}
uint16_t applied=0;
bool keypad_enabled=true;
void lights(uint16_t permille) {
    if(permille==applied)return;
    applied=permille;
    const uint32_t screen=(uint32_t)permille*period()/1000u;
    TIM8->CCR1=screen;TIM2->CCR2=keypad_enabled?screen/2u:0;
}
void pwm_pins(bool timer) {
    const auto mode=timer?GPIO_MODE_OUT_50MHZ:GPIO_MODE_OUT_2MHZ;
    const auto kind=timer?GPIO_CNF_AF_PP:GPIO_CNF_PP;
    gpio_config_pin(GPIOC,GPIO_PIN_6,mode,kind);
    gpio_config_pin(GPIOB,GPIO_PIN_3,mode,kind);
}
void enable_pwm() {
    CRM->APB2EN|=CRM_APB2EN_TIM8EN;
    CRM->APB1EN|=CRM_APB1EN_TIM2EN;
    timer8_main_output()=0;
    start_pwm(TIM8,0x68,1);
    timer8_main_output()=1u<<15;
    start_pwm(TIM2,0x6800,0x10);
    applied=0;
    pwm_pins(true);
}
void disable_pwm() {
    // Hold both outputs low on GPIO before timer ownership is removed.
    lcd_backlight_off();
    pwm_pins(false);
    if(panel!=Panel::Awake)return;
    lights(0);
    timer8_main_output()=0;
    TIM8->CR1=0;TIM2->CR1=0;
    TIM8->CCER=0;TIM2->CCER=0;
    CRM->APB2EN&=~CRM_APB2EN_TIM8EN;
    CRM->APB1EN&=~CRM_APB1EN_TIM2EN;
}
}

void board_display_init(uint32_t now) {
    CRM->APB2EN|=CRM_APB2EN_AFIOEN;
    AFIO->MAPR=(AFIO->MAPR&~((3u<<8)|AFIO_MAPR_SWJ_CFG_MASK))|(1u<<8)|(2u<<24);
    enable_pwm();
    transition_at=now;panel=Panel::Awake;
}
void board_display_keypad(bool enabled) {
    if(keypad_enabled==enabled)return;
    keypad_enabled=enabled;
    if(panel==Panel::Awake)TIM2->CCR2=enabled?TIM8->CCR1/2u:0;
}
void board_display_retime(void) {
    if(panel!=Panel::Awake)return;
    TIM8->ARR=TIM2->ARR=period()-1u;
    const auto level=applied;
    applied=0xffffu;
    lights(level);
    TIM8->EGR=TIM2->EGR=1;
}
void board_display_brightness(uint16_t permille) {
    if(panel==Panel::Awake)lights(permille<=1000u?permille:1000u);
}
bool board_display_poll(bool requested,uint32_t now) {
    const bool settled=(uint32_t)(now-transition_at)>=PanelDelay;
    switch(panel) {
    case Panel::Awake:
        if(requested)return true;
        lights(0);
        if(settled) {
            disable_pwm();
            lcd_display_on(0);lcd_sleep(1);
            transition_at=now;panel=Panel::Asleep;
        }
        break;
    case Panel::Asleep:
        if(requested && settled) {
            lcd_sleep(0);transition_at=now;panel=Panel::Waking;
        }
        break;
    case Panel::Waking:
        if(settled) {
            if(requested) {
                lcd_display_on(1);enable_pwm();panel=Panel::Awake;return true;
            }
            lcd_display_on(0);lcd_sleep(1);
            transition_at=now;panel=Panel::Asleep;
        }
        break;
    case Panel::Stopped:break;
    }
    return false;
}
void board_display_shutdown(void) {
    disable_pwm();
    panel=Panel::Stopped;
}

/* Actual RX/TX state will call this when the RF driver is connected.
 * Orange is provisionally red+green; no RGB/blue channel is assumed. */
void board_radio_led(uint8_t rx,uint8_t tx) {
    uint16_t mask=tx?(GPIO_PIN_13|GPIO_PIN_14):rx?GPIO_PIN_14:0;
    gpio_clear_pin(GPIOC,(uint16_t)((GPIO_PIN_13|GPIO_PIN_14)&~mask));
    gpio_set_pin(GPIOC,mask);
}
