// SPDX-License-Identifier: GPL-3.0-or-later
#include "board.h"
#include "drivers/gpio.h"
#include "drivers/lcd.h"
/* PC6/PB3 backlight enable pins; no alternate-function/timer remap assumed.
 * Four levels via SysTick, 250Hz PWM (50% has 500Hz edges). Verify on panel. */
static volatile uint8_t ready;
/* The flash wait loop continues this PWM while SysTick is masked. */
volatile uint8_t board_backlight_duty,board_backlight_phase;
#define duty board_backlight_duty
#define phase board_backlight_phase
static uint8_t lit;
void board_display_init(void){ready=0;phase=0;lit=0;duty=0;lcd_backlight_off();ready=1;}
void board_display_brightness(uint8_t level){duty=level<=4?level:4;}
void board_display_tick(void) {
    if(!ready)return;
    phase=(uint8_t)((phase+1u)&3u);
    uint8_t on=phase<duty;
    if(on!=lit){lit=on;if(on)lcd_backlight_on();else lcd_backlight_off();}
}
/* OEM 0.29 0x08017DCC only changes PC6/PB3, not panel registers.
 * Leave inversion/gamma/scan running while the light is off. */
void board_display_sleep(void){ready=0;duty=0;lit=0;lcd_backlight_off();ready=1;}
void board_display_begin_wake(void){board_display_sleep();}
/* The shutdown loop still receives SysTick: latch PWM off before releasing power. */
void board_display_shutdown(void){ready=0;duty=0;lit=0;lcd_backlight_off();}
/* Actual RX/TX state will call this when the RF driver is connected.
 * Orange is provisionally red+green; no RGB/blue channel is assumed. */
void board_radio_led(uint8_t rx,uint8_t tx) {
    uint16_t mask=tx?(GPIO_PIN_13|GPIO_PIN_14):rx?GPIO_PIN_14:0;
    gpio_clear_pin(GPIOC,(uint16_t)((GPIO_PIN_13|GPIO_PIN_14)&~mask));
    gpio_set_pin(GPIOC,mask);
}
