/*
 * encoder.c - Rotary encoder driver for the RT-950 Pro
 *
 * Quadrature decoding on PB4 (channel A) / PB5 (channel B).
 * Uses a 4-state Gray-code table for reliable direction detection
 * with built-in debounce, matching the OEM firmware approach
 * (V0.27 @ fw 0x0800D710) which uses a state variable + debounce counter.
 *
 * Physical v0.8 observations: detents alternate AB=00 and AB=11 (2 edges).
 * We track valid transitions and emit +1/-1 at each stable detent of the
 * step, which inherently filters noise.
 */

#include "app/encoder.h"
#include "drivers/gpio.h"
#include "rt950_pinmap.h"

/*
 * Quadrature state machine.
 *
 * 2-bit state = (B << 1) | A, sampled from PB5:PB4.
 * The Gray-code sequence for CW rotation is: 00 -> 01 -> 11 -> 10 -> 00
 * For CCW: 00 -> 10 -> 11 -> 01 -> 00
 *
 * We use a lookup table indexed by (prev_state << 2 | new_state) to
 * get direction: +1 CW, -1 CCW, 0 invalid/no-change.
 */
static const int8_t quad_table[16] = {
    /* prev\new  00   01   10   11  */
    /* 00 */      0,  +1,  -1,   0,
    /* 01 */     -1,   0,   0,  +1,
    /* 10 */     +1,   0,   0,  -1,
    /* 11 */      0,  -1,  +1,   0
};

#define ENC_DEBOUNCE_THRESHOLD  2   /* consecutive identical reads required */

static volatile uint32_t edges,invalid_edges,overruns;
static volatile uint8_t ready;
static volatile int8_t pending;
static uint8_t prev_state;          /* last accepted 2-bit Gray state */
static int8_t  accum;              /* accumulated quarter-steps */
static uint8_t debounce_count;     /* consecutive identical raw reads */
static uint8_t debounce_state;     /* state being debounced */

/* Read encoder pins and return 2-bit state: bit0 = A (PB4), bit1 = B (PB5) */
static inline uint8_t read_encoder(void)
{
    return (uint8_t)((ENC_A_PORT->IDR>>4)&3u);
}

void encoder_init(void)
{
    gpio_enable_clock(ENC_A_PORT);
    /* PB4 and PB5 as pull-up inputs before reading the initial state */
    gpio_config_pin(ENC_A_PORT, ENC_A_PIN, GPIO_MODE_INPUT, GPIO_CNF_PULL);
    gpio_config_pin(ENC_B_PORT, ENC_B_PIN, GPIO_MODE_INPUT, GPIO_CNF_PULL);

    ready=0;pending=0;edges=invalid_edges=overruns=0;
    prev_state     = read_encoder();
    debounce_state = prev_state;
    debounce_count = 0;
    accum          = 0;ready=1;
}

static int8_t decode(void)
{
    uint8_t raw = read_encoder();

    /* Debounce: require ENC_DEBOUNCE_THRESHOLD consecutive identical reads */
    if (raw != debounce_state) {
        debounce_state = raw;
        debounce_count = 1;
        return 0;
    }

    debounce_count++;
    if (debounce_count < ENC_DEBOUNCE_THRESHOLD)
        return 0;
    debounce_count = ENC_DEBOUNCE_THRESHOLD;    /* clamp to avoid overflow */

    if (raw == prev_state)
        return 0;

    /* Valid transition - look up direction */
    uint8_t idx = (uint8_t)((prev_state << 2) | raw);
    int8_t dir = quad_table[idx];
    prev_state = raw;
    ++edges;

    if (dir == 0) {
        ++invalid_edges;accum=0;
        return 0;
    }   /* invalid transition (noise or missed step) */

    /* Accumulate quarter-steps; emit at either 00 or 11 detent after two valid edges */
    accum += dir;
    if (raw==0 || raw==3) {
    if (accum >= 2) {
        accum = 0;
        return +1;  /* CW detent */
    }
    if (accum <= -2) {
        accum = 0;
        return -1;  /* CCW detent */
    }

    accum=0;
    }
    return 0;
}

uint8_t encoder_state(void) {return read_encoder();}
uint32_t encoder_edges(void) {return edges;}
uint32_t encoder_invalid_edges(void) {return invalid_edges;}

void encoder_tick(void) {
    if(!ready)return;
    int8_t step=decode();
    if(step>0){if(pending<8)++pending;else ++overruns;}
    if(step<0){if(pending> -8)--pending;else ++overruns;}
}
int8_t encoder_poll(void) {
    uint32_t mask;
    __asm volatile("mrs %0, primask\n cpsid i":"=r"(mask)::"memory");
    int8_t step=pending>0?1:pending<0?-1:0;
    pending-=step;
    __asm volatile("msr primask, %0"::"r"(mask):"memory");
    return step;
}

uint32_t encoder_overruns(void){return overruns;}
