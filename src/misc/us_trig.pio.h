// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------- //
// us_trig //
// ------- //

#define us_trig_wrap_target 0
#define us_trig_wrap 7

static const uint16_t us_trig_program_instructions[] = {
            //     .wrap_target
    0xc020, //  0: irq    wait 0                     
    0xe001, //  1: set    pins, 1                    
    0xe033, //  2: set    x, 19                      
    0xa0c1, //  3: mov    isr, x                     
    0x4066, //  4: in     null, 6                    
    0xa026, //  5: mov    x, isr                     
    0x0046, //  6: jmp    x--, 6                     
    0xe000, //  7: set    pins, 0                    
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program us_trig_program = {
    .instructions = us_trig_program_instructions,
    .length = 8,
    .origin = -1,
};

static inline pio_sm_config us_trig_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + us_trig_wrap_target, offset + us_trig_wrap);
    return c;
}
#endif
