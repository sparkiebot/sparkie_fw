#include "BuzzerComponent.hpp"
#include "../config.hpp"
#include "../hube_defs.hpp"
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <limits.h>
#include <iostream>

using namespace hubbie;

void BuzzerComponent::tone(uint32_t freq)
{
    uint slice_num = pwm_gpio_to_slice_num(BUZZ_PIN);
    uint chan = pwm_gpio_to_channel(BUZZ_PIN);

    if(freq <= 0)
    {
        pwm_set_gpio_level(BUZZ_PIN, 0);
        return;
    }
        
    uint32_t clock = 125000000;
    uint32_t divider16 = clock / freq / 4096 + 
                            (clock % (freq * 4096) != 0);
    
    if (divider16 / 16 == 0)
        divider16 = 16;
    
    uint32_t wrap = clock * 16 / divider16 / freq - 1;
    
    pwm_set_clkdiv_int_frac(slice_num, divider16/16,
                                        divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * 1/2);
}

void BuzzerComponent::tone(uint32_t freq, uint32_t duration)
{
    tone(freq);
    vTaskDelay(duration / portTICK_PERIOD_MS);
}

void BuzzerComponent::init()
{
    gpio_set_function(BUZZ_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZ_PIN);
    auto default_config = pwm_get_default_config();
    pwm_init(slice_num, &default_config, true);
}

void BuzzerComponent::play(const BuzzerAction action)
{
    int tempo = 2;

    switch (action)
    {
    case PLAY_STARTUP:
        tone(NOTE_E6,125 / tempo);
        vTaskDelay(130 / tempo);
        tone(NOTE_G6,125 / tempo);
        vTaskDelay(130 / tempo);
        tone(NOTE_E7,125 / tempo);
        vTaskDelay(130 / tempo);
        tone(NOTE_C7,125 / tempo);
        vTaskDelay(130 / tempo);
        tone(NOTE_D7,125 / tempo);
        vTaskDelay(130 / tempo);
        tone(NOTE_G7,125 / tempo);
        vTaskDelay(125 / tempo);
        tone(NOTE_MUTE);
        break;
    case CONNECTED:
        tone(NOTE_B5,200 / tempo);
        tone(NOTE_E6,550 / tempo);
        tone(NOTE_MUTE);
        break;
    default:
        break;
    }
}

        