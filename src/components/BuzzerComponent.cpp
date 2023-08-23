#include "BuzzerComponent.hpp"
#include <FreeRTOS.h>
#include <task.h>
#include "../config.hpp"
#include "../sparkie_defs.hpp"
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <limits.h>
#include <iostream>

using namespace sparkie;

PicoPwm* sparkie::BuzzerComponent::pwm = nullptr;

void BuzzerComponent::tone(uint freq)
{
    #ifndef BUZZ_MUTE
    if(freq == NOTE_MUTE)
    {
        pwm->stop();
        return;
    }
     
    pwm->setFrequency(freq);
    pwm->setDutyPercentage(50);
    #endif
}

void BuzzerComponent::tone(uint freq, uint duration)
{
    tone(freq);
    vTaskDelay(duration / portTICK_PERIOD_MS);
}

void BuzzerComponent::init()
{
    pwm = new PicoPwm(BUZZ_PIN);
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
    case ERROR:
        tone(NOTE_B6,200 / tempo);
        tone(NOTE_E7,550 / tempo);
        tone(NOTE_MUTE);
        break;
    default:
        break;
    }
}

        