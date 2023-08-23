#include "StripEffect.hpp"
#include <pico/stdlib.h>
#include "../sparkie_defs.hpp"
#include "../config.hpp"

#include <FreeRTOS.h>
#include <task.h>

using namespace sparkie::visuals;

StripEffect::StripEffect()
{
    this->last_update = 0;
}

void StripEffect::update(PicoLed::PicoLedController& controller)
{
    auto current_ticks = to_ms_since_boot(get_absolute_time());
    uint32_t delta_time = (current_ticks - this->last_update);

    if(this->last_update != 0)
    {
        this->animate(controller, delta_time);
    }
    
    this->last_update = current_ticks;
}