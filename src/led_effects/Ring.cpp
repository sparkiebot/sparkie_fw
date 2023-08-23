#include "Ring.hpp"
#include "../config.hpp"
#include "colors.hpp"
#include <pico/stdlib.h>


using namespace sparkie::visuals;

Ring::Ring(PicoLed::Color color, uint size, float speed)
{
    this->base_color = colors::IDLE;
    this->color = color;
    this->size = size;
    this->speed = speed;
    this->pos = 0.0;
    this->led_count = LED_LENGTH;
}

void Ring::reset()
{
    this->pos = 0.0;
}

bool Ring::done()
{
    return false;
}

void Ring::animate(PicoLed::PicoLedController& controller, uint32_t delta_time)
{
    if(((uint)this->pos) % this->led_count == 0)
    {
        this->pos = 0.0;
    }

    this->pos += this->speed * this->led_count * (delta_time / 1000.0f);
    
    controller.fill(this->color, this->pos + LED_OFFSET, this->size);

    if(this->pos + this->size > this->led_count)
    {
        controller.fill(this->color, LED_OFFSET, (this->pos + this->size) - this->led_count);
    }

}
