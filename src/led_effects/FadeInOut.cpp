#include "FadeInOut.hpp"
#include "colors.hpp"
#include "../config.hpp"
#include <pico/stdlib.h>
#include "../components/agent/AgentComponent.hpp"

using namespace sparkie::visuals;

FadeInOut::FadeInOut(PicoLed::Color base_color, float speed) : FadeInOut(base_color, speed, false)
{

}

FadeInOut::FadeInOut(PicoLed::Color base_color, float speed, bool loop)
{
    this->first_color = base_color;
    this->second_color = colors::BLACK;
    this->fade_time = 1000.0f / speed;
    this->reverse = false;
    this->loop = loop;
    this->curr_ticks = 0.0f;
    this->cycles_count = 0;
}


void FadeInOut::setSpeed(float speed)
{
    this->fade_time = std::max<uint32_t>(1, 1000.0 / speed);
}

void FadeInOut::reset()
{
    this->cycles_count = 0;
    this->curr_ticks = 0;
    this->reverse = false;
}

bool FadeInOut::done()
{
    return !this->loop && this->cycles_count >= 2;
}

void FadeInOut::animate(PicoLed::PicoLedController& controller, uint32_t delta_time)
{
    this->curr_ticks += delta_time;

    /**
     * If the current ticks are greater than the fade time, then we need to change the color.
     */
    if(this->curr_ticks >= this->fade_time)
    {
        if(!this->loop)
            this->cycles_count++;

        this->curr_ticks = 0;
        this->reverse = !this->reverse;
    }
    
    /**
     * Calculate the ratio of the current ticks to the fade time.
     * If the reverse flag is set, then we need to subtract the current ticks from the fade time.
     * This will make the fade effect go from the second color to the first color.
     */
    auto ratio = ((!this->reverse ? this->curr_ticks : (this->fade_time - this->curr_ticks)) / this->fade_time);

    auto curr_color = this->first_color;
    
    curr_color.red *= ratio;
    curr_color.green *= ratio;
    curr_color.blue *= ratio;

    controller.fill(curr_color, LED_OFFSET);
}

