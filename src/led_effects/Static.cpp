#include "Static.hpp"
#include <pico/stdlib.h>
#include "../config.hpp"
#include "../components/agent/AgentComponent.hpp"

using namespace sparkie::visuals;

Static::Static(PicoLed::Color color)
{
    this->color = color;
}

void Static::reset()
{
}

bool Static::done()
{
    return false;
}

void Static::animate(PicoLed::PicoLedController& controller, uint32_t delta_time)
{
    controller.fill(this->color, LED_OFFSET);
}
