#pragma once

#include "StripEffect.hpp"

namespace sparkie::visuals
{
    class Ring : public StripEffect
    {
    public:
        Ring(PicoLed::Color color, uint size, float speed);
        virtual void reset();
        virtual bool done();
    private:
        virtual void animate(PicoLed::PicoLedController& controller, uint32_t delta_time);
        PicoLed::Color color, base_color;
        uint size, led_count;
        float pos, speed;
    };
} // namespace sparkie::visuals
