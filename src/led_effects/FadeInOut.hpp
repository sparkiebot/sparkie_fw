#pragma once

#include "StripEffect.hpp"

namespace sparkie::visuals
{
    class FadeInOut : public StripEffect
    {
    public:

        FadeInOut(PicoLed::Color target_color, float speed);
        FadeInOut(PicoLed::Color target_color, float speed, bool loop);
        void setSpeed(float speed);
        virtual void reset();
        virtual bool done();
    private:
        virtual void animate(PicoLed::PicoLedController& controller, uint32_t delta_time);
        PicoLed::Color first_color, second_color;
        float fade_time, curr_ticks;
        uint8_t cycles_count;
        bool reverse, loop;
    };
} // namespace sparkie::visuals
