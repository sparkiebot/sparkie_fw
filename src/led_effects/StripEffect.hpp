#pragma once

#include <PicoLed.hpp>

namespace sparkie::visuals
{
    class StripEffect
    {
    public:
        StripEffect();
        void update(PicoLed::PicoLedController& controller);
        virtual void reset() = 0;
        virtual bool done() = 0;
    protected:
        virtual void animate(PicoLed::PicoLedController& controller, uint32_t delta_time) = 0;
        uint32_t last_update;
    };    
} // namespace sparkie::visuals
