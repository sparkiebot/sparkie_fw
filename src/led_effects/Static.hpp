#pragma once

#include "StripEffect.hpp"

namespace sparkie::visuals
{
    /**
     * @brief Displays the specified color.
    */
    class Static : public StripEffect
    {
    public:
        Static(PicoLed::Color color);
        virtual void reset();
        virtual bool done();
    private:
        virtual void animate(PicoLed::PicoLedController& controller, uint32_t delta_time);
        PicoLed::Color color;
    };
} // namespace sparkie::visuals
