#pragma once

#include <PicoLed.hpp>

namespace sparkie::visuals
{
    /**
     * @brief Base class representing a generic led strip effect.
    */
    class StripEffect
    {
    public:
        StripEffect();
        
        /**
         * @brief Updates new effect state.
         * 
         * Most effects will need to be updated frequently to work properly.
         * This is called by LedStripComponent.
        */
        void update(PicoLed::PicoLedController& controller);
        virtual void reset() = 0;
        virtual bool done() = 0;
    protected:

        /**
         * @brief Defines the actual led strip effect behaviour
         * 
         * This is called every delta_time by StripEffect::update.
         * If effect is not static, a good practice is to turn off every led in layer before <br>
         * actually "rendering" the new frame.
        */
        virtual void animate(PicoLed::PicoLedController& controller, uint32_t delta_time) = 0;
        uint32_t last_update;
    };    
} // namespace sparkie::visuals
