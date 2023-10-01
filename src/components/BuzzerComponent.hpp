#pragma once

#include <picopwm.h>
namespace sparkie
{
    enum BuzzerAction
    {
        PLAY_STARTUP = 0,
        CONNECTED = 1,
        ERROR = 2
    };

    /**
     * Handy class for managing buzzer and make it play a set of predefined sounds (BuzzerAction).
    */
    class BuzzerComponent
    {
    public:
        static void init();
        static void play(const BuzzerAction action);
    private:
        static void tone(const uint freq);
        static void tone(const uint freq, const uint duration);
        static PicoPwm* pwm;
    }; 
   
} // namespace sparkie
