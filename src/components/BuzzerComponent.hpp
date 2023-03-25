#pragma once

#include "Component.hpp"
#include <queue.h>

namespace hubbie
{
    enum BuzzerAction
    {
        PLAY_STARTUP = 0,
        CONNECTED = 1,
    };

    class BuzzerComponent
    {
    public:
        static void init();
        static void play(const BuzzerAction action);
    private:
        static void tone(const uint32_t freq);
        static void tone(const uint32_t freq, const uint32_t duration);
    }; 
   
} // namespace hubbie
