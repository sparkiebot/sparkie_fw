#pragma once

#include <PicoLed.hpp>

namespace sparkie::visuals::colors
{
    const PicoLed::Color BLACK = PicoLed::RGB(0, 0, 0);
    const PicoLed::Color WHITE = PicoLed::RGB(221, 230, 237);
    const PicoLed::Color GREEN = PicoLed::RGB(10, 150, 10);
    const PicoLed::Color ORANGE = PicoLed::RGB(204, 105, 0);
    const PicoLed::Color RED = PicoLed::RGB(219, 37, 24);
    const PicoLed::Color VIOLET = PicoLed::RGB(227, 0, 204);
    const PicoLed::Color BLUE = PicoLed::RGB(0, 153, 255);

    const PicoLed::Color IDLE = BLACK;
}