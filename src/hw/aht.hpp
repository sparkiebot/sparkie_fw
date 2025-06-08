
#pragma once
#include <cstdint>

namespace sparkie
{
    /**
     * Component for publishing temperature and humidity data.
     */
    class AHT
    {
    public:
        AHT(uint8_t addr);
        int init();
        int read_data(float *temp, float *hum);
    private:    
        uint8_t addr;

    };
} // namespace sparkie
