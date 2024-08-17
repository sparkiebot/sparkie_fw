#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

double clamp(double value, double min, double max);

namespace sparkie
{

    /**
     * @brief Simple implementation of a PID controller.
     * 
     * Value is also clamped between a minimum and a maximum value.
    */
    class Pid
    {
    public:
        Pid(double kp, double ki, double kd, uint16_t deadband_offset, uint16_t min_value, uint16_t max_value);
        void reset();
        uint16_t compute(double goal, double current_measure);
    private:
        double kp, ki, kd;

        double proportional;
        double integral;
        double derivate;

        double error;
        double last_error;

        uint16_t max_value, min_value;
        uint16_t deadband_offset;
    };
    
}