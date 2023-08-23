#pragma once

#include <stdio.h>
#include <stdbool.h>

double clamp(double value, double min, double max);

namespace sparkie
{

    class Pid
    {
    public:
        Pid(double kp, double ki, double kd, double min_value, double max_value);
        void reset();
        double compute(double goal, double current_measure, double delta_time);
    private:
        double kp, ki, kd;

        double proportional;
        double integral;
        double derivate;

        double error;
        double last_error;

        double max_value, min_value;
    };
    
}