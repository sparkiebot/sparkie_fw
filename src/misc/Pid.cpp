#include "Pid.hpp"
#include <math.h>

using namespace sparkie;

double clamp(double value, double min, double max)
{
    return std::min(std::max(value, min), max);
}

Pid::Pid(double kp, double ki, double kd, uint16_t deadband_offset, uint16_t min_value, uint16_t max_value)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;

    this->proportional = 0.0;
    this->integral = 0.0;
    this->derivate = 0.0;

    this->last_error = 0.0;
    this->error = 0.0;

    this->min_value = min_value;
    this->max_value = max_value;
    this->deadband_offset = deadband_offset;
}

void Pid::reset()
{
    this->proportional = 0.0;
    this->integral = 0.0;
    this->derivate = 0.0;
    this->last_error = 0.0;
}

uint16_t Pid::compute(double goal, double current_measure)
{
    this->error = goal - current_measure;

    this->proportional = this->kp * this->error;
    
    this->integral += this->ki * this->error;

    this->derivate = this->kd * ((this->error - this->last_error));
    
    this->last_error = this->error;

    auto pid_value = this->proportional + this->integral + this->derivate;

    pid_value += this->deadband_offset;

    return clamp(pid_value, this->min_value, this->max_value);
}