#include "Pid.hpp"
#include <math.h>

using namespace sparkie;

double clamp(double value, double min, double max)
{
    return std::min(std::max(value, min), max);
}

Pid::Pid(double kp, double ki, double kd, double min_value, double max_value)
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
}

void Pid::reset()
{
    this->proportional = 0.0;
    this->integral = 0.0;
    this->derivate = 0.0;
    this->last_error = 0.0;
}

double Pid::compute(double goal, double current_measure, double delta_time)
{
    this->error = goal - current_measure;

    this->proportional = this->kp * this->error;
    
    this->integral += this->ki * this->error * delta_time;

    this->derivate = this->kd * ((this->error - this->last_error) / delta_time);
    
    this->last_error = this->error;
    
    return clamp(
        this->proportional + this->integral + this->derivate, 
        this->min_value, this->max_value
    );

}