#include "ServoComponent.hpp"

#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>

#include "../../sparkie_defs.hpp"

#include <iostream>

using namespace sparkie;

// Define message type for servo control
const MessageType MSG_SERVO_CONTROL = {"servo", 0};

/**
 * Adapted function from Arduino original map()
*/
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ServoComponent::ServoComponent(const std::string& name, uint pin, uint freq) 
    : MessageComponent("joint_", CORE0, SERVO_PRIORITY, UROS_SERVO_RATE), pwm(pin)
{
    Component::name.append(name);
    this->pin = pin;
    this->freq = freq;
    this->period = (1e6 / freq); // us
}

void ServoComponent::setMicros(float micros)
{
    if(micros < SERVO_MIN_MICROS || micros > SERVO_MAX_MICROS)
        return;
    this->pwm.setDuty((micros / this->period) * 65535);
}

void ServoComponent::setDegrees(float degrees)
{
    this->setMicros(
        map(degrees, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_MICROS, SERVO_MAX_MICROS)
    );
}

uint8_t ServoComponent::getHandlesNum()
{
    return 1;
}

void ServoComponent::msgInit()
{
    this->addSubscription(
        "/" + this->getName().substr(6),
        &MSG_SERVO_CONTROL,
        ServoComponent::onMessage
    );
}

void ServoComponent::init()
{
    this->pwm.setFrequency(this->freq);   
    this->setDegrees(SERVO_ZERO_POS); 
}

void ServoComponent::onMessage(MessageComponent* component, const void* msg_in)
{
    const ServoData* msg = (const ServoData*) msg_in;

    if (msg->angle < SERVO_SOFT_MIN_ANGLE || msg->angle > SERVO_SOFT_MAX_ANGLE)
        return;
    
    auto servo = (ServoComponent*) component;
    servo->setDegrees(SERVO_ZERO_POS + msg->angle);
}

void ServoComponent::loop(TickType_t* xLastWakeTime)
{    
    // No regular updates needed for servo component
}

void ServoComponent::safeStop()
{
    this->setDegrees(SERVO_ZERO_POS); 
}
