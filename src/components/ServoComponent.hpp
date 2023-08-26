#pragma once

#include "URosComponent.hpp"
#include "../config.hpp"

#include <picopwm.h>

#include <string>

#include <std_msgs/msg/float32.h>

namespace sparkie
{
    class ServoComponent : public URosComponent
    {
    public:
        ServoComponent(const std::string& topic_name, uint pin, uint freq);
        virtual uint8_t getHandlesNum();
    protected:
        void rosInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);
        virtual void safeStop();

        static void onMessage(URosComponent* component, const void* msg_in);

        void setMicros(float micros);
        void setDegrees(float degrees);

        float curr_angle;
        uint pin;
        PicoPwm pwm;
        uint period, freq;
        std_msgs__msg__Float32 angle_msg;
    };    
} // namespace sparkie
