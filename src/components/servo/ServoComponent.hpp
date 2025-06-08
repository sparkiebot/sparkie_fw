#pragma once

#include "../messaging/MessageComponent.hpp"
#include "../../config.hpp"

#include <picopwm.h>

#include <string>

namespace sparkie
{
    /**
     * Simple message structure for servo control
     */
    typedef struct _servo_data
    {
        float angle;  // Servo angle in degrees
    } ServoData;

    class ServoComponent : public MessageComponent
    {
    public:
        ServoComponent(const std::string& topic_name, uint pin, uint freq);
        virtual uint8_t getHandlesNum();
    protected:
        void msgInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);
        virtual void safeStop();

        static void onMessage(MessageComponent* component, const void* msg_in);

        void setMicros(float micros);
        void setDegrees(float degrees);

        float curr_angle;
        uint pin;
        PicoPwm pwm;
        uint period, freq;
        ServoData servo_data;
    };    
} // namespace sparkie
