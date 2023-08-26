#pragma once

#include "URosComponent.hpp"
#include <std_msgs/msg/u_int8.h>

namespace sparkie
{
    class SystemComponent : public URosComponent
    {
    public:
        SystemComponent(); 
        virtual uint8_t getHandlesNum(); 
    protected:
        virtual void rosInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);
        static void onMessage(URosComponent* component, const void* msg_in);
        std_msgs__msg__UInt8 data_msg;
    };    
} // namespace sparkie
