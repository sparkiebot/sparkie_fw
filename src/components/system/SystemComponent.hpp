#pragma once

#include "../uros/URosComponent.hpp"
#include <std_msgs/msg/u_int8.h>

namespace sparkie
{
    /**
     * @brief Component used for rebooting board either normally or in programming mode.
     * 0 means normal mode.
     * 1 means programming mode.
    */
    class SystemComponent : public URosComponent
    {
    public:
        SystemComponent(); 
        virtual uint8_t getHandlesNum(); 
        static void onActionBtn(uint gpio, uint32_t events);
    protected:
        virtual void rosInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);
        static void onMessage(URosComponent* component, const void* msg_in);
        std_msgs__msg__UInt8 data_msg;
    };    
} // namespace sparkie
