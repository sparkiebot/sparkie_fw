#pragma once

#include "../messaging/MessageComponent.hpp"

namespace sparkie
{
    /**
     * Simple message structure for system control
     */
    typedef struct _system_data
    {
        uint8_t command;  // 0: normal reboot, 1: programming mode
    } SystemData;

    /**
     * @brief Component used for rebooting board either normally or in programming mode.
     * 0 means normal mode.
     * 1 means programming mode.
    */
    class SystemComponent : public MessageComponent
    {
    public:
        SystemComponent(); 
        virtual uint8_t getHandlesNum(); 
        static void onActionBtn(uint gpio, uint32_t events);
    protected:
        virtual void msgInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);
        static void onMessage(MessageComponent* component, const void* msg_in);
        SystemData system_data;
    };    
} // namespace sparkie
