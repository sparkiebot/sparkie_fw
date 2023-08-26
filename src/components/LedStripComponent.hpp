#pragma once

#include "URosComponent.hpp"
#include "../config.hpp"
#include "../sparkie_defs.hpp"
#include <array>
#include <map>
#include <std_msgs/msg/u_int8.h>
#include <PicoLed.hpp>
#include "../led_effects/StripEffect.hpp"


namespace sparkie
{

    enum class LedStripMode
    {
        Off = -1,

        Charging,
        ChargeComplete,
        LowBattery,
        AbnormalCharging,

        Idle = 10,
        Good,
        Bad,
        StartingUp,
        Calculating,
        Connecting,

    };

    class LedStripComponent : public URosComponent
    {
    public:
        LedStripComponent();
        static LedStripComponent* getInstance();
        virtual uint8_t getHandlesNum();
        void setMode(const LedStripMode mode, uint8_t index = LED_NOTIFY_LAYER);
        const LedStripMode getMode(uint8_t index);
        
    protected:
        virtual void rosInit();
    private:

        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);
        virtual void safeStop();

        static void onMessage(URosComponent* component, const void* msg_in);
        
        uint8_t sm_index;

        std_msgs__msg__UInt8 data_msg;
        PicoLed::PicoLedController controller;
        
        std::array<std::pair<visuals::StripEffect*, LedStripMode>, 2> curr_effects;
        std::map<LedStripMode, visuals::StripEffect*> modes;

        static LedStripComponent* instance;
    };    
} // namespace sparkie
