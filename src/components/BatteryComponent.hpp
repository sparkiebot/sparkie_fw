#pragma once

#include "URosComponent.hpp"
#include <sensor_msgs/msg/battery_state.h>
#include "../config.hpp"
#include <array>

namespace sparkie
{
    enum class BatteryStatus
    {
        Unknown = 0,
        Charging = 1,
        Discharging = 2,
        NotCharging = 3,
        Full = 4,
        AbnormalCharging = 5
    };

    typedef struct _reads
    {
        std::array<float, BATT_READS_COUNT> data;
        uint8_t current_index;
    } BatteryReads;

    class BatteryComponent : public URosComponent
    {
    public:
        BatteryComponent();
    protected:
        virtual void rosInit();
    private:
        static void onChargingCheck(TimerHandle_t timer);

        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        BatteryReads reads;

        bool low_power;
        BatteryStatus curr_status;
        static bool previous_state;
        float curr_read;
        sensor_msgs__msg__BatteryState battery_msg;
        static QueueHandle_t charge_state_queue;
        TimerHandle_t charging_check;
    };    
} // namespace sparkie
