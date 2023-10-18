#pragma once

#include "../uros/URosComponent.hpp"
#include <sensor_msgs/msg/battery_state.h>
#include "../../config.hpp"
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

    /**
     * Structure used to send data between charge_check timer and main component's task
    */
    typedef struct _reads
    {
        std::array<float, BATT_READS_COUNT> data;
        uint8_t current_index;
    } BatteryReads;

    /**
     * This component is responsible of calculating current SoC (State of Charge) <br>
     * and checking if battery is being charged <br>
     * Currently calculation is being done by simply measuring current voltage and <br> 
     * calculating the percetage using a particular function.
    */
    class BatteryComponent : public URosComponent
    {
    public:
        BatteryComponent();
    protected:
        virtual void rosInit();
    private:

        /**
         * Timer that checks every half second whether battery is being charged or not.
        */
        static void onChargingCheck(TimerHandle_t timer);

        virtual void init();

        /**
         * This component measures battery voltage, checks for any new updates from ChargingCheck timer,
         * and then it publishes the obtained data into a BatteryMsg
         * Voltage measurements are filtered using a moving average algorithm.
        */
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
