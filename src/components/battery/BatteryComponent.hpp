#pragma once

#include "../messaging/MessageComponent.hpp"
#include "../../config.hpp"
#include <array>
#include <timers.h>

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
     * Message type structure for battery data
     */
    typedef struct _battery_data
    {
        float voltage;         // Battery voltage
        float percentage;      // Battery percentage (0.0 - 1.0)
        uint8_t status;        // Battery status (using BatteryStatus enum)
        float charge;          // Current charge level
        float capacity;        // Total capacity
    } BatteryData;

    /**
     * @brief this component is responsible of calculating current SoC (State of Charge) <br>
     * and checking if battery is being charged <br>
     * Currently calculation is being done by simply measuring current voltage and <br> 
     * calculating the percentage using a particular function.
    */
    class BatteryComponent : public MessageComponent
    {
    public:
        BatteryComponent();
    protected:
        virtual void msgInit();
    private:

        /**
         * Timer that checks every half second whether battery is being charged or not.
        */
        static void onChargingCheck(TimerHandle_t timer);

        virtual void init();

        /**
         * Measures battery voltage, checks for any new updates from ChargingCheck timer,
         * and then it publishes the obtained data into a BatteryMsg
         * Voltage measurements are filtered using a moving average algorithm.
        */
        virtual void loop(TickType_t* xLastWakeTime);

        BatteryReads reads;

        bool low_power;
        BatteryStatus curr_status;
        static bool previous_state;
        float curr_read;
        BatteryData battery_data;
        static QueueHandle_t charge_state_queue;
        TimerHandle_t charging_check;
    };    
} // namespace sparkie
