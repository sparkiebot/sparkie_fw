#pragma once

#include "../messaging/MessageComponent.hpp"
#include <hardware/pio.h>
#include "../../config.hpp"

namespace sparkie
{
    /**
     * Message structure for range data
     */
    typedef struct _range_data
    {
        float range;              // Range in meters
        float field_of_view;      // Field of view in radians
        float min_range;          // Minimum range in meters
        float max_range;          // Maximum range in meters
        uint8_t radiation_type;   // Type of radiation (0: ultrasonic)
        uint32_t timestamp_ms;    // Timestamp in milliseconds
    } RangeData;

    typedef struct _UltrasonicSensor
    {
        std::string frame;
        float data;
        uint8_t sm_index;
    } UltrasonicSensor;

    /**
     * @brief Component that publishes all ultrasonic sensors data.
     * 
     * It will initialize one PIO that will send the trigger signal on every sensor. <br>
     * and three PIOs for calculating the actual distances.
    */
    class UltrasonicComponent : public MessageComponent
    {
    public:
        UltrasonicComponent();
    protected:
        virtual void msgInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        /**
         * These two methods will actually setup PIOs.
        */
        void initTriggerPio();
        void initEchoPio(PIO pio, uint sm, uint echo_pin);
        
        /**
         * @brief Inits messaging structures for sensor representation.
        */
        void initSensor(UltrasonicSensor* sensor, int index, const std::string& name);
        
        /**
         * @brief Reads data from all sensors using PIO instructions.
        */
        void readData(TickType_t* lastWakeTime);
        
        RangeData range_data[US_NUM];
        UltrasonicSensor sensors[US_NUM];
    };    
} // namespace sparkie
