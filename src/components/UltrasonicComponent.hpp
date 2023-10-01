#pragma once

#include "URosComponent.hpp"
#include "hardware/pio.h"
#include "../config.hpp"
#include <sensor_msgs/msg/range.h>

namespace sparkie
{
    typedef struct _UltrasonicSensor
    {
        rosidl_runtime_c__String frame;
        float data;
        uint8_t sm_index;
    } UltrasonicSensor;

    /**
     * @brief Component that publishes all ultrasonic sensors data.
     * 
     * It will initialize one PIO that will send the trigger signal on every sensor. <br>
     * and three PIOs for calculating the actual distances.
    */
    class UltrasonicComponent : public URosComponent
    {
    public:
        UltrasonicComponent();
    protected:
        virtual void rosInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        /**
         * These two methods will actually setup PIOs.
        */

        void initTriggerPio();
        void initEchoPio(PIO pio, uint sm, uint echo_pin);
        
        /**
         * @brief Inits ros structures for sensor representation.
        */
        void initSensor(UltrasonicSensor* sensor, int index, const std::string& name);
        
        /**
         * @brief Reads data from all sensors using PIO instructions.
        */
        void readData(TickType_t* lastWakeTime);
        sensor_msgs__msg__Range ros_msg[US_NUM];
        UltrasonicSensor sensors[US_NUM];
    };    
} // namespace sparkie
