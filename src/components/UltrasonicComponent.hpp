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

    class UltrasonicComponent : public URosComponent
    {
    public:
        UltrasonicComponent();
    protected:
        virtual void rosInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        void initTriggerPio();
        void initEchoPio(PIO pio, uint sm, uint echo_pin);
        void initSensor(UltrasonicSensor* sensor, int index, const std::string& name);
        void readData(TickType_t* lastWakeTime);
        sensor_msgs__msg__Range ros_msg[US_NUM];
        UltrasonicSensor sensors[US_NUM];
    };    
} // namespace sparkie
