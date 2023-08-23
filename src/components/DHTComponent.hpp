#pragma once

#include "URosComponent.hpp"
#include <dht.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>


namespace sparkie
{
    class DHTComponent : public URosComponent
    {
    public:
        DHTComponent();
    protected:
        virtual void rosInit();
        
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        sensor_msgs__msg__Temperature temp_msg;
        sensor_msgs__msg__RelativeHumidity hum_msg;
        dht_t sensor;
    };    
} // namespace sparkie
