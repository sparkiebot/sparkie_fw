#pragma once

#include "../uros/URosComponent.hpp"
#include <ahtxx/ahtxx.hpp>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/relative_humidity.h>

namespace sparkie
{
    typedef struct _aht_data
    {
        float temp;
        float hum;
    } AhtData;

    /**
     * Component for publishing temperature and humidity data.
     */
    class AHTComponent : public URosComponent
    {
    public:
        AHTComponent();

    protected:
        virtual void rosInit();

    private:
        virtual void init();
        virtual void loop(TickType_t *xLastWakeTime);

        sensor_msgs__msg__Temperature temp_msg;
        sensor_msgs__msg__RelativeHumidity hum_msg;
        LIB_AHTXX sensor;
        AhtData aht_data;
    };
} // namespace sparkie
