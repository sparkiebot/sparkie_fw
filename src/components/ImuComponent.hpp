#pragma once

#include "URosComponent.hpp"

#define ICM20689_SLEEP(ms) vTaskDelay(ms / portTICK_PERIOD_MS)
#include <icm20689pico/icm20689pico.h>

#include <sensor_msgs/msg/imu.h>

namespace hubbie
{
    class ImuComponent : public URosComponent
    {
    public:
        ImuComponent();
        virtual ~ImuComponent();
        virtual configSTACK_DEPTH_TYPE getMaxStackSize();
        virtual void rosInit();
        virtual void run();
    private:
        icm20689_t imu;
        sensor_msgs__msg__Imu ros_msg;
    };    
} // namespace hubbie
