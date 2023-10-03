#pragma once

#include "URosComponent.hpp"
#include <icm20689pico/icm20689pico.h>
#include <sensor_msgs/msg/imu.h>

namespace sparkie
{
    class ImuComponent : public URosComponent
    {
    public:
        ImuComponent();
    protected:
        virtual void rosInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        icm20689_t imu;
        sensor_msgs__msg__Imu ros_msg;
    };
} // namespace sparkie
