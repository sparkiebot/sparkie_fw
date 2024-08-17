#pragma once

#include "../uros/URosComponent.hpp"
#include <gy85/gy85.hpp>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

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

        gy85 imu;
        sensor_msgs__msg__Imu imu_msg;
        sensor_msgs__msg__MagneticField mag_msg;
    };
} // namespace sparkie
