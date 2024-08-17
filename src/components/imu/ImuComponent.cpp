#include "ImuComponent.hpp"
#include "../logger/LoggerComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../../config.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <chrono>
#include <iostream>
#include <rmw_microros/rmw_microros.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/i2c.h>

using namespace sparkie;
    
void sleep_fn(uint32_t time_ms)
{
    vTaskDelay(time_ms / portTICK_PERIOD_MS);
}

ImuComponent::ImuComponent() : URosComponent("imu", CORE1, IMU_PRIORITY, UROS_IMU_RATE)
{
    this->imu.set_sleep_fn(sleep_fn);
}

void ImuComponent::init()
{   
    vTaskDelay(50 / portTICK_PERIOD_MS);
    this->imu.init();
    this->imu.set_adxl345_range(adxl345_range_t::RANGE_8_G);
    // Calibrate the IMU with 20 samples
    this->imu.calibrate();
}

void ImuComponent::rosInit()
{
    this->addPublisher(
        "imu",
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        false
    );

    this->imu_msg.header.frame_id = 
        micro_ros_string_utilities_init(UROS_IMU_FRAME);

    for (uint i = 0; i < 9; i++)
    {
        this->imu_msg.angular_velocity_covariance[i] = 0.0;
        this->imu_msg.linear_acceleration_covariance[i] = 0.0;
        this->imu_msg.orientation_covariance[i] = 0.0;
    }

    this->imu_msg.orientation.x = 0.0;
    this->imu_msg.orientation.y = 0.0;
    this->imu_msg.orientation.z = 0.0;
    this->imu_msg.orientation.w = 1.0;

    this->addPublisher(
        "mag",
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        false
    );

    this->mag_msg.header.frame_id = 
        micro_ros_string_utilities_init(UROS_IMU_FRAME);

    for (uint i = 0; i < 9; i++)
    {
        this->mag_msg.magnetic_field_covariance[i] = 0.0;
    }

    this->mag_msg.magnetic_field.x = 0.0;
    this->mag_msg.magnetic_field.y = 0.0;
    this->mag_msg.magnetic_field.z = 0.0;
    
}

void ImuComponent::loop(TickType_t* xLastWakeTime)
{
    this->imu.read();

    this->imu_msg.header.stamp.nanosec = (uint32_t) rmw_uros_epoch_nanos();
    this->imu_msg.header.stamp.sec = (int32_t) (rmw_uros_epoch_millis() / 1000);

    this->imu_msg.angular_velocity.x = this->imu.get_gyro().x;
    this->imu_msg.angular_velocity.y = this->imu.get_gyro().y;
    this->imu_msg.angular_velocity.z = this->imu.get_gyro().z;

    this->imu_msg.linear_acceleration.x = this->imu.get_accel().x;
    this->imu_msg.linear_acceleration.y = this->imu.get_accel().y;
    this->imu_msg.linear_acceleration.z = this->imu.get_accel().z;
    
    this->imu_msg.orientation.x = 0.0;
    this->imu_msg.orientation.y = 0.0;
    this->imu_msg.orientation.z = 0.0;

    this->sendMessage(0, &this->imu_msg);


    this->mag_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();
    this->mag_msg.header.stamp.sec = (int32_t)(rmw_uros_epoch_millis() / 1000);

    this->mag_msg.magnetic_field.x = this->imu.get_mag().x;
    this->mag_msg.magnetic_field.y = this->imu.get_mag().y;
    this->mag_msg.magnetic_field.z = this->imu.get_mag().z;

    this->sendMessage(1, &this->mag_msg);

}
