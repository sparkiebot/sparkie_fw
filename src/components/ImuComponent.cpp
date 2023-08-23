#include "ImuComponent.hpp"
#include "LoggerComponent.hpp"
#include "../sparkie_defs.hpp"
#include "../config.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <chrono>
#include <iostream>
#include <rmw_microros/rmw_microros.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/i2c.h>

using namespace sparkie;
    
void sleep_fn(uint time_ms)
{
    vTaskDelay(time_ms / portTICK_PERIOD_MS);
}

ImuComponent::ImuComponent() : URosComponent("imu", CORE1, IMU_PRIORITY, UROS_IMU_RATE)
{

}

void ImuComponent::init()
{   
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    icm20689_set_sleep_function(&this->imu, sleep_fn);
    icm20689_init(&this->imu, IMU_ADDR, IMU_SAMPLES_NUM);
}

void ImuComponent::rosInit()
{
    this->addPublisher(
        "imu",
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu)
    );

    this->ros_msg.header.frame_id = 
        micro_ros_string_utilities_init(UROS_IMU_FRAME);

    for (uint i = 0; i < 9; i++)
    {
        this->ros_msg.angular_velocity_covariance[i] = 0.0;
        this->ros_msg.linear_acceleration_covariance[i] = 0.0;
        this->ros_msg.orientation_covariance[i] = 0.0;
    }

    this->ros_msg.orientation.x = 0.0;
    this->ros_msg.orientation.y = 0.0;
    this->ros_msg.orientation.z = 0.0;
    this->ros_msg.orientation.w = 1.0;
    
}

void ImuComponent::loop(TickType_t* xLastWakeTime)
{
    icm20689_read_gyroacc(&this->imu, NULL, NULL);
    
    this->ros_msg.header.stamp.nanosec = (uint32_t) rmw_uros_epoch_nanos();
    this->ros_msg.header.stamp.sec = (int32_t) (rmw_uros_epoch_millis() / 1000);

    this->ros_msg.angular_velocity.x = this->imu.gyroData[0];
    this->ros_msg.angular_velocity.y = this->imu.gyroData[1];
    this->ros_msg.angular_velocity.z = this->imu.gyroData[2];

    this->ros_msg.linear_acceleration.x = this->imu.accData[0];
    this->ros_msg.linear_acceleration.y = this->imu.accData[1];
    this->ros_msg.linear_acceleration.z = this->imu.accData[2];
    
    this->sendMessage(0, &this->ros_msg);
}
