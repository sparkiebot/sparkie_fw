#include "ImuComponent.hpp"
#include "../hube_defs.hpp"
#include "../config.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <chrono>
#include <iostream>
#include <rmw_microros/rmw_microros.h>

#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/i2c.h>

#define ICM20689_SLEEP(ms) vTaskDelay(ms / portTICK_PERIOD_MS)
#define ROS_CHECK(ret) if(ret != RCL_RET_OK) while(true) {std::cout << "error code: " << ret << "\r\n"; vTaskDelay(500 / portTICK_PERIOD_MS); }

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

using namespace hubbie;
ImuComponent::ImuComponent() : URosComponent("imu", CORE1, IMU_PRIORITY)
{
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

}

ImuComponent::~ImuComponent()
{
    URosComponent::~URosComponent();
}

configSTACK_DEPTH_TYPE ImuComponent::getMaxStackSize()
{
    return 1000;
}

void ImuComponent::rosInit()
{
    rcl_publisher_t* imu_pub;
    imu_pub = (rcl_publisher_t*) malloc(sizeof(rcl_publisher_t));
    this->publishers.push_back(imu_pub);

    auto default_opt = rcl_publisher_get_default_options();

    ROS_CHECK(rclc_publisher_init_best_effort(
        imu_pub,
        this->parent_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "board/imu"
    ));

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
    this->ros_msg.orientation.w = 0.0;
}

void ImuComponent::run()
{
    icm20689_init(&this->imu, IMU_SAMPLES_NUM);
    
    auto xLastWakeTime = xTaskGetTickCount();

    while(this->running)
    {
        this->ros_msg.header.stamp.nanosec = (uint32_t) rmw_uros_epoch_nanos();
        this->ros_msg.header.stamp.sec = (int32_t) (rmw_uros_epoch_millis() / 1000);
        
        icm20689_read_gyroacc(&this->imu, NULL, NULL);
        
        this->ros_msg.angular_velocity.x = this->imu.gyroData[0];
        this->ros_msg.angular_velocity.y = this->imu.gyroData[1];
        this->ros_msg.angular_velocity.z = this->imu.gyroData[2];

        this->ros_msg.linear_acceleration.x = this->imu.accData[0];
        this->ros_msg.linear_acceleration.y = this->imu.accData[1];
        this->ros_msg.linear_acceleration.z = this->imu.accData[2];
        
        rcl_publish(this->publishers.at(0), &this->ros_msg, NULL);
        
        xTaskDelayUntil(&xLastWakeTime, HZ_TO_MS(UROS_IMU_RATE));
    }
}
