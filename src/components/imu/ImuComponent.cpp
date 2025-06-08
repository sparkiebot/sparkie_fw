#include "ImuComponent.hpp"
#include "../logger/LoggerComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../../config.hpp"
#include <chrono>
#include <iostream>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/i2c.h>

using namespace sparkie;

// Define message types for IMU and magnetic field data
const MessageType MSG_IMU_DATA = {"imu", 0};
const MessageType MSG_MAG_DATA = {"mag", 1};

void sleep_fn(uint32_t time_ms)
{
    vTaskDelay(time_ms / portTICK_PERIOD_MS);
}

ImuComponent::ImuComponent() : MessageComponent("imu", CORE1, IMU_PRIORITY, UROS_IMU_RATE)
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
    
    // Initialize IMU data structure
    this->imu_data.orientation.x = 0.0;
    this->imu_data.orientation.y = 0.0;
    this->imu_data.orientation.z = 0.0;
    this->imu_data.orientation.w = 1.0;
    
    this->imu_data.angular_velocity.x = 0.0;
    this->imu_data.angular_velocity.y = 0.0;
    this->imu_data.angular_velocity.z = 0.0;
    
    this->imu_data.linear_acceleration.x = 0.0;
    this->imu_data.linear_acceleration.y = 0.0;
    this->imu_data.linear_acceleration.z = 0.0;
    
    // Initialize magnetic field data structure
    this->mag_data.magnetic_field.x = 0.0;
    this->mag_data.magnetic_field.y = 0.0;
    this->mag_data.magnetic_field.z = 0.0;
}

void ImuComponent::msgInit()
{
    // Add publishers for IMU and magnetic field data
    this->addPublisher("/imu", &MSG_IMU_DATA);
    this->addPublisher("/mag", &MSG_MAG_DATA, false);
}

void ImuComponent::loop(TickType_t* xLastWakeTime)
{
    this->imu.read();

    // Get current timestamp in milliseconds
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    
    // Update timestamps
    this->imu_data.timestamp_ms = current_time_ms;
    this->mag_data.timestamp_ms = current_time_ms;

    // Apply low-pass filter (alpha = 0.5)
    const float alpha = 0.5;

    // Update angular velocity data with low-pass filter
    this->imu_data.angular_velocity.x = this->imu_data.angular_velocity.x * alpha + this->imu.get_gyro().x * (1.0 - alpha);
    this->imu_data.angular_velocity.y = this->imu_data.angular_velocity.y * alpha + this->imu.get_gyro().y * (1.0 - alpha);
    this->imu_data.angular_velocity.z = this->imu_data.angular_velocity.z * alpha + this->imu.get_gyro().z * (1.0 - alpha);

    // Update linear acceleration data with low-pass filter
    this->imu_data.linear_acceleration.x = this->imu_data.linear_acceleration.x * alpha + this->imu.get_accel().x * (1.0 - alpha);
    this->imu_data.linear_acceleration.y = this->imu_data.linear_acceleration.y * alpha + this->imu.get_accel().y * (1.0 - alpha);
    this->imu_data.linear_acceleration.z = this->imu_data.linear_acceleration.z * alpha + this->imu.get_accel().z * (1.0 - alpha);
    
    // Set orientation (currently not calculated, using default values)
    this->imu_data.orientation.x = 0.0;
    this->imu_data.orientation.y = 0.0;
    this->imu_data.orientation.z = 0.0;
    this->imu_data.orientation.w = 1.0;

    // Send IMU data message
    this->sendMessage(0, &this->imu_data, sizeof(this->imu_data));

    // Update magnetic field data with low-pass filter
    this->mag_data.magnetic_field.x = this->mag_data.magnetic_field.x * alpha + this->imu.get_mag().x * (1.0 - alpha);
    this->mag_data.magnetic_field.y = this->mag_data.magnetic_field.y * alpha + this->imu.get_mag().y * (1.0 - alpha);
    this->mag_data.magnetic_field.z = this->mag_data.magnetic_field.z * alpha + this->imu.get_mag().z * (1.0 - alpha);

    // Send magnetic field data message
    this->sendMessage(1, &this->mag_data, sizeof(this->mag_data));
}
