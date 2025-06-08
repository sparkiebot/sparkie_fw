#include "OdometryComponent.hpp"
#include "../../config.hpp"
#include "../../sparkie_defs.hpp"
#include "../motors/MotorsComponent.hpp"
#include <math.h>
#include <pico/stdlib.h>

using namespace sparkie;

// Define message type for odometry
const MessageType MSG_ODOMETRY = {"wheels/odom", 0};

OdometryComponent::OdometryComponent()
    : MessageComponent("odometry", CORE1, ODOM_PRIORITY, UROS_ODOM_RATE)
{
}

uint8_t OdometryComponent::getHandlesNum()
{
    return 0;
}

void OdometryComponent::init()
{
    this->x = 0;
    this->y = 0;
    this->theta = 0;
    this->linear_velocity = 0;
    this->angular_velocity = 0;

    this->raw_motor_data[0] = 0;
    this->raw_motor_data[1] = 0;
    
    // Initialize odometry data structure
    this->odom_data.position.x = 0;
    this->odom_data.position.y = 0;
    this->odom_data.position.z = 0;
    
    this->odom_data.orientation.x = 0;
    this->odom_data.orientation.y = 0;
    this->odom_data.orientation.z = 0;
    this->odom_data.orientation.w = 1;
    
    this->odom_data.linear_velocity.x = 0;
    this->odom_data.linear_velocity.y = 0;
    this->odom_data.linear_velocity.z = 0;
    
    this->odom_data.angular_velocity.x = 0;
    this->odom_data.angular_velocity.y = 0;
    this->odom_data.angular_velocity.z = 0;
    
    this->odom_data.timestamp_ms = 0;
}

void OdometryComponent::msgInit()
{
    // Initialize odometry publisher
    this->addPublisher("/wheels/odom", &MSG_ODOMETRY, false);
}

void OdometryComponent::loop(TickType_t *xLastWakeTime)
{
    // Get motors velocities
    if (MotorsComponent::getMotorsQueue() != NULL && 
        xQueueReceive(MotorsComponent::getMotorsQueue(), &this->raw_motor_data, 0) == pdFALSE)
    {
        return;
    }

    auto left_vel = this->raw_motor_data[0] * RPM_TO_RADS;
    auto right_vel = this->raw_motor_data[1] * RPM_TO_RADS;

    this->linear_velocity = ((- left_vel + right_vel) / 2.0) * MOTORS_WHEEL_RADIUS;
    this->angular_velocity = ((left_vel + right_vel) / (MOTORS_WHEEL_SEPARATION)) * MOTORS_WHEEL_RADIUS;

    // Calculate odometry
    this->theta += this->angular_velocity * (1 / UROS_ODOM_RATE);
    this->x += this->linear_velocity * cos(this->theta) * (1 / UROS_ODOM_RATE);
    this->y += this->linear_velocity * sin(this->theta) * (1 / UROS_ODOM_RATE);

    // Get current timestamp
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    
    // Update odometry data
    this->odom_data.timestamp_ms = current_time_ms;
    
    this->odom_data.linear_velocity.x = this->linear_velocity;
    this->odom_data.angular_velocity.z = this->angular_velocity;

    this->odom_data.position.x = this->x;
    this->odom_data.position.y = this->y;
    this->odom_data.position.z = 0;

    this->odom_data.orientation.x = 0;
    this->odom_data.orientation.y = 0;
    this->odom_data.orientation.z = sin(this->theta / 2);
    this->odom_data.orientation.w = cos(this->theta / 2);

    // Publish odometry
    this->sendMessage(0, &this->odom_data, sizeof(this->odom_data));
}
