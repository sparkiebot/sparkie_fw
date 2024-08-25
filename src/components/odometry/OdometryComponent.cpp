#include "OdometryComponent.hpp"
#include "../../config.hpp"
#include "../../sparkie_defs.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include "../motors/MotorsComponent.hpp"
#include <math.h>

using namespace sparkie;

OdometryComponent::OdometryComponent()
    : URosComponent("odometry", CORE1, ODOM_PRIORITY, UROS_ODOM_RATE)
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
}

void OdometryComponent::rosInit()
{
    // Initialize odometry publisher

    this->addPublisher(
        "wheels/odom", 
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), false
    );

    // Initialize odometry message

    this->odom_msg.pose.pose.position.x = 0;
    this->odom_msg.pose.pose.position.y = 0;
    this->odom_msg.pose.pose.position.z = 0;

    this->odom_msg.pose.pose.orientation.x = 0;
    this->odom_msg.pose.pose.orientation.y = 0;
    this->odom_msg.pose.pose.orientation.z = 0;
    this->odom_msg.pose.pose.orientation.w = 1;

    this->odom_msg.twist.twist.linear.x = 0;
    this->odom_msg.twist.twist.linear.y = 0;
    this->odom_msg.twist.twist.linear.z = 0;

    this->odom_msg.twist.twist.angular.x = 0;
    this->odom_msg.twist.twist.angular.y = 0;
    this->odom_msg.twist.twist.angular.z = 0;

    // Initialize odometry message covariance
    for (int i = 0; i < 36; i++)
    {
        this->odom_msg.pose.covariance[i] = 0;
        this->odom_msg.twist.covariance[i] = 0;
    }

    this->odom_msg.header.frame_id = micro_ros_string_utilities_init(UROS_ODOM_FRAME);
    this->odom_msg.child_frame_id = micro_ros_string_utilities_init(UROS_BASE_FRAME);
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

    // Update odometry message
    this->odom_msg.twist.twist.linear.x = this->linear_velocity;
    this->odom_msg.twist.twist.angular.z = this->angular_velocity;

    this->odom_msg.pose.pose.position.x = this->x;
    this->odom_msg.pose.pose.position.y = this->y;
    this->odom_msg.pose.pose.position.z = 0;

    this->odom_msg.pose.pose.orientation.x = 0;
    this->odom_msg.pose.pose.orientation.y = 0;
    this->odom_msg.pose.pose.orientation.z = sin(this->theta / 2);
    this->odom_msg.pose.pose.orientation.w = cos(this->theta / 2);

    // Publish odometry
    this->sendMessage(0, &this->odom_msg);

}