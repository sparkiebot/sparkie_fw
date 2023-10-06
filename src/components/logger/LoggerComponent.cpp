#include "LoggerComponent.hpp"
#include <iostream>
#include "../../sparkie_defs.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>

using namespace sparkie;

LoggerComponent* LoggerComponent::instance;

LoggerComponent::LoggerComponent() : URosComponent("logger", CORE0, LOGGER_PRIORITY)
{
    instance = this;
}

void LoggerComponent::rosInit()
{
    rcl_publisher_t* pub;
    pub = (rcl_publisher_t*) pvPortMalloc(sizeof(rcl_publisher_t));

    ROS_CHECK(rclc_publisher_init(
        pub,
        this->parent_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
        "/rosout",
        &rmw_qos_profile_system_default
    ));

    UPub_t upub;
    upub.pub = pub;
    upub.pub_queue = xQueueCreate(1, sizeof(PubMsg_t));
    upub.msg.pub = pub;
    upub.msg.msg = nullptr;

    this->publishers.push_back(upub);

    log_msg.msg = micro_ros_string_utilities_init("");
    log_msg.file = micro_ros_string_utilities_init("");
    log_msg.function = micro_ros_string_utilities_init("");
    log_msg.name = micro_ros_string_utilities_init("sparkie_board");
}

void LoggerComponent::log(LogLevel level, std::string msg, const std::source_location& location)
{
    auto logger = LoggerComponent::instance;

    logger->log_msg.stamp.nanosec = (uint32_t) rmw_uros_epoch_nanos();
    logger->log_msg.stamp.sec = (int32_t) (rmw_uros_epoch_millis() / 1000);
    logger->log_msg.level = level;
    logger->log_msg.msg = micro_ros_string_utilities_set(logger->log_msg.msg, msg.c_str());
    logger->log_msg.file = micro_ros_string_utilities_set(logger->log_msg.file, location.file_name());
    logger->log_msg.function = micro_ros_string_utilities_set(logger->log_msg.function, location.function_name());
    logger->log_msg.line = location.line();
    
    if(AgentComponent::isConnected())
        logger->sendMessage(0, &logger->log_msg);
    else
        rcl_publish(logger->publishers[0].pub, &logger->log_msg, NULL);
}

void LoggerComponent::init()
{

}

void LoggerComponent::loop(TickType_t* xLastWakeTime)
{

}