#include "DHTComponent.hpp"
#include "../sparkie_defs.hpp"
#include "../config.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <iostream>

using namespace sparkie;

DHTComponent::DHTComponent() : URosComponent("dht_sensor", CORE1, DHT_PRIORITY, UROS_DHT_RATE)
{
}

void DHTComponent::init()
{

    // Variances are calculated from DHT11 datasheet

    this->temp_msg.header.frame_id = 
        micro_ros_string_utilities_init(UROS_BOARD_FRAME);
    this->temp_msg.variance = 0.02;

    this->hum_msg.header.frame_id = 
        micro_ros_string_utilities_init(UROS_BOARD_FRAME);
    this->hum_msg.variance = 0.05;

    dht_init(&this->sensor, DHT11, pio0, DHT_PIN, true);
}

void DHTComponent::rosInit()
{
    this->addPublisher(
        "temperature", 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature)
    );

    this->addPublisher(
        "humidity", 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity)
    );
}

void DHTComponent::loop(TickType_t* xLastWakeTime)
{
    float temp, hum;
    dht_start_measurement(&this->sensor);
    xTaskDelayUntil(xLastWakeTime, pdMS_TO_TICKS(30)); // Wait the required ms time
    dht_finish_measurement_blocking(&this->sensor, &hum, &temp);

    auto ns = (uint32_t) rmw_uros_epoch_nanos();
    auto s = (int32_t) (rmw_uros_epoch_millis() / 1000);
    
    this->temp_msg.header.stamp.nanosec = ns;
    this->temp_msg.header.stamp.sec = s;

    this->hum_msg.header.stamp.nanosec = ns;
    this->hum_msg.header.stamp.sec = s;
    
    
    this->temp_msg.temperature = temp;
    this->hum_msg.relative_humidity = hum;
    
    this->sendMessage(0, &this->temp_msg);
    this->sendMessage(1, &this->hum_msg);
}