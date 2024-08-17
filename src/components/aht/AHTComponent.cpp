#include "AHTComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../../config.hpp"
#include "../air_quality/AirQualityComponent.hpp"
#include "../logger/LoggerComponent.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <iostream>

using namespace sparkie;

AHTComponent::AHTComponent() : URosComponent("aht_sensor", CORE1, AHT_PRIORITY, UROS_AHT_RATE),
                               sensor(AHT10_ADDRESS_0X38, I2C_PORT, I2C_SDA, I2C_SCL, 400)
{
}

void AHTComponent::init()
{

    // Variances are calculated from AHT10 datasheet

    this->temp_msg.header.frame_id =
        micro_ros_string_utilities_init(UROS_BOARD_FRAME);
    this->temp_msg.variance = 0.09;

    this->hum_msg.header.frame_id =
        micro_ros_string_utilities_init(UROS_BOARD_FRAME);
    this->hum_msg.variance = 0.004;

    this->sensor.AHT10_InitI2C(AHT10_SENSOR);
    this->sensor.AHT10_begin();
}

void AHTComponent::rosInit()
{
    this->addPublisher(
        "temperature",
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature));

    this->addPublisher(
        "humidity",
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity));
}

void AHTComponent::loop(TickType_t *xLastWakeTime)
{
    float temp, hum;
    
    temp = this->sensor.AHT10_readTemperature(true);
    hum = this->sensor.AHT10_readHumidity(true);

    this->temp_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();
    this->temp_msg.header.stamp.sec = (int32_t)(rmw_uros_epoch_millis() / 1000);

    this->temp_msg.temperature = temp;

    this->sendMessage(0, &this->temp_msg);


    this->hum_msg.header.stamp.nanosec = (uint32_t)rmw_uros_epoch_nanos();
    this->hum_msg.header.stamp.sec = (int32_t)(rmw_uros_epoch_millis() / 1000);

    this->hum_msg.relative_humidity = hum;

    this->sendMessage(1, &this->hum_msg);

    // Send data to air quality sensor.
    if (AgentComponent::isConnected() && AirQualityComponent::aht_queue != NULL)
    {
        this->aht_data.temp = temp;
        this->aht_data.hum = hum;
        xQueueOverwrite(AirQualityComponent::aht_queue, &this->aht_data);
    }
}