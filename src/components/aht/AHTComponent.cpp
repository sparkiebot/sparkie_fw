#include "AHTComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../../config.hpp"
#include "../air_quality/AirQualityComponent.hpp"
#include "../logger/LoggerComponent.hpp"
#include "components/messaging/MessageComponent.hpp"
#include "../messaging/MessageBroker.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <iostream>

using namespace sparkie;

const MessageType MSG_AHT_TEMPERATURE = {"temperature", 0};
const MessageType MSG_AHT_HUMIDITY = {"humidity", 1};

AHTComponent::AHTComponent() : MessageComponent("aht_sensor", CORE1, AHT_PRIORITY, UROS_AHT_RATE),
                               sensor(0x38)
{

}

void AHTComponent::init()
{

    // Variances are calculated from AHT10 datasheet
    this->sensor.init();
    
    this->addPublisher("/temperature", &MSG_AHT_TEMPERATURE);
    this->addPublisher("/humidity", &MSG_AHT_HUMIDITY);
}

void AHTComponent::loop(TickType_t *xLastWakeTime)
{
    this->sensor.read_data(&this->temp, &this->hum);

    

    this->sendMessage(0, &this->temp, sizeof(this->temp));


    this->sendMessage(1, &this->hum, sizeof(this->hum));

    // // Send data to air quality sensor.
    if (MessageBroker::isConnected() && AirQualityComponent::aht_queue != NULL)
    {
        this->aht_data.temp = temp;
        this->aht_data.hum = hum;
        xQueueOverwrite(AirQualityComponent::aht_queue, &this->aht_data);
    }
}