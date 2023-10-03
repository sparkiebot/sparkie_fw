#include "AirQualityComponent.hpp"
#include "../sparkie_defs.hpp"
#include "../config.hpp"
#include "LoggerComponent.hpp"

#include <hardware/i2c.h>

using namespace sparkie;

QueueHandle_t AirQualityComponent::dht_queue;

AirQualityComponent::AirQualityComponent() 
    : URosComponent("co2_sensor", CORE1, AIRQUALITY_PRIORITY, UROS_AIRQUALITY_RATE)
{

}

void AirQualityComponent::init()
{
    dht_queue = xQueueCreate(1, sizeof(DhtData));
    this->init_error = false;

    uint8_t id;
    if((id = this->getDeviceID()) != AIRQ_ID)
    {
        this->init_error = true;
        return;
    }
    
    this->reset();
    vTaskDelay(1);
    this->setMeasMode(AIRQ_EVERYSEC_MODE);
}

void AirQualityComponent::rosInit()
{
    if(this->init_error)
    {
        LoggerComponent::log(LogLevel::Error, 
            "Invalid air quality sensor id");
    }

    this->addPublisher(
        "air/co2",
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16)
    );
}

bool AirQualityComponent::read_from(uint8_t reg, uint8_t* buff, uint8_t len)
{
    auto written = i2c_write_timeout_us(I2C_PORT, AIRQ_ADDR, &reg, 1, true, 2000);
    if(written != 1)
        return false;
    
    auto read = i2c_read_timeout_us(I2C_PORT, AIRQ_ADDR, buff, len, false, 2000);
    if(read != len)
        return false;
    
    return true;
}

bool AirQualityComponent::write_into(uint8_t reg, uint8_t* buff, uint8_t len)
{
    // maximum buff size we are using will be 4
    uint8_t data_raw[] = {0, 0, 0, 0, 0};
    data_raw[0] = reg;
    for (size_t i = 1; i < len + 1; i++)
    {
        data_raw[i] = buff[i-1];
    }
    
    auto written = i2c_write_timeout_us(I2C_PORT, AIRQ_ADDR, data_raw, len + 1, false, 2000);
    
    if(written != (len + 1))
        return false;

    return true;
}

uint8_t AirQualityComponent::getDeviceID()
{
    uint8_t data;
    if(this->read_from(AIRQ_REG_DEVID, &data, 1))
        return data;
    
    return 0;
}

void AirQualityComponent::reset()
{
    uint8_t reset_value = 0xF4;
    i2c_write_blocking(I2C_PORT, AIRQ_ADDR, &reset_value, 1, false);
}


void AirQualityComponent::setMeasMode(uint8_t mode)
{
    uint8_t mode_raw = mode << 4;
    this->write_into(AIRQ_REG_MEASMODE, &mode_raw, 1);
}

bool AirQualityComponent::isReady()
{
    uint8_t status;
    if(this->read_from(AIRQ_REG_STATUS, &status, 1))
    {
        return (status >> 3) & 0x01;
    }

    return false;
}

uint16_t AirQualityComponent::getECo2()
{
    uint8_t data_raw[2];
    if(this->read_from(AIRQ_REG_DATA, data_raw, 2))
    {
        return ((uint16_t)data_raw[0] << 8) | data_raw[1];
    }

    return 0;
}   

#define uint16to8(value) (uint8_t)(value & 0xff), (uint8_t)(value >> 8)

void AirQualityComponent::setEnvData(float temp, float hum)
{
    uint16_t raw_temp = (temp + 25.0) * 512;
    uint16_t raw_hum = hum * 512;
    uint8_t buff[] = {uint16to8(raw_hum), uint16to8(raw_temp)};
    this->write_into(AIRQ_REG_ENV, buff, 4);
}


void AirQualityComponent::loop(TickType_t* xLastWakeTime)
{
    if(xQueueReceive(dht_queue, &this->dht_data, 0) != pdFALSE)
    {
        this->setEnvData(this->dht_data.temp, this->dht_data.hum);
    }

    if(this->isReady())
    {
        this->co2_msg.data = this->getECo2();
    }

    this->sendMessage(0, &this->co2_msg);
}