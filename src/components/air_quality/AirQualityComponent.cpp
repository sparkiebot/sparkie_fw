#include "AirQualityComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../../config.hpp"
#include "../logger/LoggerComponent.hpp"

#include "hw/i2c.hpp"

using namespace sparkie;

const MessageType MSG_CO2 = {"co2", 0};
QueueHandle_t AirQualityComponent::aht_queue = nullptr;

AirQualityComponent::AirQualityComponent() 
    : MessageComponent("airq_sensor", CORE1, AIRQUALITY_PRIORITY, UROS_AIRQUALITY_RATE)
{

}

void AirQualityComponent::init()
{
    aht_queue = xQueueCreate(1, sizeof(AhtData));
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
    
    if(this->init_error)
    {
        LoggerComponent::log(LogLevel::Error, 
            "Invalid air quality sensor id");
    }

    
    
}


void AirQualityComponent::msgInit()
{
    this->addPublisher("/air/co2", &MSG_CO2);
}

uint8_t AirQualityComponent::getDeviceID()
{
    uint8_t data;
    
    if(I2C::getInstance()->read_byte(AIRQ_ADDR, AIRQ_REG_DEVID, &data) == PICO_OK)
    {
        return data;
    }
    
    return 0;
}

void AirQualityComponent::reset()
{
    uint8_t reset_value = 0xF4;
    I2C::getInstance()->raw_write(AIRQ_ADDR, &reset_value, 1);
}


void AirQualityComponent::setMeasMode(uint8_t mode)
{
    uint8_t mode_raw = mode << 4;
    I2C::getInstance()->write_byte(AIRQ_ADDR, AIRQ_REG_MEASMODE, mode_raw);
}

bool AirQualityComponent::isReady()
{
    uint8_t status;
    if(I2C::getInstance()->read_byte(AIRQ_ADDR, AIRQ_REG_STATUS, &status) == PICO_OK)
    {
        return (status >> 3) & 0x01;
    }

    return false;
}

void AirQualityComponent::obtainECo2()
{
    if(I2C::getInstance()->read_word_swapped(AIRQ_ADDR, AIRQ_REG_DATA, &this->co2_ppm) != PICO_OK)
    {
        this->co2_ppm = 0;
    }
}   

#define uint16to8(value) (uint8_t)(value & 0xff), (uint8_t)(value >> 8)

void AirQualityComponent::setEnvData(float temp, float hum)
{
    uint16_t raw_temp = (temp + 25.0) * 512;
    uint16_t raw_hum = hum * 512;
    uint8_t buff[] = {uint16to8(raw_hum), uint16to8(raw_temp)};
    //I2C::getInstance()->write_into(AIRQ_REG_ENV, buff, 4);
    I2C::getInstance()->write_word(AIRQ_ADDR, AIRQ_REG_ENV, raw_hum);
    I2C::getInstance()->write_word(AIRQ_ADDR, AIRQ_REG_ENV + 2, raw_temp);
}


void AirQualityComponent::loop(TickType_t* xLastWakeTime)
{
    if(xQueueReceive(this->aht_queue, &this->aht_data, 0) != pdFALSE)
    {
        this->setEnvData(this->aht_data.temp, this->aht_data.hum);
    }

    if (this->isReady())
    {
        this->obtainECo2();
        this->sendMessage(0, &this->co2_ppm, sizeof(this->co2_ppm));
    }

    
}