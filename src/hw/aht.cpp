#include "aht.hpp"
#include "i2c.hpp"

#define AHT_CMD_NORMAL 0xA8
#define AHT_CMD_INIT 0xE1
#define AHT_CMD_MEAS 0xAC
#define AHT_CMD_SOFT_RESET 0xBA

#define AHT_DATA_MEASURMENT_CMD 0x33


#define AHT_CMD_DELAY 350

sparkie::AHT::AHT(uint8_t addr)
{
    this->addr = addr;
}

int sparkie::AHT::init()
{
    int status;


    status = I2C::getInstance()->write_word(this->addr, AHT_CMD_NORMAL, 0x00);

    if (status != PICO_OK)
    {
        return status;
    }

    vTaskDelay(AHT_CMD_DELAY / portTICK_PERIOD_MS);

    uint16_t payload = 0x08 << 8 || 0x00;

    status = I2C::getInstance()->write_byte(this->addr, AHT_CMD_INIT, payload);

    if (status != PICO_OK)
    {
        return status;
    }

    vTaskDelay(AHT_CMD_DELAY / portTICK_PERIOD_MS);

    return PICO_OK;
}

int sparkie::AHT::read_data(float *temp, float *hum)
{

    int status;

    // Raw data read

    status = I2C::getInstance()->write_byte(this->addr, AHT_CMD_MEAS, AHT_DATA_MEASURMENT_CMD);

    if (status != PICO_OK)
    {
        return status;
    }

    uint8_t raw_data[6];

    status = I2C::getInstance()->raw_read(this->addr, raw_data, 6);

    if (status != PICO_OK)
    {
        return status;
    }

    // Data processing

    uint32_t raw_hum = (((uint32_t)raw_data[1] << 16) | 
        ((uint16_t)raw_data[2] << 8) | (raw_data[3])) >> 4; // 20-bit raw humidity data

    *hum = (float)raw_hum * 0.000095;

    if (*hum < 0)
        *hum = 0;
    if (*hum > 100)
        *hum = 100;

    uint32_t raw_temp = ((uint32_t)(raw_data[3] & 0x0F) << 16) |
                        ((uint16_t)raw_data[4] << 8) | raw_data[5]; // 20-bit raw temperature data

    *temp = (float)(raw_temp * 0.000191 - 50);

    return PICO_OK;


}
