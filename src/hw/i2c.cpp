#include "i2c.hpp"
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include "config.hpp"
#include <memory>
#include <string.h>


using namespace sparkie;

I2C* I2C::instance = nullptr;

I2C* I2C::getInstance()
{
    if(instance == nullptr)
    {
        instance = new I2C();
    }

    return instance;
}

void I2C::init()
{
    i2c_dma_init(&this->i2c_dma, I2C_PORT, 100*1000, I2C_SDA, I2C_SCL);
}

int sparkie::I2C::raw_write(uint8_t addr, const uint8_t *wbuf, size_t wbuf_len)
{
    return i2c_dma_write(this->i2c_dma, addr, wbuf, wbuf_len);
}

int sparkie::I2C::write_into(uint8_t addr, uint8_t reg, const uint8_t *wbuf, size_t wbuf_len)
{
    uint8_t *buf = (uint8_t *)malloc(wbuf_len + 1);
    buf[0] = reg;
    memcpy(buf + 1, wbuf, wbuf_len);

    int status = i2c_dma_write(this->i2c_dma, addr, buf, wbuf_len + 1);

    free(buf);

    return status;
}

int sparkie::I2C::write_byte(uint8_t addr, uint8_t reg, uint8_t byte)
{
    return i2c_dma_write_byte(this->i2c_dma, addr, reg, byte);
}

int sparkie::I2C::write_word(uint8_t addr, uint8_t reg, uint16_t word)
{
    return i2c_dma_write_word(this->i2c_dma, addr, reg, word);
}

int sparkie::I2C::raw_read(uint8_t addr, uint8_t *rbuf, size_t rbuf_len)
{
    return i2c_dma_read(this->i2c_dma, addr, rbuf, rbuf_len);
}

int sparkie::I2C::read_byte(uint8_t addr, uint8_t reg, uint8_t *data)
{
    return i2c_dma_read_byte(this->i2c_dma, addr, reg, data);
}

int8_t read_registers(uint8_t port, uint8_t addr, uint8_t reg, uint8_t count, uint8_t *buffer)
{
    int read;
    i2c_write_timeout_us(port == 0 ? i2c0 : i2c1, addr, &reg, 1, true, 200);
    read = i2c_read_timeout_us(port == 0 ? i2c0 : i2c1, addr, buffer, count, false, 200);
    if (read < count)
    {
        return PICO_ERROR_GENERIC;
    }
    return PICO_OK;
}

int sparkie::I2C::read_from(uint8_t addr, uint8_t reg, uint8_t *rbuf, size_t rbuf_len)
{
    int status;

    status = i2c_dma_write(this->i2c_dma, addr, &reg, 1);

    if (status != PICO_OK)
    {
        return status;
    }

    status = i2c_dma_read(this->i2c_dma, addr, rbuf, rbuf_len);

    if (status != PICO_OK)
    {
        return status;
    }


    return PICO_OK;
}

int sparkie::I2C::read_word_swapped(uint8_t addr, uint8_t reg, uint16_t *word)
{
    return i2c_dma_read_word_swapped(this->i2c_dma, addr, reg, word);
}
