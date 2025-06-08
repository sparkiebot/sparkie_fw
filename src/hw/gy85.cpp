#include "gy85/gy85.hpp"
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c.hpp"

#define DEG_TO_RAD (M_PI / 180.0)

using namespace sparkie;

gy85::gy85(uint8_t i2c_port, uint8_t adxl345_addr, uint8_t itg3205_addr, uint8_t qmc5883l_addr)
{
    this->i2c_port = i2c_port;
    this->adxl345_addr = adxl345_addr;
    this->itg3205_addr = itg3205_addr;
    this->qmc5883l_addr = qmc5883l_addr;

    this->adxl345_scale_factor = ADXL345_SCALE_FACTOR;
    
    this->accel.x = 0;
    this->accel.y = 0;
    this->accel.z = 0;

    this->accel_offset.x = 0;
    this->accel_offset.y = 0;
    this->accel_offset.z = 0;

    this->gyro.x = 0;
    this->gyro.y = 0;
    this->gyro.z = 0;

    this->gyro_offset.x = 0;
    this->gyro_offset.y = 0;
    this->gyro_offset.z = 0;

    this->mag.x = 0;
    this->mag.y = 0;
    this->mag.z = 0;
    
    this->sleep_fn = sleep_ms;
}

int gy85::init()
{
    if (init_adxl345() != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    if (init_itg3205() != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }
    
    if (init_qmc5883l() != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::read()
{
    if (read_adxl345(&this->accel) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    if (read_itg3205(&this->gyro) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    if (read_qmc5883l(&this->mag) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_sleep_fn(void (*sleep_fn)(uint32_t))
{
    this->sleep_fn = sleep_fn;
    return PICO_OK;
}

int gy85::calibrate(uint16_t samples)
{
    if (calibrate_adxl345(samples) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    if (calibrate_itg3205(samples) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

const vec3f_t gy85::get_accel()
{
    return this->accel;
}

const vec3f_t gy85::get_gyro()
{
    return this->gyro;
}

const vec3f_t gy85::get_mag()
{
    return this->mag;
}

/**
 * ADXL345 Functions
 */

int gy85::init_adxl345()
{
    uint8_t id;
    if(I2C::getInstance()->read_byte(this->adxl345_addr, ADXL345_REG_DEVID, &id) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    if (id != ADXL345_ID)
    {
        return PICO_ERROR_GENERIC;
    }

    // Power up the ADXL345
    if(I2C::getInstance()->write_byte(this->adxl345_addr, ADXL345_REG_POWER_CTL, 0x08) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    // Set range to +/- 2g
    if (set_adxl345_range(adxl345_range_t::RANGE_2_G) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    // Set data rate to 100Hz
    if (set_adxl345_data_rate(adxl345_data_rate_t::DATARATE_100_HZ) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }


    return PICO_OK;
}

int gy85::set_adxl345_range(adxl345_range_t range)
{
    uint8_t reg;
    
    if(I2C::getInstance()->read_byte(this->adxl345_addr, ADXL345_REG_DATA_FORMAT, &reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    reg &= ~0x0F;
    reg |= range;

    //if (write_register(this->i2c_port, this->adxl345_addr, ADXL345_REG_DATA_FORMAT, reg) != PICO_OK)
    if(I2C::getInstance()->write_byte(this->adxl345_addr, ADXL345_REG_DATA_FORMAT, reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    switch (range)
    {
    case adxl345_range_t::RANGE_2_G:
        this->adxl345_scale_factor = ADXL345_SCALE_FACTOR;
        break;
    case adxl345_range_t::RANGE_4_G:
        this->adxl345_scale_factor = ADXL345_SCALE_FACTOR * 2;
        break;
    case adxl345_range_t::RANGE_8_G:
        this->adxl345_scale_factor = ADXL345_SCALE_FACTOR * 4;
        break;
    case adxl345_range_t::RANGE_16_G:
        this->adxl345_scale_factor = ADXL345_SCALE_FACTOR * 8;
        break;
    default:
        this->adxl345_scale_factor = ADXL345_SCALE_FACTOR;
        break;
    }

    return PICO_OK;
}

int gy85::set_adxl345_data_rate(adxl345_data_rate_t dataRate)
{
    uint8_t reg;
    
    if(I2C::getInstance()->read_byte(this->adxl345_addr, ADXL345_REG_BW_RATE, &reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    reg &= ~0x0F;
    reg |= dataRate;

    if(I2C::getInstance()->write_byte(this->adxl345_addr, ADXL345_REG_BW_RATE, reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_adxl345_fifo_mode(adxl345_fifo_mode_t mode)
{
    uint8_t reg;
    
    if(I2C::getInstance()->read_byte(this->adxl345_addr, ADXL345_REG_FIFO_CTL, &reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    reg &= ~0b11000000;
    reg |= mode;

    if(I2C::getInstance()->write_byte(this->adxl345_addr, ADXL345_REG_FIFO_CTL, reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_adxl345_interrupt(bool enable)
{
    uint8_t reg;
    
    if(I2C::getInstance()->read_byte(this->adxl345_addr, ADXL345_REG_INT_ENABLE, &reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    reg &= ~0x80;
    reg |= enable << 7;

    if(I2C::getInstance()->write_byte(this->adxl345_addr, ADXL345_REG_INT_ENABLE, reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_adxl345_sleep(bool sleep)
{
    uint8_t reg;
    
    if(I2C::getInstance()->read_byte(this->adxl345_addr, ADXL345_REG_POWER_CTL, &reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    reg &= 0b11111011;
    reg |= sleep << 2;

    if(I2C::getInstance()->write_byte(this->adxl345_addr, ADXL345_REG_POWER_CTL, reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::calibrate_adxl345(uint16_t samples)
{
    vec3f_t accel;
    vec3f_t accel_sum = {0, 0, 0};

    for (uint16_t i = 0; i < samples; i++)
    {
        if (read_adxl345(&accel) != PICO_OK)
        {
            return PICO_ERROR_GENERIC;
        }

        accel_sum.x += accel.x;
        accel_sum.y += accel.y;

        this->sleep_fn(10);
    }

    this->accel_offset.x = accel_sum.x / samples;
    this->accel_offset.y = accel_sum.y / samples;
    this->accel_offset.z = 0;

    return PICO_OK;
}

int gy85::read_adxl345(vec3f_t *accel)
{
    uint8_t buffer[6];
    
    if(I2C::getInstance()->read_from(this->adxl345_addr, ADXL345_REG_DATAX0, buffer, 6) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    int16_t x = uint16_t(buffer[1]) << 8 | uint16_t(buffer[0]);
    int16_t y = uint16_t(buffer[3]) << 8 | uint16_t(buffer[2]);
    int16_t z = uint16_t(buffer[5]) << 8 | uint16_t(buffer[4]);

    accel->x = x * this->adxl345_scale_factor * SENSORS_GRAVITY_EARTH;
    accel->y = y * this->adxl345_scale_factor * SENSORS_GRAVITY_EARTH;
    accel->z = z * this->adxl345_scale_factor * SENSORS_GRAVITY_EARTH;

    accel->x -= this->accel_offset.x;
    accel->y -= this->accel_offset.y;
    accel->z -= this->accel_offset.z;

    return PICO_OK;
}

/**
 * ITG3205 Functions
 */

int gy85::init_itg3205()
{

    // Power up the ITG3205
    // Set clock source to internal oscillator
    
    if(I2C::getInstance()->write_byte(this->itg3205_addr, ITG3205_REG_PWR_MGM, 0x00) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    // Set sample rate divider
    // Sample rate = 1kHz / (divider + 1)
    // 1kHz / 8 = 125Hz
    if (set_itg3205_sample_rate_div(0x07) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    // Set digital low pass filter and full scale range
    // FS_SEL = 3 -> +/- 2000 deg/s
    // DLPF_CFG = 3 -> 42Hz low pass filter
    if (set_itg3205_dlpf_fs(0x1E) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_itg3205_sample_rate_div(uint8_t div)
{
    if(I2C::getInstance()->write_byte(this->itg3205_addr, ITG3205_REG_SMPLRT_DIV, div) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::get_itg3205_dlpf_fs(uint8_t *dlpf_fs)
{
    uint8_t reg;
    if(I2C::getInstance()->read_byte(this->itg3205_addr, ITG3205_REG_DLPF_FS, &reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    *dlpf_fs = reg;

    return PICO_OK;
}

int gy85::set_itg3205_dlpf_fs(uint8_t dlpf_fs)
{
    if(I2C::getInstance()->write_byte(this->itg3205_addr, ITG3205_REG_DLPF_FS, dlpf_fs) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_itg3205_fs(itg3205_fs_t fs)
{
    uint8_t curr_dlpf_fs;

    if (get_itg3205_dlpf_fs(&curr_dlpf_fs) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    curr_dlpf_fs &= ~0b00011000;
    curr_dlpf_fs |= fs << 3;

    if (set_itg3205_dlpf_fs(curr_dlpf_fs) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_itg3205_dlpf(itg3205_dlpf_t dlpf)
{
    uint8_t curr_dlpf_fs;

    if (get_itg3205_dlpf_fs(&curr_dlpf_fs) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    curr_dlpf_fs &= ~0b00000111;
    curr_dlpf_fs |= dlpf;

    if (set_itg3205_dlpf_fs(curr_dlpf_fs) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_itg3205_interrupt(bool enable)
{
    // Enable interrupt on data ready
    // Interupt clear on any read operation
    if(I2C::getInstance()->write_byte(this->itg3205_addr, ITG3205_REG_INT_CFG, enable | 0b00010000) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::reset_itg3205()
{
    if(I2C::getInstance()->write_byte(this->itg3205_addr, ITG3205_REG_PWR_MGM, 0x80) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    if (this->init_itg3205() != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_itg3205_sleep(bool sleep)
{
    uint8_t reg;
    if(I2C::getInstance()->read_byte(this->itg3205_addr, ITG3205_REG_PWR_MGM, &reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    reg &= ~0b01000000;
    reg |= sleep;

    if(I2C::getInstance()->write_byte(this->itg3205_addr, ITG3205_REG_PWR_MGM, reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::calibrate_itg3205(uint16_t samples)
{
    vec3f_t gyro;
    vec3f_t gyro_sum = {0, 0, 0};

    for (uint16_t i = 0; i < samples; i++)
    {
        if (read_itg3205(&gyro) != PICO_OK)
        {
            return PICO_ERROR_GENERIC;
        }

        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum.z += gyro.z;

        this->sleep_fn(10);
    }

    this->gyro_offset.x = gyro_sum.x / samples;
    this->gyro_offset.y = gyro_sum.y / samples;
    this->gyro_offset.z = gyro_sum.z / samples;

    return PICO_OK;
}

int gy85::read_itg3205(vec3f_t *gyro)
{
    uint8_t buffer[6];
    if(I2C::getInstance()->read_from(this->itg3205_addr, ITG3205_REG_GYRO_XOUT_H, buffer, 6) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    int16_t x = uint16_t(buffer[0]) << 8 | uint16_t(buffer[1]);
    int16_t y = uint16_t(buffer[2]) << 8 | uint16_t(buffer[3]);
    int16_t z = uint16_t(buffer[4]) << 8 | uint16_t(buffer[5]);

    gyro->x = x * (DEG_TO_RAD / ITG3205_DIGIT_TO_DEG);
    gyro->y = y * (DEG_TO_RAD / ITG3205_DIGIT_TO_DEG);
    gyro->z = z * (DEG_TO_RAD / ITG3205_DIGIT_TO_DEG);

    gyro->x -= this->gyro_offset.x;
    gyro->y -= this->gyro_offset.y;
    gyro->z -= this->gyro_offset.z;

    return PICO_OK;
}

/**
 * HMC5883L Functions
 */

int gy85::init_qmc5883l()
{
    // Set reset
    if(I2C::getInstance()->write_byte(this->qmc5883l_addr, QMC5883L_REG_CONFIG_B, 0x80) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }
    
    // Set period
    if(I2C::getInstance()->write_byte(this->qmc5883l_addr, QMC5883L_REG_PERIOD, 0x01) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    // Set roll over pointer
    if(I2C::getInstance()->write_byte(this->qmc5883l_addr, QMC5883L_REG_CONFIG_B, 0x40) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    // Set Continuous, 100 HZ, 2 GA, 512 OSR
    if (this->set_qmc5883l_ctrl(0x19) != PICO_OK)
    {
         return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::read_qmc5883l(vec3f_t *mag)
{
    uint8_t buffer[6];
    if(I2C::getInstance()->read_from(this->qmc5883l_addr, QMC5883L_REG_DATA, buffer, 6) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    int16_t x = (buffer[1]) << 8 | (buffer[0]);
    int16_t y = (buffer[3]) << 8 | (buffer[2]);
    int16_t z = (buffer[5]) << 8 | (buffer[4]);

    mag->x = x;
    mag->y = y;
    mag->z = z;

    return PICO_OK;
}

int gy85::get_qmc5883l_ctrl(uint8_t *ctrl)
{
    uint8_t reg;
    if(I2C::getInstance()->read_byte(this->qmc5883l_addr, QMC5883L_REG_CONFIG_A, &reg) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    *ctrl = reg;

    return PICO_OK;
}

int gy85::set_qmc5883l_ctrl(uint8_t ctrl)
{
    if(I2C::getInstance()->write_byte(this->qmc5883l_addr, QMC5883L_REG_CONFIG_A, ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_qmc5883l_mode(qmc5883l_mode_t mode)
{
    uint8_t curr_ctrl;
    if (get_qmc5883l_ctrl(&curr_ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    curr_ctrl &= ~0b00000011;
    curr_ctrl |= mode;

    if (set_qmc5883l_ctrl(curr_ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_qmc5883l_scale(qmc5883l_scale_t scale)
{
    uint8_t curr_ctrl;
    if (get_qmc5883l_ctrl(&curr_ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    curr_ctrl &= ~0b00110000;
    curr_ctrl |= scale << 4;

    if (set_qmc5883l_ctrl(curr_ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_qmc5883l_output_rate(qmc5883l_output_rate_t output_rate)
{
    uint8_t curr_ctrl;
    if (get_qmc5883l_ctrl(&curr_ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    curr_ctrl &= ~0b00001100;
    curr_ctrl |= output_rate << 2;

    if (set_qmc5883l_ctrl(curr_ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_qmc5883l_over_sample(qmc5883l_over_sample_t over_sample)
{
    uint8_t curr_ctrl;
    if (get_qmc5883l_ctrl(&curr_ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    curr_ctrl &= ~0b11000000;
    curr_ctrl |= over_sample << 6;

    if (set_qmc5883l_ctrl(curr_ctrl) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_qmc5883l_interrupt(bool enable)
{
    if(I2C::getInstance()->write_byte(this->qmc5883l_addr, QMC5883L_REG_CONFIG_B, !enable) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::reset_qmc5883l()
{
    if(I2C::getInstance()->write_byte(this->qmc5883l_addr, QMC5883L_REG_CONFIG_B, 0x80) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

int gy85::set_qmc5883l_sleep(bool sleep)
{
    if (this->set_qmc5883l_mode(sleep ? qmc5883l_mode_t::STANDBY : qmc5883l_mode_t::CONTINUOUS) != PICO_OK)
    {
        return PICO_ERROR_GENERIC;
    }

    return PICO_OK;
}

