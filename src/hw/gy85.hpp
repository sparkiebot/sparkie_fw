#pragma once
#include <stdint.h>

typedef struct
{
    double x;
    double y;
    double z;
} vec3f_t;

// ADXL345 Misc
#define ADXL345_ID (0xE5) ///< ADXL345 ID
#define ADXL345_ADDR (0x53) ///< ADXL345 I2C Address
#define ADXL345_SCALE_FACTOR (0.0039) ///< 4mg per lsb
#define SENSORS_GRAVITY_EARTH (9.80665F)

// ADXL345 Registers
#define ADXL345_REG_DEVID (0x00)        ///< Device ID
#define ADXL345_REG_THRESH_TAP (0x1D)   ///< Tap threshold
#define ADXL345_REG_OFSX (0x1E)         ///< X-axis offset
#define ADXL345_REG_OFSY (0x1F)         ///< Y-axis offset
#define ADXL345_REG_OFSZ (0x20)         ///< Z-axis offset
#define ADXL345_REG_DUR (0x21)          ///< Tap duration
#define ADXL345_REG_LATENT (0x22)       ///< Tap latency
#define ADXL345_REG_WINDOW (0x23)       ///< Tap window
#define ADXL345_REG_THRESH_ACT (0x24)   ///< Activity threshold
#define ADXL345_REG_THRESH_INACT (0x25) ///< Inactivity threshold
#define ADXL345_REG_TIME_INACT (0x26)   ///< Inactivity time
#define ADXL345_REG_ACT_INACT_CTL \
    (0x27)                                ///< Axis enable control for activity and inactivity detection
#define ADXL345_REG_THRESH_FF (0x28)      ///< Free-fall threshold
#define ADXL345_REG_TIME_FF (0x29)        ///< Free-fall time
#define ADXL345_REG_TAP_AXES (0x2A)       ///< Axis control for single/double tap
#define ADXL345_REG_ACT_TAP_STATUS (0x2B) ///< Source for single/double tap
#define ADXL345_REG_BW_RATE (0x2C)        ///< Data rate and power mode control
#define ADXL345_REG_POWER_CTL (0x2D)      ///< Power-saving features control
#define ADXL345_REG_INT_ENABLE (0x2E)     ///< Interrupt enable control
#define ADXL345_REG_INT_MAP (0x2F)        ///< Interrupt mapping control
#define ADXL345_REG_INT_SOURCE (0x30)     ///< Source of interrupts
#define ADXL345_REG_DATA_FORMAT (0x31)    ///< Data format control
#define ADXL345_REG_DATAX0 (0x32)         ///< X-axis data 0
#define ADXL345_REG_DATAX1 (0x33)         ///< X-axis data 1
#define ADXL345_REG_DATAY0 (0x34)         ///< Y-axis data 0
#define ADXL345_REG_DATAY1 (0x35)         ///< Y-axis data 1
#define ADXL345_REG_DATAZ0 (0x36)         ///< Z-axis data 0
#define ADXL345_REG_DATAZ1 (0x37)         ///< Z-axis data 1
#define ADXL345_REG_FIFO_CTL (0x38)       ///< FIFO control
#define ADXL345_REG_FIFO_STATUS (0x39)    ///< FIFO status

// ADXL345 Data Rate
typedef enum
{
    DATARATE_3200_HZ = 0b1111, ///< 1600Hz Bandwidth   140µA IDD
    DATARATE_1600_HZ = 0b1110, ///<  800Hz Bandwidth    90µA IDD
    DATARATE_800_HZ = 0b1101,  ///<  400Hz Bandwidth   140µA IDD
    DATARATE_400_HZ = 0b1100,  ///<  200Hz Bandwidth   140µA IDD
    DATARATE_200_HZ = 0b1011,  ///<  100Hz Bandwidth   140µA IDD
    DATARATE_100_HZ = 0b1010,  ///<   50Hz Bandwidth   140µA IDD
    DATARATE_50_HZ = 0b1001,   ///<   25Hz Bandwidth    90µA IDD
    DATARATE_25_HZ = 0b1000,   ///< 12.5Hz Bandwidth    60µA IDD
    DATARATE_12_5_HZ = 0b0111, ///< 6.25Hz Bandwidth    50µA IDD
    DATARATE_6_25HZ = 0b0110,  ///< 3.13Hz Bandwidth    45µA IDD
    DATARATE_3_13_HZ = 0b0101, ///< 1.56Hz Bandwidth    40µA IDD
    DATARATE_1_56_HZ = 0b0100, ///< 0.78Hz Bandwidth    34µA IDD
    DATARATE_0_78_HZ = 0b0011, ///< 0.39Hz Bandwidth    23µA IDD
    DATARATE_0_39_HZ = 0b0010, ///< 0.20Hz Bandwidth    23µA IDD
    DATARATE_0_20_HZ = 0b0001, ///< 0.10Hz Bandwidth    23µA IDD
    DATARATE_0_10_HZ = 0b0000  ///< 0.05Hz Bandwidth    23µA IDD (default value)
} adxl345_data_rate_t;

// ADXL345 Range
typedef enum
{
    RANGE_16_G = 0b11, ///< +/- 16g
    RANGE_8_G = 0b10,  ///< +/- 8g
    RANGE_4_G = 0b01,  ///< +/- 4g
    RANGE_2_G = 0b00   ///< +/- 2g (default value)
} adxl345_range_t;

// ADXL345 FIFO Mode
typedef enum
{
    FIFO_BYPASS = 0b00, ///< Bypass mode
    FIFO_FIFO = 0b01,   ///< FIFO mode
    FIFO_STREAM = 0b10, ///< Stream mode
    FIFO_TRIGGER = 0b11 ///< Trigger mode
} adxl345_fifo_mode_t;

// ITG3205 Misc
#define ITG3205_ADDR (0x68) ///< ITG3205 I2C Address
#define ITG3205_DIGIT_TO_DEG 14.375

// ITG3205 Registers
#define ITG3205_REG_SMPLRT_DIV 0x15
#define ITG3205_REG_DLPF_FS 0x16
#define ITG3205_REG_INT_CFG 0x17
#define ITG3205_REG_PWR_MGM 0x3E
#define ITG3205_REG_GYRO_XOUT_H 0x1D

// ITG3205 Offsets
#define ITG3205_X_OFFSET 120
#define ITG3205_Y_OFFSET 20
#define ITG3205_Z_OFFSET 93

// ITG3205 Full Scale
typedef enum
{
    FS_2000 = 0x03,
} itg3205_fs_t;

// ITG3205 Digital Low Pass Filter
typedef enum
{
    DLPF_256_8 = 0x00,
    DLPF_188_1 = 0x01,
    DLPF_98_1 = 0x02,
    DLPF_42_1 = 0x03,
    DLPF_20_1 = 0x04,
    DLPF_10_1 = 0x05,
    DLPF_5_1 = 0x06,
} itg3205_dlpf_t;



// QMC5883L Misc
#define QMC5883L_ADDR (0x0D)
#define QMC5883L_ID (0xFF)

// QMC5883L Registers
#define QMC5883L_REG_ID (0x0D)
#define QMC5883L_REG_PERIOD (0x0B)
#define QMC5883L_REG_CONFIG_A (0x09)
#define QMC5883L_REG_CONFIG_B (0x0A)
#define QMC5883L_REG_DATA (0x00)

// QMC5883L Mode
typedef enum
{
    STANDBY = 0x00,
    CONTINUOUS = 0x01,
} qmc5883l_mode_t;

// QMC5883L Scale
typedef enum
{
    SCALE_2_GA = 0x00,
    SCALE_8_GA = 0x01,
} qmc5883l_scale_t;

// QMC5883L Output Rate
typedef enum
{
    OUTPUT_10_HZ = 0x00,
    OUTPUT_50_HZ = 0x01,
    OUTPUT_100_HZ = 0x02,
    OUTPUT_200_HZ = 0x03,
} qmc5883l_output_rate_t;

// QMC5883L Over Sample Rate
typedef enum
{
    OVER_SAMPLE_512 = 0x00,
    OVER_SAMPLE_256 = 0x01,
    OVER_SAMPLE_128 = 0x02,
    OVER_SAMPLE_64 = 0x03,
} qmc5883l_over_sample_t;

class gy85
{
private:
    uint8_t i2c_port;
    uint8_t adxl345_addr;
    uint8_t itg3205_addr;
    uint8_t qmc5883l_addr;

    float adxl345_scale_factor;
    
    vec3f_t accel, accel_offset;
    vec3f_t gyro, gyro_offset;
    vec3f_t mag;

    void (*sleep_fn)(uint32_t);
public:
    gy85(uint8_t i2c_port = 0, uint8_t adxl345_addr = ADXL345_ADDR, uint8_t itg3205_addr = ITG3205_ADDR, uint8_t qmc5883l_addr = QMC5883L_ADDR);

    int init();

    int set_sleep_fn(void (*sleep_fn)(uint32_t));

    int calibrate(uint16_t samples = 20);
    int read();

    const vec3f_t get_accel();
    const vec3f_t get_gyro();
    const vec3f_t get_mag();
    

    /**
     * ADXL345 functions
     */

    int init_adxl345();
    int read_adxl345(vec3f_t *accel);
    int calibrate_adxl345(uint16_t samples = 20);
    int set_adxl345_range(adxl345_range_t range);
    int set_adxl345_data_rate(adxl345_data_rate_t dataRate);
    int set_adxl345_interrupt(bool enable);
    int set_adxl345_fifo_mode(adxl345_fifo_mode_t mode);
    int set_adxl345_sleep(bool sleep);

    /**
     * ITG3205 functions
     */

    int init_itg3205();
    int read_itg3205(vec3f_t *gyro);
    int calibrate_itg3205(uint16_t samples = 20);
    int set_itg3205_sample_rate_div(uint8_t div);
    int set_itg3205_dlpf_fs(uint8_t dlpf_fs);
    int get_itg3205_dlpf_fs(uint8_t *dlpf_fs);
    int set_itg3205_fs(itg3205_fs_t fs);
    int set_itg3205_dlpf(itg3205_dlpf_t dlpf);
    int set_itg3205_interrupt(bool enable);
    int set_itg3205_sleep(bool sleep);
    int reset_itg3205();
    
    /**
     * QMC5883L functions
     */

    int init_qmc5883l();
    int read_qmc5883l(vec3f_t *mag);
    int get_qmc5883l_ctrl(uint8_t *ctrl);
    int set_qmc5883l_ctrl(uint8_t ctrl);
    int set_qmc5883l_mode(qmc5883l_mode_t mode);
    int set_qmc5883l_scale(qmc5883l_scale_t scale);
    int set_qmc5883l_output_rate(qmc5883l_output_rate_t output_rate);
    int set_qmc5883l_over_sample(qmc5883l_over_sample_t over_sample);
    int set_qmc5883l_interrupt(bool enable);
    int set_qmc5883l_sleep(bool sleep);
    int reset_qmc5883l();


};
