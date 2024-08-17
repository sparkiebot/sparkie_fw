#pragma once

#define BUZZ_MUTE

// Micro Ros settings
#define UROS_NAMESPACE "sparkie"
#define UROS_NODE_NAME "board"
#define UROS_PING_TIMEOUT 1000
#define UROS_PING_ATTEMPTS 60

// URosComponent's task update rates
#define UROS_IMU_RATE 50.0
#define UROS_BATTERY_RATE 1.0
#define UROS_AIRQUALITY_RATE 0.2
#define UROS_AHT_RATE 0.2
#define UROS_ULTRASONIC_RATE 24.0 // This will be fixed because limited by the sensor.
#define UROS_SERVO_RATE 20.0
#define UROS_SYSTEM_MANAGER_RATE 1.0
#define UROS_LEDSTRIP_RATE 24.0
#define UROS_STATS_RATE 0.2
#define UROS_MOTORS_RATE 30.0

// Frame names
#define UROS_BASE_FRAME "base_link"
#define UROS_ULTRASONIC_FRAME "us_"
#define UROS_BOARD_FRAME "board_link"
#define UROS_IMU_FRAME "imu_link"

// Hardware related

// I2C
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

// UART
#define UART_TX 16
#define UART_RX 17

// Misc
#define LED_PIN 10
#define BUZZ_PIN 9

// Battery 
#define BATT_VOLT_PIN 26
#define BATT_RECHARGE_PIN 8
// Reads count for running average algorithm
#define BATT_READS_COUNT 10
#define BATT_CAPACITY 4 // Ampere

// Ultrasonic Sensors
#define US_TRIG_PIN 0
#define US_LEFT_PIN 1
#define US_FRONT_PIN 2
#define US_RIGHT_PIN 3

#define US_NUM 3
#define US_FOV 0.26
#define US_MIN_RANGE 0.02
#define US_MAX_RANGE 4

// Servos
#define SERVO_FREQUENCY 50
#define SERVO_HEAD_TILT_PIN 13

#define SERVO_ZERO_POS 0
#define SERVO_SOFT_MAX_ANGLE 30
#define SERVO_SOFT_MIN_ANGLE -30

#define SERVO_MIN_MICROS 488

#define SERVO_MAX_MICROS 2500

#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE 90

// LedStrip
#define LEDSTRIP_PIN 11
#define LED_OFFSET 4
#define LED_LENGTH 30

// Imu
#define IMU_ADDR 0x69
#define IMU_EN_PIN 25
#define IMU_SAMPLES_NUM 0

// Motors
#define MOTORS_NUM 2
#define MOTORS_WHEEL_RADIUS 0.0335 // meters
#define MOTORS_WHEEL_SEPARATION 0.2182 // meters

#define MOTOR_SOFT_MAX_RPM 178
#define MOTOR_MAX_RPM 178
#define MOTOR_MIN_RPM 10
#define MOTOR_PULSES_PER_REVOLUTIONS 11.0
#define MOTOR_REDUCTION_RATE 56.0


#define MOTOR_PID_KP 50
#define MOTOR_PID_KI 3.5
#define MOTOR_PID_KD -10

#define MOTOR_DEADBAND_PWM 12000

#define MOTOR_A0_PIN 20
#define MOTOR_A1_PIN 21
#define MOTOR_A_ENC0_PIN 14
#define MOTOR_A_ENC1_PIN 15

#define MOTOR_B0_PIN 18
#define MOTOR_B1_PIN 19
#define MOTOR_B_ENC0_PIN 17
#define MOTOR_B_ENC1_PIN 16

// AirQuality Sensor
#define AIRQ_ADDR 0x5A
#define AIRQ_ID 0x81
#define AIRQ_WAKE_PIN 12
#define AIRQ_DISABLED_MODE 0
#define AIRQ_EVERYSEC_MODE 1
#define AIRQ_EVERYTENSEC_MODE 2
#define AIRQ_EVERYMIN_MODE 3

#define AIRQ_REG_STATUS 0x00
#define AIRQ_REG_DEVID 0x20
#define AIRQ_REG_MEASMODE 0x01
#define AIRQ_REG_DATA 0x02
#define AIRQ_REG_ENV 0x05

// Action Button

#define ACTN_BTN_PIN 24