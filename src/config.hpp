#pragma once

#define UROS_UART

#define BUZZ_MUTE

// Micro Ros settings
#define UROS_NAMESPACE "sparkie"
#define UROS_NODE_NAME "board"
#define UROS_PING_TIMEOUT 1000
#define UROS_PING_ATTEMPTS 60
#define UROS_DEFAULT_POLL_TIME 300

// URosComponent's task update rates
#define UROS_IMU_RATE 60.0
#define UROS_BATTERY_RATE 1.0
#define UROS_DHT_RATE 0.2
#define UROS_ULTRASONIC_RATE 24.0 // This will be fixed because limited by the sensor.
#define UROS_SERVO_RATE 20.0
#define UROS_LEDSTRIP_RATE 24.0
#define UROS_STATS_RATE 0.2
#define UROS_MOTORS_RATE 60.0

// Frame names
#define UROS_BASE_FRAME "base_link"
#define UROS_ULTRASONIC_FRAME "us_"
#define UROS_BOARD_FRAME "board"
#define UROS_ODOM_FRAME "odom"
#define UROS_IMU_FRAME "imu"

// Hardware related

// UART
#define UART_TX 16
#define UART_RX 17

// Misc
#define LED_PIN 0
#define BUZZ_PIN 2
#define DHT_PIN 29

// Battery 
#define BATT_VOLT_PIN 26
#define BATT_RECHARGE_PIN 8
// Reads count for running average algorithm
#define BATT_READS_COUNT 10
#define BATT_CAPACITY 4 // Ampere
// Ultrasonic Sensors
#define US_TRIG_PIN 3
#define US_LEFT_PIN 4
#define US_FRONT_PIN 5
#define US_RIGHT_PIN 6

#define US_NUM 3
#define US_FOV 0.26
#define US_MIN_RANGE 0.02
#define US_MAX_RANGE 4

// Servos
#define SERVO_FREQUENCY 50
#define SERVO_HEAD_TILT_PIN 7

#define SERVO_ZERO_POS 0
#define SERVO_SOFT_MAX_ANGLE 25
#define SERVO_SOFT_MIN_ANGLE -10

#define SERVO_MIN_MICROS 488
#define SERVO_MAX_MICROS 2500

#define SERVO_MIN_ANGLE -90
#define SERVO_MAX_ANGLE 90

// LedStrip
#define LEDSTRIP_PIN 27
#define LED_OFFSET 4
#define LED_LENGTH 30

// Imu
#define IMU_ADDR 0x69
#define IMU_EN_PIN 25
#define IMU_SAMPLES_NUM 0

// Motors
#define MOTORS_NUM 2
#define MOTORS_WHEEL_RADIUS 0.0335 // meters
#define MOTORS_WHEEL_SEPARATION 0.243 // meters

#define MOTOR_SOFT_MAX_RPM 75
#define MOTOR_MAX_RPM 178
#define MOTOR_MIN_RPM 10
#define MOTOR_PULSES_PER_REVOLUTIONS 11.0
#define MOTOR_REDUCTION_RATE 56.0

#define MOTOR_PID_KP 0.2
#define MOTOR_PID_KI 2.8
#define MOTOR_PID_KD 0.015

#define MOTORS_ENABLE_PIN 9

#define MOTOR_A_PWM_PIN 10
#define MOTOR_A0_PIN 11
#define MOTOR_A1_PIN 12
#define MOTOR_A_ENC0_PIN 22
#define MOTOR_A_ENC1_PIN 23

#define MOTOR_B_PWM_PIN 13
#define MOTOR_B0_PIN 14
#define MOTOR_B1_PIN 15
#define MOTOR_B_ENC0_PIN 18
#define MOTOR_B_ENC1_PIN 19

// Battery

