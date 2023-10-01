#pragma once

#define I2C_PORT i2c0
#define I2C_SDA 20
#define I2C_SCL 21

#define CORE0 0x01
#define CORE1 0x02
#define CORE_UNDEFINED 0x03

#define HZ_TO_MS(hz) (1000 / hz) / portTICK_PERIOD_MS

#ifdef DEBUG
    #define ROS_CHECK(ret) if(ret != RCL_RET_OK) while(true) { std::cout << ("Error runnig ros function: " + std::to_string(ret)) << "\r\n"; vTaskDelay(2000 / portTICK_PERIOD_MS); }
#else
    #define ROS_CHECK(ret) ret
#endif

#define UROS_PUB_QUEUE_SIZE 1

#define MAX_TASK_NUM 32

#define AGENT_PRIORITY 60
#define IMU_PRIORITY 45
#define MOTORS_PRIORITY 55
#define SERVO_PRIORITY 40
#define ULTRASONIC_PRIORITY 30
#define LEDSTRIP_PRIORITY 25
#define DHT_PRIORITY 15
#define BATT_PRIORITY 14
#define SYS_MGNT_PRIORITY 13
#define AIRQUALITY_PRIORITY 12
#define STATS_PRIORITY 10
#define LOGGER_PRIORITY 5

#define AGENT_UPDATE_RATE 60

#define RPM_TO_RADS 0.10472

#define SPARKIE_VIS_DEBUG(ms) gpio_put(LED_PIN, 1); vTaskDelay(ms / portTICK_PERIOD_MS); gpio_put(LED_PIN, 0);

// Use this define if you wanna see stats published
//#define SPARKIE_SHOW_STATS

#define LED_BATTERY_LAYER 0 
#define LED_NOTIFY_LAYER 1


#pragma region Notes
// Notes
#define NOTE_MUTE 0
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

#pragma endregion Notes

