#include "components/logger/LoggerComponent.hpp"
#include "pico/stdlib.h"
#include <iostream>
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include "misc/TaskRunTime.h"

#include <bsp/board_api.h>
#include <tusb.h>

#include <stdlib.h>
#include <hardware/i2c.h>
#include <hardware/watchdog.h>
#include <tusb.h>

#include "components/agent/AgentComponent.hpp"
#include "components/system/SystemComponent.hpp"
#include "components/imu/ImuComponent.hpp"
#include "components/servo/ServoComponent.hpp"
#include "components/air_quality/AirQualityComponent.hpp"
#include "components/aht/AHTComponent.hpp"
#include "components/battery/BatteryComponent.hpp"
#include "components/ultrasonic/UltrasonicComponent.hpp"
#include "components/buzzer/BuzzerComponent.hpp"
#include "components/led_strip/LedStripComponent.hpp"
#include "components/motors/MotorsComponent.hpp"
#include "components/odometry/OdometryComponent.hpp"
#include "hw/i2c.hpp"
#include "projdefs.h"
#include "sparkie_defs.hpp"

#include "config.hpp"
#include "transports/tusb_transport.hpp"

#include <array>

template <typename Base, typename T>
inline bool instanceof(const T *ptr)
{
    return dynamic_cast<const Base *>(ptr) != nullptr;
}
/*
This function adds a watchdog timer to the default idle function.
If any tasks is still blocked, the board will restart automatically.
*/
extern "C"
{
    bool wdt_started = false;
    void vApplicationIdleHook(void)
    {
        if (!wdt_started && sparkie::AgentComponent::isConnected())
        {
            watchdog_enable(2000, false);
            wdt_started = true;
        }

        watchdog_update();
    }
}

void onActionBtn(uint gpio, uint32_t events)
{
    sparkie::BuzzerComponent::play(sparkie::BuzzerAction::ERROR);
    // It will reboot the board.
    watchdog_reboot(0, 0, 0);
}

void setup_task(void *params)
{
    /**
     * Initializing I2C port
     */
    sparkie::I2C::getInstance()->init();
    vTaskDelay(100 / portTICK_PERIOD_MS);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    sparkie::BuzzerComponent::init();
    sparkie::BuzzerComponent::play(sparkie::PLAY_STARTUP);

    sparkie::TinyUSBTransport transport;
    transport.start();

    vTaskDelay(pdMS_TO_TICKS(5000));
    
    /**
     * Initializing all components
     */

    std::vector<std::unique_ptr<sparkie::Component>> components;

    components.emplace_back(std::make_unique<sparkie::ImuComponent>());
    components.emplace_back(std::make_unique<sparkie::LedStripComponent>());
    components.emplace_back(std::make_unique<sparkie::UltrasonicComponent>());
    components.emplace_back(std::make_unique<sparkie::AirQualityComponent>());
    components.emplace_back(std::make_unique<sparkie::AHTComponent>());
    components.emplace_back(std::make_unique<sparkie::BatteryComponent>());
    components.emplace_back(std::make_unique<sparkie::MotorsComponent>());
    components.emplace_back(std::make_unique<sparkie::OdometryComponent>());
    components.emplace_back(std::make_unique<sparkie::ServoComponent>("head/tilt", SERVO_HEAD_TILT_PIN, SERVO_FREQUENCY));
    components.emplace_back(std::make_unique<sparkie::SystemComponent>());

    auto uros_agent = sparkie::AgentComponent();

    for (auto &&comp : components)
    {
        if (instanceof<sparkie::URosComponent>(comp.get()))
            uros_agent.addComponent(dynamic_cast<sparkie::URosComponent *>(comp.get()));
    }

    /**
     * Adding a delay between each component start to give them time to initialize correctly.
     */

    for (auto &&comp : components)
    {
        comp->start();
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    /**
     * Finally start agent component and wait for the microros agent connection.
     */

    //sparkie::LedStripComponent::getInstance()->setMode(sparkie::LedStripMode::StartingUp);

    uros_agent.start();
    
    
    
    vTaskSuspend(NULL);
}

int main()
{
    // let pico sdk use the first cdc interface for std io
    // Initialize TinyUSB stack
    board_init();
    tusb_init();

    // TinyUSB board init callback after init
    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }

    // let pico sdk use the first cdc interface for std io
    stdio_init_all();


    // Start setup task
    TaskHandle_t task;
    xTaskCreate(setup_task, "setup", 5000, NULL, 5, &task);
    vTaskCoreAffinitySet(task, CORE0);


    vTaskStartScheduler();
}