#include "pico/stdlib.h"
#include <iostream>
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include "misc/TaskRunTime.h"

#include <stdlib.h>
#include <hardware/i2c.h>
#include <hardware/watchdog.h>

#include "components/AgentComponent.hpp"
#include "components/SystemComponent.hpp"
#include "components/StatsComponent.hpp"
#include "components/ImuComponent.hpp"
#include "components/ServoComponent.hpp"
#include "components/DHTComponent.hpp"
#include "components/BatteryComponent.hpp"
#include "components/UltrasonicComponent.hpp"
#include "components/BuzzerComponent.hpp"
#include "components/LedStripComponent.hpp"
#include "components/MotorsComponent.hpp"
#include "sparkie_defs.hpp"

#include "config.hpp"

template<typename Base, typename T>
inline bool instanceof(const T *ptr) {
   return dynamic_cast<const Base*>(ptr) != nullptr;
}

/*
This function adds a watchdog timer to the default idle function.
If any tasks is still blocked, the board will restart automatically.
*/
extern "C" {
    bool wdt_started = false;
    void vApplicationIdleHook(void)
    {
        if(!wdt_started)
        {
            watchdog_enable(2000, false);
            wdt_started = true;
        }

        watchdog_update();
    }
}

void setup_task(void* params)
{
    std::vector<sparkie::Component*> components;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    sparkie::BuzzerComponent::init();
    sparkie::BuzzerComponent::play(sparkie::PLAY_STARTUP);

    components.push_back(new sparkie::LedStripComponent());
    components.push_back(new sparkie::UltrasonicComponent());
    components.push_back(new sparkie::DHTComponent());
    components.push_back(new sparkie::BatteryComponent());
    components.push_back(new sparkie::MotorsComponent());
    components.push_back(new sparkie::ServoComponent("head/tilt", SERVO_HEAD_TILT_PIN, SERVO_FREQUENCY));
    components.push_back(new sparkie::StatsComponent());
    components.push_back(new sparkie::ImuComponent());
    components.push_back(new sparkie::SystemComponent());
    
    auto uros_agent = new sparkie::AgentComponent();

    for (auto &&comp :components)
    {
        if(instanceof<sparkie::URosComponent>(comp))
            uros_agent->addComponent(dynamic_cast<sparkie::URosComponent*>(comp));
    }

    for (auto &&comp :components)
    {
        comp->start();
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    uros_agent->start();

    vTaskSuspend(NULL);
}

int main()
{
    TaskHandle_t task;
    xTaskCreate(setup_task, "setup", 2000, NULL, 1, &task);
    vTaskCoreAffinitySet(task, CORE0);
    // Start the tasks and timer running. 
    vTaskStartScheduler();
    return 0;
}

