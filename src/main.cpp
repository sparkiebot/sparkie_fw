#include "pico/stdlib.h"
#include <iostream>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>

#include <stdlib.h>


#include "components/AgentComponent.hpp"
#include "components/ImuComponent.hpp"
#include "components/BuzzerComponent.hpp"
#include "hube_defs.hpp"

#define ICM20689_SLEEP(ms) vTaskDelay(ms / portTICK_PERIOD_MS)
#include <icm20689pico/icm20689pico.h>

#include "config.hpp"

#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

void setupTask(void* params)
{
    auto uros_agent = new hubbie::AgentComponent();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    hubbie::BuzzerComponent::init();
    hubbie::BuzzerComponent::play(hubbie::PLAY_STARTUP);
    
    uros_agent->addComponent(new hubbie::ImuComponent());
    uros_agent->start();

    vTaskSuspend(NULL);
}

int main()
{
    stdio_init_all();
    std::cout << "Hello world!\r\n";

    TaskHandle_t task;
    xTaskCreate(setupTask, "setup", 1000, NULL, TASK_PRIORITY, &task);
    vTaskCoreAffinitySet(task, CORE0);
    /* Start the tasks and timer running. */
    vTaskStartScheduler();
    return 0;
}