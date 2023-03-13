#include "pico/stdlib.h"
#include <iostream>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>

#include "agents/URosAgent.hpp"
#include "hube_defs.hpp"

#include "config.hpp"

#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

void setupTask(void* params)
{
    hubbie::URosAgent uros_agent;
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    uros_agent.start(CORE0, 10);

    while(true) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_put(LED_PIN, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    std::cout << "Hello world!\r\n";

    TaskHandle_t task;
    xTaskCreate(setupTask, "MainTask", 500, NULL, TASK_PRIORITY, &task);
    vTaskCoreAffinitySet(task, CORE0);
    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    return 0;
}