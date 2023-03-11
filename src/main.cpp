#include "pico/stdlib.h"
#include <iostream>
#include <FreeRTOS.h>
#include <task.h>

#define CORE0 0x01
#define CORE1 0x02
#define CORE_UNDEFINED 0x03

#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

void helloTask(void* params)
{
    while(true)
    {
        std::cout << "Hello on Core " << sio_hw->cpuid << std::endl;
		vTaskDelay(500);
    }
}

void mainTask(void* params)
{
    TaskHandle_t task;
    xTaskCreate(helloTask, "HelloTask", 500, NULL, TASK_PRIORITY + 1, &task);
    vTaskCoreAffinitySet(task, CORE_UNDEFINED);


    while (true) { // Loop forever
		std::cout << "Main from Core " << sio_hw->cpuid << std::endl;
		vTaskDelay(2001);
	}
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    std::cout << "Hello world!\n";

    TaskHandle_t task;
    xTaskCreate(mainTask, "MainThread", 500, NULL, TASK_PRIORITY, &task);
    vTaskCoreAffinitySet(task, ( ( 1 << 0 ) | ( 1 << 2 ) ));
    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    return 0;
}