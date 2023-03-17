#include "pico/stdlib.h"
#include <iostream>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>

#include <stdlib.h>

#include "hardware/i2c.h"

#include "components/AgentComponent.hpp"
#include "components/ImuComponent.hpp"
#include "hube_defs.hpp"

#define ICM20689_SLEEP(ms) vTaskDelay(ms / portTICK_PERIOD_MS)
#include <icm20689pico/icm20689pico.h>

#include "config.hpp"

#define TASK_PRIORITY		( tskIDLE_PRIORITY + 1UL )

#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

void setupTask(void* params)
{
    auto uros_agent = new hubbie::AgentComponent();
    
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    
    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    uros_agent->addComponent(new hubbie::ImuComponent());
    uros_agent->start();
    
    while(true)
    {
        
    }

    vTaskDelete(NULL);
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    std::cout << "Hello world!\r\n";

    TaskHandle_t task;
    xTaskCreate(setupTask, "setup", 500, NULL, TASK_PRIORITY, &task);
    vTaskCoreAffinitySet(task, CORE0);
    /* Start the tasks and timer running. */
    vTaskStartScheduler();

    return 0;
}