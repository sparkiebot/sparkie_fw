#include "Component.hpp"
#include <vector>
#include "pico/stdlib.h"
#include <iostream>
#include "../sparkie_defs.hpp"

using namespace sparkie;

#define TASK_MAX_STACK_SIZE 2000

void Component::stop()
{
    this->safeStop();
    if(this->xHandle != NULL)
    {
        vTaskDelete(this->xHandle);
        this->xHandle = NULL;
    }
}


bool Component::start()
{
    UBaseType_t res;

    res = xTaskCreate(
        Component::vTask,
        this->name.c_str(),
        TASK_MAX_STACK_SIZE,
        (void *)this,
        priority,
        &this->xHandle);

    /**
     * If the task is created successfully, it will be assigned to the specified core.
     */
    if(this->xHandle != NULL)
        vTaskCoreAffinitySet(this->xHandle, this->core);

    if(res != pdPASS)
    {
        return false;
    }

    return true;
}

void Component::vTask(void* params)
{
    Component *component = (Component*) params;
    
    if(component != NULL)
        component->run();
}