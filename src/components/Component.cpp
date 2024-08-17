#include "Component.hpp"
#include "stats/StatsComponent.hpp"
#include <vector>
#include "pico/stdlib.h"
#include <iostream>
#include "../sparkie_defs.hpp"

using namespace sparkie;

#define TASK_MAX_STACK_SIZE 1000

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

    TaskRunTime_t taskParam;

    /**
     * One time operation that adds two idle tasks (one per core) used to calculate cpu usage
     */

    if(StatsComponent::taskParams.empty())
    {
        auto taskHandles = xTaskGetIdleTaskHandle();
        for (size_t i = 0; i < configNUM_CORES; i++)
        {
            taskParam.name = "IDLE" + std::to_string(i);
            taskParam.handle = taskHandles[i];
            taskParam.core_time[0] = 0;
            taskParam.core_time[1] = 0;
            taskParam.start_time = 0;    
            taskParam.idle = true;
            StatsComponent::taskParams.push_back(taskParam);
        }
    }

    /**
     * Adds the current task to the list of tasks to be monitored
     */
    taskParam.name = this->name;
    taskParam.handle = this->xHandle;
    taskParam.core_time[0] = 0;
    taskParam.core_time[1] = 0;
    taskParam.start_time = 0;  
    taskParam.idle = false;  
    StatsComponent::taskParams.push_back(taskParam);

    return true;
}

void Component::vTask(void* params)
{
    Component *component = (Component*) params;
    
    if(component != NULL)
        component->run();
}