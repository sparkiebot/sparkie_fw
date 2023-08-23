#include "Component.hpp"
#include "StatsComponent.hpp"
#include <vector>
#include "pico/stdlib.h"
#include <iostream>
#include "../sparkie_defs.hpp"

using namespace sparkie;

Component::Component(std::string_view name, UBaseType_t coreid, UBaseType_t priority)
{
    this->name = std::string(name);
    this->core = coreid;
    this->priority = priority;
    this->running = false;
}

Component::~Component()
{
    this->stop();
}

void Component::stop()
{
    if(this->xHandle != NULL)
    {
        vTaskDelete(this->xHandle);
        this->xHandle = NULL;
    }
}

uint Component::getStakHighWater()
{
    if(this->xHandle != NULL)
        return uxTaskGetStackHighWaterMark(this->xHandle);
    else
        return 0;
}

configSTACK_DEPTH_TYPE Component::getMaxStackSize()
{
    return 1000;
}

TaskHandle_t Component::getTaskHandle()
{
    return this->xHandle;
}

const std::string& Component::getName()
{
    return this->name;
}

bool Component::start()
{
    UBaseType_t res;

    res = xTaskCreate(
        Component::vTask,
        this->name.c_str(),
        getMaxStackSize(),
        (void*) this,
        priority,
        &this->xHandle
    );

    if(this->xHandle != NULL)
        vTaskCoreAffinitySet(this->xHandle, this->core);

    if(res != pdPASS)
    {
        return false;
    }

    TaskRunTime_t taskParam;

    /*
    One time operation that adds two idle tasks (one per core) used for calculating cpu usage
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

    component->stop();
}


UBaseType_t Component::getRunningCore()
{
    return sio_hw->cpuid;
}