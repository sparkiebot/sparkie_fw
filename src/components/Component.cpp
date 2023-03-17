#include "Component.hpp"
#include "pico/stdlib.h"
#include <iostream>
#include "../hube_defs.hpp"

using namespace hubbie;

Component::Component(const char *name, UBaseType_t coreid, UBaseType_t priority)
{
    this->name = std::string(name);
    this->core = coreid;
    this->priority = priority;
}

Component::~Component()
{
    this->stop(HUBE_OK);
}


void Component::stop(hube_ret code)
{
    if(code != HUBE_OK)
        std::cerr 
            << "COMPONENT: '" << this->name << "' exited with code " 
            << (uint16_t) code << "\r\n";

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

TaskHandle_t Component::getTaskHandle()
{
    return this->xHandle;
}

bool Component::start()
{
    UBaseType_t res;

    std::cout << 
        "COMPONENT: Starting '" << this->name << "' task "
        "on core " << this->core << "...\r\n";

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

    return res == pdPASS;
}

void Component::vTask(void* params)
{
    Component *component = (Component*) params;
    
    if(component != NULL)
        component->run();

    component->stop(HUBE_OK);
}


UBaseType_t Component::getRunningCore()
{
    return sio_hw->cpuid;
}