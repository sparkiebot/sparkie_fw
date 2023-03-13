#include "Agent.hpp"
#include "pico/stdlib.h"
#include <iostream>
#include "../hube_defs.hpp"

using namespace hubbie;

Agent::Agent(const char* name)
{
    this->name = std::string(name);
}

Agent::~Agent()
{
    this->stop(HUBE_OK);
}


void Agent::stop(hube_ret code)
{
    if(code != HUBE_OK)
        std::cerr 
            << "AGENT: '" << this->name << "' exited with code " 
            << (uint16_t) code << "\r\n";

    if(this->xHandle != NULL)
    {
        vTaskDelete(this->xHandle);
        this->xHandle = NULL;
    }
}

uint Agent::getStakHighWater()
{
    if(this->xHandle != NULL)
        return uxTaskGetStackHighWaterMark(this->xHandle);
    else
        return 0;
}

TaskHandle_t Agent::getTaskHandle()
{
    return this->xHandle;
}

bool Agent::start(uint8_t coreid, UBaseType_t priority)
{
    UBaseType_t res;

    std::cout << 
        "AGENT: Starting '" << this->name << "' task "
        "on core " << coreid << "...\r\n";

    res = xTaskCreate(
        Agent::vTask,
        this->name.c_str(),
        getMaxStackSize(),
        (void*) this,
        priority,
        &this->xHandle
    );

    if(this->xHandle != NULL)
        vTaskCoreAffinitySet(this->xHandle, coreid);

    return res == pdPASS;
}

void Agent::vTask(void* params)
{
    Agent *agent = (Agent*) params;
    
    if(agent != NULL)
        agent->run();

    agent->stop(HUBE_OK);
}


uint8_t Agent::getRunningCore()
{
    return (uint8_t) sio_hw->cpuid;
}