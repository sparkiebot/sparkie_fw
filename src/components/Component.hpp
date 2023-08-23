#pragma once

#include <string>
#include <FreeRTOS.h>
#include <task.h>
#include <vector>
#include "../misc/TaskRunTime.h"

namespace sparkie
{
    class Component
    {
    public:
        Component(std::string_view name, UBaseType_t coreid = 0x01, UBaseType_t priority = tskIDLE_PRIORITY);
        virtual ~Component();

        virtual bool start();
        virtual void stop();

        uint getStakHighWater();
        TaskHandle_t getTaskHandle();
        UBaseType_t getRunningCore();


        const std::string& getName();
        
    protected:
        virtual configSTACK_DEPTH_TYPE getMaxStackSize();
        static void vTask(void *params);
        virtual void run() = 0;

        TaskHandle_t xHandle = NULL;
        std::string name;
        bool running;
        UBaseType_t core;
        UBaseType_t priority;
    };
    
} // namespace sparkie
