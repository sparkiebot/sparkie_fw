#pragma once

#include <string>
#include <FreeRTOS.h>
#include <task.h>

namespace hubbie
{
    typedef uint8_t hube_ret;

    class Component
    {
    public:
        Component(const char *name, UBaseType_t coreid = 0x01, UBaseType_t priority = tskIDLE_PRIORITY);
        virtual ~Component();

        virtual bool start();
        virtual void stop(hube_ret code);

        virtual uint getStakHighWater();
        virtual TaskHandle_t getTaskHandle();
        virtual UBaseType_t getRunningCore();

    protected:
        static void vTask(void *params);
        virtual void run() = 0;

        virtual configSTACK_DEPTH_TYPE getMaxStackSize() = 0;

        // The task
        TaskHandle_t xHandle = NULL;

        std::string name;
        bool running;
        UBaseType_t core;
        UBaseType_t priority;
    };
    
} // namespace hubbie
