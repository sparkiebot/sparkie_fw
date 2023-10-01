#pragma once

#include <string>
#include <FreeRTOS.h>
#include <task.h>
#include <vector>
#include "../misc/TaskRunTime.h"

namespace sparkie
{
    /**
     * @brief Abstract rapresentation of a freertos task.
     * 
     * It includes some methods for handling current state. <br>
    */
    class Component
    {
    public:
        /**
         * @param name name used for rtos task
         * @param coreid Core identifier, CORE0=0x01, CORE1=0x02. Task can also be run on two core in parallel using 0x00
         * @param priority Avoid using same priorities for tasks running on the same core.
        */
        Component(std::string_view name, UBaseType_t coreid = 0x01, UBaseType_t priority = tskIDLE_PRIORITY);
        virtual ~Component();

        /**
         * Starts task and adds its reference to the task stats list
        */
        virtual bool start();

        virtual void stop();
        
        /**
         * This method is responsible of safely disposing any critical component. <br>
         * For example, it is used in MotorsComponent to stop motors.
        */
        virtual void safeStop();

        uint getStakHighWater();
        TaskHandle_t getTaskHandle();
        UBaseType_t getRunningCore();


        const std::string& getName();
        
    protected:
    
        virtual configSTACK_DEPTH_TYPE getMaxStackSize();
        
        /**
         * Actual freertos task code.
        */
        static void vTask(void *params);
        
        virtual void run() = 0;
        

        TaskHandle_t xHandle = NULL;
        std::string name;
        bool running;
        UBaseType_t core;
        UBaseType_t priority;
    };
    
} // namespace sparkie
