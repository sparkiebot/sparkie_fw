#pragma once

#include <string>
#include <FreeRTOS.h>
#include "pico/stdlib.h"
#include <task.h>
#include <vector>
#include "../misc/TaskRunTime.h"

namespace sparkie
{
    /**
     * @brief Abstract representation of a freertos task. <br>
     * It includes some methods for handling current state.
     */
    class Component
    {
    public:
        /**
         * @param name used for rtos task
         * @param coreid core identifier, CORE0=0x01, CORE1=0x02. It can also run on two core using 0x00 identifier.
         * @param priority aoid using same priorities for tasks running on the same core.
        */
        Component(std::string_view name, UBaseType_t coreid = 0x01, UBaseType_t priority = tskIDLE_PRIORITY):name(name), core(coreid),priority(priority), running(false){};
        virtual ~Component(){ this->stop(); };

        /**
         * @brief task and adds its reference to the task stats list
        */
        virtual bool start();

        /**
         * @brief method is called when the task is stopped.
         */
        virtual void stop();
        
        /**
         * @brief method is responsible of safely disposing any critical component. <br>
         * For example, it is used in MotorsComponent to stop motors.
        */
        virtual void safeStop(){};

        /**
         * @brief Returns the rtos task handle.
        */
        TaskHandle_t getTaskHandle() const { return this->xHandle; };

        UBaseType_t getRunningCore() const { return sio_hw->cpuid; };

        const std::string& getName() const { return this->name; };
        
    protected:
    
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
