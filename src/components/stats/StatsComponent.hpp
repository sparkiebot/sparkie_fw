#pragma once

#include "../uros/URosComponent.hpp"
#include <map>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <diagnostic_msgs/msg/key_value.h>

namespace sparkie
{
    /**
     * @brief Component responsible of sending all tasks stats. (Optional)
     * If a particular macro definition is used, <br> 
     * StatsComponent will send cpu usage and memory usage data for every running component
    */
    class StatsComponent : public URosComponent
    {
    public:
        StatsComponent();   
        static std::vector<TaskRunTime_t> taskParams;    
    protected:
        virtual void rosInit();
    private:

        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        /**
         * Calculates resources usage by checking an array containing every task.
         * This array will have a fixed predefined size.
        */
        void calculateCPUData();
        void calculateMemData();

        diagnostic_msgs__msg__DiagnosticStatus cpu_msg;
        
        std::map<std::string, uint64_t> tasks_cpu_time;
        diagnostic_msgs__msg__KeyValue* cpu_values;

        diagnostic_msgs__msg__DiagnosticStatus memory_msg;
        diagnostic_msgs__msg__KeyValue* mem_values;
    };    
} // namespace sparkie
