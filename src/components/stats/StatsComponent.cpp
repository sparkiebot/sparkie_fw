#include "StatsComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../../config.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <malloc.h>
#include <pico/stdlib.h>

// Varibales used to determine the available free memory
extern "C" char __StackLimit;
extern "C" char __bss_end__;

using namespace sparkie;

std::vector<TaskRunTime_t> sparkie::StatsComponent::taskParams;

void taskCoreRunTime(int bOut)
{
	TaskHandle_t _hTask = xTaskGetCurrentTaskHandle();

    for (auto &&taskParam : StatsComponent::taskParams)
    {
        if ( _hTask == taskParam.handle)
		{
			if ( bOut ) taskParam.core_time[get_core_num()] += ( time_us_64() - taskParam.start_time);
			else taskParam.start_time = time_us_64(); 		
			break;
		}
    }
}

StatsComponent::StatsComponent() : URosComponent("stats", CORE1, STATS_PRIORITY, UROS_STATS_RATE)
{
}

void StatsComponent::init()
{
}

void StatsComponent::rosInit()
{

    this->addPublisher(
        "cpu",
        ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus)
    );

    this->addPublisher(
        "memory",
        ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus)
    );

    this->cpu_msg.name = micro_ros_string_utilities_init("Sparkie Board Cpu");
    this->cpu_msg.hardware_id = micro_ros_string_utilities_init("RP2040");
    this->cpu_msg.level = 0;
    this->cpu_msg.message = micro_ros_string_utilities_init("Stats");
    
    this->cpu_values = (diagnostic_msgs__msg__KeyValue*) 
        malloc(MAX_TASK_NUM * sizeof(diagnostic_msgs__msg__KeyValue));

    for (size_t i = 0; i < MAX_TASK_NUM; i++)
    {
        this->cpu_values[i].key = micro_ros_string_utilities_init("");
        this->cpu_values[i].value = micro_ros_string_utilities_init("");
    }
    
    this->cpu_msg.values.capacity = MAX_TASK_NUM;
    this->cpu_msg.values.size = 0;
    this->cpu_msg.values.data = this->cpu_values;

    this->memory_msg.name = micro_ros_string_utilities_init("Sparkie Board Memory");
    this->memory_msg.hardware_id = micro_ros_string_utilities_init("RP2040");
    this->memory_msg.level = 0;
    this->memory_msg.message = micro_ros_string_utilities_init("Stats");

    this->mem_values = (diagnostic_msgs__msg__KeyValue*) 
        malloc(4 * sizeof(diagnostic_msgs__msg__KeyValue));

    for (size_t i = 0; i < 4; i++)
    {
        this->mem_values[i].key = micro_ros_string_utilities_init("");
        this->mem_values[i].value = micro_ros_string_utilities_init("");
    }
    

    this->memory_msg.values.capacity = 4;
    this->memory_msg.values.size = 4;
    this->memory_msg.values.data = this->mem_values;
}

void setKeyValuef(diagnostic_msgs__msg__KeyValue* values, uint index, std::string key, float value)
{
    values[index].key = micro_ros_string_utilities_set(values[index].key, key.c_str());
    values[index].value = micro_ros_string_utilities_set(values[index].value, std::to_string(value).c_str());
}

void setKeyValue(diagnostic_msgs__msg__KeyValue* values, uint index, std::string key, int value)
{
    values[index].key = micro_ros_string_utilities_set(values[index].key, key.c_str());
    values[index].value = micro_ros_string_utilities_set(values[index].value, std::to_string(value).c_str());
}

float calculatePercentage(uint64_t a, uint64_t b)
{
    return 100.0f * a / b;
}

void StatsComponent::calculateCPUData()
{
    uint64_t total_run_time = 0;
    uint64_t idle_time = 0;

    this->tasks_cpu_time.clear();

    for (auto &&taskParam : StatsComponent::taskParams)
    {
        if(taskParam.idle)
            idle_time += taskParam.core_time[0] + taskParam.core_time[1];
        else
        {
            this->tasks_cpu_time.insert(std::make_pair(taskParam.name, taskParam.core_time[0] + taskParam.core_time[1]));
        }
        total_run_time += taskParam.core_time[0] + taskParam.core_time[1];
    }

    auto cpu_usage = 1.0f - (1.0f * idle_time / total_run_time);

    auto i = 0;
    auto size = 1;
    
    setKeyValuef(this->cpu_values, i, "Total Usage", 100.0f * cpu_usage);
    i++;

    for (auto &&pair : this->tasks_cpu_time)
    {
        setKeyValuef(this->cpu_values, i, pair.first + " task", calculatePercentage(pair.second, total_run_time));
        i++;
        size++;
    }

    this->cpu_msg.values.size = size;

}

void StatsComponent::calculateMemData()
{
    auto mem_info = mallinfo();
    auto used_heap = mem_info.uordblks;
    auto total_heap =  &__StackLimit  - &__bss_end__; 

    setKeyValuef(this->mem_values, 0, "Usage", calculatePercentage(used_heap, total_heap));
    setKeyValue(this->mem_values, 1, "Used", used_heap);
    setKeyValue(this->mem_values, 2, "Free", total_heap - used_heap);
    setKeyValue(this->mem_values, 3, "Total", total_heap);
}

void StatsComponent::loop(TickType_t* xLastWakeTime)
{
    if(AgentComponent::isConnected())
    {
        this->calculateCPUData();
        this->calculateMemData();
    }

 
    this->sendMessage(0, &this->cpu_msg);
    this->sendMessage(1, &this->memory_msg);
}