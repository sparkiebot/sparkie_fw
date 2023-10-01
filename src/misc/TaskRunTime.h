#pragma once

#include <FreeRTOS.h>
#include <task.h>

/**
 * Struct used by StatsComponent to get all resources usages from tasks.
 * start_time variable is the last time task was been executed.
 * core_time is the time spent for task execution; the array length is 2 representing time for each core.
*/
typedef struct 
{
    bool idle;
    TaskHandle_t handle;
    std::string name;
    uint64_t			start_time;
    uint64_t			core_time[2];
} TaskRunTime_t;