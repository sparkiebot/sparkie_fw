#pragma once

#include <FreeRTOS.h>
#include <task.h>

typedef struct 
{
    bool idle;
    TaskHandle_t handle;
    std::string name;
    uint64_t			start_time;
    uint64_t			core_time[2];
} TaskRunTime_t;