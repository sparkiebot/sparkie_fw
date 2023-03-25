#pragma once

#include <vector>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "Component.hpp"
#include <timers.h>

namespace hubbie
{
    class URosComponent;

    class AgentComponent : public Component
    {
    public:
        AgentComponent();
        virtual ~AgentComponent();
        void addComponent(URosComponent* component);
    protected:
        virtual void run();
        virtual configSTACK_DEPTH_TYPE getMaxStackSize();
    private:
        std::vector<URosComponent*> components;
        uint8_t exec_num;
        bool running;

        rcl_node_t node;
        rcl_allocator_t allocator;
        rclc_support_t support;
        rclc_executor_t executor;
        xTimerHandle pingTimer;
    };
} // namespace hubbie
