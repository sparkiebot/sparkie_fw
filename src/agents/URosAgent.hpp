#pragma once

#include <vector>

#include <rclc_lifecycle/rclc_lifecycle.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include "Agent.hpp"
#include "../components/URosComponent.hpp"

namespace hubbie
{
    class URosAgent : public Agent
    {
    public:
        URosAgent();
        virtual ~URosAgent();
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
    };
    

    
    
} // namespace hubbie
