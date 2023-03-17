#include "AgentComponent.hpp"
#include "URosComponent.hpp"
#include "../config.hpp"
#include "../hube_defs.hpp"

#include <pico/stdlib.h>
#include <iostream>

#include <rmw_microros/rmw_microros.h>
#include <rcl/error_handling.h>
#include "../transport/pico_uart_transports.h"
#include <timers.h>


#define ROS_CHECK(ret) if(ret != RCL_RET_OK) stop(HUBE_UROS_GENERIC + 30 + ret)

using namespace hubbie;

configSTACK_DEPTH_TYPE AgentComponent::getMaxStackSize()
{
    return 1000;
}

AgentComponent::AgentComponent() : Component("uros", CORE0, 10)
{
    components = std::vector<URosComponent*>();
    exec_num = 0;
    running = true;
}

AgentComponent::~AgentComponent()
    {
        for (auto *component : this->components)
        {
            delete component;
        }
        this->components.clear();        

        this->stop(HUBE_OK);
    }

void AgentComponent::run()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
    
    auto ret = rmw_uros_ping_agent(UROS_PING_TIMEOUT, UROS_PING_ATTEMPTS);

    if(ret != RMW_RET_OK)
    {
        stop(HUBE_UROS_PING_TIMEOUT);
    }

    this->allocator = rcl_get_default_allocator();

    ROS_CHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));

    ROS_CHECK(rclc_node_init_default(
        &this->node, 
        UROS_NODE_NAME, UROS_NAMESPACE, 
        &this->support
    ));

    for (auto &&comp : this->components)
    {
        gpio_put(LED_PIN, 1);
        comp->rosInit();
        comp->start();
        vTaskDelay(200 / portTICK_PERIOD_MS);
        gpio_put(LED_PIN, 0);
    }
    

    if(this->exec_num > 0)
    {
        ROS_CHECK(rclc_executor_init(
            &this->executor, 
            &this->support.context, 
            this->exec_num, 
            &this->allocator
        ));
    }
        
    rmw_uros_sync_session(200);
    
    while(this->running)
    {
        if(this->exec_num > 0)
        {
            ROS_CHECK(rclc_executor_spin_some(&this->executor, 
                RCL_MS_TO_NS(UROS_DEFAULT_POLL_TIME)));
            vTaskDelay(UROS_DEFAULT_POLL_TIME / portTICK_PERIOD_MS);
        }
    }
    
    if(this->exec_num > 0)
        rclc_executor_fini(&this->executor);
    
    for (auto &&comp : this->components)
    {
        // Destroies all the micro ros components data...
        delete comp;
    }
    this->components.clear();

    rcl_node_fini(&this->node);
    rclc_support_fini(&this->support);
    
}

void AgentComponent::addComponent(URosComponent* component)
{
    component->parent_node = &this->node;
    this->components.push_back(component);
}