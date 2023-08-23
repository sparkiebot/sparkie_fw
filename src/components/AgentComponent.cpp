#include "AgentComponent.hpp"
#include "URosComponent.hpp"
#include "../config.hpp"
#include "../sparkie_defs.hpp"

#include <pico/bootrom.h>

#include "LoggerComponent.hpp"
#include "BuzzerComponent.hpp"
#include "LedStripComponent.hpp"

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/watchdog.h>
#include <iostream>
#include <format>

#include <hardware/i2c.h>

#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/error_handling.h>

#ifdef UROS_UART
    #include "../transports/pico_uart_transport.h"
#else
    #ifdef UROS_USB
        #include "../transports/pico_usb_transport.h"
    #endif
#endif

using namespace sparkie;


sparkie::AgentComponent* sparkie::AgentComponent::instance;

void blinkTask(TimerHandle_t timer)
{
    SPARKIE_VIS_DEBUG(200);
}

// See https://forums.raspberrypi.com/viewtopic.php?t=326333
static void wait_and_reboot()
{
#if configUSE_IDLE_HOOK				// if there's an IDLE hook, it's because it is used to kick the watchdog
	__asm volatile ( " cpsid i " );
	while ( true )  {};				// reboot should happen here when watchdog times out
#else
	watchdog_reboot(0,0,25);
#endif

}

AgentComponent::AgentComponent() : Component("uros", CORE0, AGENT_PRIORITY)
{
    instance = this;

    components = std::vector<URosComponent*>();

    this->initialized = false;
    this->connected = false;

    #ifdef UROS_UART
        rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_uart_transport_open,
            pico_uart_transport_close,
            pico_uart_transport_write,
            pico_uart_transport_read
        );
    #else
    #ifdef UROS_USB
        rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_usb_transport_open,
            pico_usb_transport_close,
            pico_usb_transport_write,
            pico_usb_transport_read
        );
    #endif
    #endif
}

AgentComponent* AgentComponent::getInstance()
{
    return AgentComponent::instance;
}

void AgentComponent::sendResetRequest(ResetRequest request)
{
    instance->request = request;
    instance->connected = false;
    LoggerComponent::log(LogLevel::Info, "Received reset request...");
}

bool AgentComponent::isConnected()
{
    return AgentComponent::getInstance()->connected;
}

void AgentComponent::system_service(const void * msg_in)
{
    auto msg = (const std_msgs__msg__UInt8*)msg_in;

    if(msg->data > 1)
        return;

    AgentComponent::sendResetRequest((ResetRequest)(msg->data));
}

void AgentComponent::initConnection()
{
    gpio_put(LED_PIN, 1);

    this->request = ResetRequest::None;

    auto ret = rmw_uros_ping_agent(UROS_PING_TIMEOUT, UROS_PING_ATTEMPTS);
    
    if(ret != RMW_RET_OK)
    {
        return;
    }

    sparkie::BuzzerComponent::play(sparkie::CONNECTED);
    
    this->allocator = rcl_get_default_allocator();

    ROS_CHECK(rclc_support_init(&this->support, 0, NULL, &this->allocator));

    ROS_CHECK(rclc_node_init_default(
        &this->node, 
        UROS_NODE_NAME, UROS_NAMESPACE, 
        &this->support
    ));

    rmw_uros_sync_session(200);

    // Init logger
    logger = new LoggerComponent();
    logger->parent_node = &this->node;
    logger->rosInit();

    this->handles_num = 0;
    for (auto &&comp : this->components)
    {
        this->handles_num += comp->getHandlesNum();
    }

    ROS_CHECK(
        rclc_subscription_init_default(
            &this->sys_service,
            &this->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
            "board/system"
        )
    );
    this->handles_num++;

    /*
    Executor needs to know the number of  subscribers/services beforehand.
    This is why every URosComponent is responsible of telling this information through a function.
    */
    if(this->handles_num > 0)
        ROS_CHECK(
            rclc_executor_init(
                &this->executor,
                &this->support.context,
                this->handles_num,
                &this->allocator
            )
        );

    ROS_CHECK(
        rclc_executor_add_subscription(
            &this->executor,
            &this->sys_service,
            &this->sys_msg,
            AgentComponent::system_service,
            ON_NEW_DATA
        )
    );

    LoggerComponent::log(LogLevel::Info, "Starting..");

    for (auto &&comp : this->components)
    {
        LoggerComponent::log(LogLevel::Debug, comp->name + " initializing..");

        comp->rosInit();
        
        LoggerComponent::log(LogLevel::Info, comp->name + " initialized.");
    }

    gpio_put(LED_PIN, 0);

    LoggerComponent::log(LogLevel::Info, "Started successfully.");

    this->connected = true;
    this->initialized = true;
}

void AgentComponent::destroyConnection()
{
    if(!this->initialized)
        return;

    LoggerComponent::log(LogLevel::Info, "Closing connection...");

    AgentComponent::connected = false;

    for (auto &&comp : this->components)
    {
        comp->rosDestroy();
    }
    
    rcl_node_fini(&this->node);
    rclc_support_fini(&this->support);

    auto rmw_context = rcl_context_get_rmw_context(&this->support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    
    this->initialized = false;
}

void AgentComponent::addComponent(URosComponent* component)
{
    component->parent_node = (&this->node);
    this->components.push_back(component);
}

void AgentComponent::run()
{

    this->blinkTimer = xTimerCreate(
        "uros_blink", pdMS_TO_TICKS(1000 * 2), 
        true, NULL,
        blinkTask
    );

    while (true)
    {
        LedStripComponent::getInstance()->setMode(LedStripMode::StartingUp);

        this->initConnection();

        LedStripComponent::getInstance()->setMode(LedStripMode::Off);

        xTimerStart(this->blinkTimer, 0);

        auto lastWakeTime = xTaskGetTickCount();

        while(this->connected)
        {
            if(rmw_uros_ping_agent(200, 10) != RMW_RET_OK)
            {
                sparkie::BuzzerComponent::play(sparkie::ERROR);
                break;
            }
            
            if(this->handles_num > 0)
                rclc_executor_spin_some(&this->executor, RCL_MS_TO_NS(1));
            
            for (auto &&comp : this->components)
            {
                for (auto &&pub : comp->publishers)
                {
                    if(xQueueReceive(pub.pub_queue, &this->recv_msg, 0) == pdTRUE)
                    {
                        rcl_publish(this->recv_msg.pub, this->recv_msg.msg, NULL);
                    }
                }
            }

            xTaskDelayUntil(&lastWakeTime, HZ_TO_MS(AGENT_UPDATE_RATE));
        }

        xTimerStop(this->blinkTimer, 0);

        this->destroyConnection();

        if(this->request != ResetRequest::None)
        {
            if(this->request == ResetRequest::Normal)
            {
                watchdog_reboot(0, 0, 0);
            }
            else
            {
                // It goes into programming mode and also stops the other core.
                // See https://forums.raspberrypi.com/viewtopic.php?t=326333
                vTaskSuspendAll();
                taskENTER_CRITICAL();
                multicore_reset_core1();
                multicore_launch_core1(wait_and_reboot);
                reset_usb_boot(0,0);
                taskEXIT_CRITICAL();	// should never get here!!!!
                xTaskResumeAll();
            }

            break;
        }
    } 
}

