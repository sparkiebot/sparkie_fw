#include "AgentComponent.hpp"
#include "../uros/URosComponent.hpp"
#include "../../config.hpp"
#include "../../sparkie_defs.hpp"

#include <pico/bootrom.h>

#include "../logger/LoggerComponent.hpp"
#include "../buzzer/BuzzerComponent.hpp"
#include "../led_strip/LedStripComponent.hpp"

#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/watchdog.h>
#include <iostream>

#include <hardware/i2c.h>

#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/error_handling.h>

#include "../../transports/pico_usb_transport.h"

using namespace sparkie;


sparkie::AgentComponent* sparkie::AgentComponent::instance = nullptr;

void blinkTask(TimerHandle_t timer)
{
    SPARKIE_VIS_DEBUG(200);
}

AgentComponent::AgentComponent() : Component("uros_agent", CORE0, AGENT_PRIORITY)
{
    instance = this;

    components = std::vector<URosComponent*>();

    this->initialized = false;
    this->connected = false;

    // Set custom transport
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_usb_transport_open,
        pico_usb_transport_close,
        pico_usb_transport_write,
        pico_usb_transport_read
    );
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

void AgentComponent::disconnect()
{
    AgentComponent::getInstance()->connected = false;
}

bool AgentComponent::isConnected()
{

    if (AgentComponent::instance == nullptr)
        return false;
    
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

    // Syncs time with the main agent
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

    /**
     * Executor needs to know the number of subscribers/services beforehand.
     * This is why every URosComponent is responsible of telling this information through a function.
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

    if(watchdog_caused_reboot())
        LoggerComponent::log(LogLevel::Debug, "Restarted by watchdog.");

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
            if(rmw_uros_ping_agent(1, 100) != RMW_RET_OK)
            {
                BuzzerComponent::play(BuzzerAction::ERROR);
                this->connected = false;
                continue;
            }

            rclc_executor_spin_some(&this->executor, RCL_MS_TO_NS(3));

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

        /**
         * Stop anything critical if connection crashes.
         */
        for (auto &&comp : this->components)
        {
            comp->safeStop();
        }

        xTimerStop(this->blinkTimer, 0);

        this->destroyConnection();
    } 
}

