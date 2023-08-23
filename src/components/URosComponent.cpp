#include "URosComponent.hpp"
#include <iostream>
#include "pico/stdlib.h"
#include "../config.hpp"
#include "../sparkie_defs.hpp"

#include "LoggerComponent.hpp"

using namespace sparkie;


URosComponent::URosComponent(
    const std::string& name, 
    UBaseType_t coreid, 
    UBaseType_t priority,
    float update_rate) : Component(name, coreid, priority)
{
    this->publishers = std::vector<UPub_t>();
    this->subscriptions = std::vector<USub_t>();
    this->update_rate = update_rate;
}

uint8_t URosComponent::getHandlesNum()
{
    return 0;
}

void URosComponent::addPublisher(
            const std::string& topic, 
            const rosidl_message_type_support_t* msg_type,
            bool best_effort
        )
{
    rcl_publisher_t* pub;
    pub = (rcl_publisher_t*) pvPortMalloc(sizeof(rcl_publisher_t));

    uint res;

    if(best_effort)
    {
        res = rclc_publisher_init_best_effort(
            pub,
            this->parent_node,
            msg_type,
            ("board/" + topic).c_str()
        );
    }
    else
    {
        res = rclc_publisher_init_default(
            pub,
            this->parent_node,
            msg_type,
            ("board/" + topic).c_str()
        );
    }
    

    if(res != RCL_RET_OK)
    {
        LoggerComponent::log(
            LogLevel::Error, 
            "Error creating publisher for " + this->name + ": " + std::to_string(res)
        );
    }

    UPub_t upub;
    upub.pub = pub;
    upub.pub_queue = xQueueCreate(1, sizeof(PubMsg_t));
    upub.msg.pub = pub;
    upub.msg.msg = nullptr;

    this->publishers.push_back(upub);
}

void URosComponent::addSubscription(
            const std::string& topic, 
            const rosidl_message_type_support_t* msg_type,
            USubCallback_t callback,
            void* msg,
            bool best_effort
        )
{
    rcl_subscription_t* sub;
    sub = (rcl_subscription_t*) pvPortMalloc(sizeof(rcl_subscription_t));

    std::string topic_name;

    if(topic.starts_with("/"))
    {
        topic_name = topic;
    }
    else
    {
        topic_name = ("board/" + topic);
    }

    uint res;

    if(best_effort)
        res = rclc_subscription_init_best_effort(
            sub,
            this->parent_node,
            msg_type,
            topic_name.c_str()
        );
    else
        res = rclc_subscription_init_default(
            sub,
            this->parent_node,
            msg_type,
            topic_name.c_str()
        );

    if(res != RCL_RET_OK)
    {
        LoggerComponent::log(
            LogLevel::Error, 
            "Error creating subscription for " + this->name + ": " + std::to_string(res)
        );
        return;
    }

    USub_t usub;
    usub.sub = sub;
    usub.sub_queue = xQueueCreate(1, sizeof(SubMsg_t));
    usub.callback = callback;

    this->subscriptions.push_back(usub);

    res = rclc_executor_add_subscription_with_context(
        &AgentComponent::getInstance()->executor,
        sub,
        msg,
        URosComponent::onMessage,
        &this->subscriptions.at(this->subscriptions.size() - 1),
        ON_NEW_DATA
    );

    if(res != RCL_RET_OK)
    {
        LoggerComponent::log(
            LogLevel::Error, 
            "Error adding subscription to exec for " + this->name + ": " + std::to_string(res)
        );
        return;
    }
}

/*
This function is executed by the rcl_executor (running on another task).
The obtained message is then sent to the subscriber's task via a queue. 
*/
void URosComponent::onMessage(const void * msg_in, void * context)
{
    auto usub = (USub_t*) context;
    usub->msg.data = msg_in;
    xQueueOverwrite(usub->sub_queue, &usub->msg);
}

/*
Messages are sent to the agent component via a single length queue for each pubblisher as 
publishing is not thread safe.
*/
void URosComponent::sendMessage(uint pub_index, const void* msg)
{
    if(!AgentComponent::isConnected())
        return;

    auto upub = this->publishers[pub_index];
    upub.msg.msg = msg;
    xQueueOverwrite(upub.pub_queue, &upub.msg);
}

/*
Messages are being pulled from a queue because also subscription handling is not thread safe.
Once a message is being received the corrisponding callback is invoked.
*/
void URosComponent::pollMessages()
{
    if(!AgentComponent::isConnected())
        return;

    for (auto &&sub : this->subscriptions)
    {
        SubMsg_t msg_data;
        if(xQueueReceive(sub.sub_queue, &msg_data, 0) == pdTRUE)
        {
            sub.callback(this, msg_data.data);
        }
    }
}

void URosComponent::rosDestroy()
{
    if(this->parent_node != NULL)
    {
        for (auto &&pub : this->publishers)
        {
            rcl_publisher_fini(pub.pub, this->parent_node);
            vPortFree(pub.pub);
            vQueueDelete(pub.pub_queue);
        }
        this->publishers.clear();

        for (auto &&sub : this->subscriptions)
        {
            rcl_subscription_fini(sub.sub, this->parent_node);
            vPortFree(sub.sub);
            vQueueDelete(sub.sub_queue);
        }
        this->subscriptions.clear();
    }
}


void URosComponent::run()
{
    this->init();

    this->running = true;

    auto xLastWakeTime = xTaskGetTickCount();
    
    while(this->running)
    {
        this->pollMessages();
        this->loop(&xLastWakeTime);
        if(this->update_rate != 0.0)
            xTaskDelayUntil(&xLastWakeTime, HZ_TO_MS(this->update_rate));
    }
}