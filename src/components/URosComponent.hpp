#pragma once

#include "Component.hpp"
#include "AgentComponent.hpp"

#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rcl/service.h>

#include <memory>
#include <vector>


namespace sparkie
{
    typedef struct upub
    {
        QueueHandle_t pub_queue;
        PubMsg_t msg;
        rcl_publisher_t* pub; 
    } UPub_t;

    typedef void (* USubCallback_t)(URosComponent *, const void*);

    typedef struct usub
    {
        QueueHandle_t sub_queue;
        USubCallback_t callback;
        SubMsg_t msg;
        rcl_subscription_t* sub; 
    } USub_t;

    class URosComponent : public Component
    {
    public:
        URosComponent(
            const std::string& name, 
            UBaseType_t coreid = 0x01, 
            UBaseType_t priority = tskIDLE_PRIORITY,
            float update_rate = 0.0);
        
        friend class AgentComponent;

        // It represents the number of subscribers which will be polling messages.
        virtual uint8_t getHandlesNum();
    protected:
        virtual void rosInit() = 0;
        void rosDestroy();
        void sendMessage(uint pub_index, const void* msg);
        
        void addPublisher(
            const std::string& topic, 
            const rosidl_message_type_support_t* msg_type,
            bool best_effort = true
        );

        void addSubscription(
            const std::string& topic, 
            const rosidl_message_type_support_t* msg_type,
            USubCallback_t callback,
            void* msg,
            bool best_effort = true
        );

        std::vector<UPub_t> publishers;
        std::vector<USub_t> subscriptions;
        rcl_node_t* parent_node;
    private:
        void pollMessages();
        static void onMessage(const void * msg_in, void * context);
        virtual void init() = 0;
        virtual void loop(TickType_t* xLastWakeTime) = 0;
        virtual void run();


        float update_rate;
    };
    

    
} // namespace sparkie
