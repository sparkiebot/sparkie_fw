#pragma once

#include "Component.hpp"
#include "AgentComponent.hpp"

#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rcl/service.h>
#include <vector>


namespace hubbie
{
    class URosComponent : public Component
    {
    public:
        URosComponent(const char *name, UBaseType_t coreid = 0x01, UBaseType_t priority = tskIDLE_PRIORITY);
        virtual void rosInit() = 0;
        const std::vector<rcl_publisher_t*>& getPublishers();
        const std::vector<rcl_subscription_t*>& getSubscription();
        const std::vector<rcl_service_t*>& getServices();
        virtual ~URosComponent();

        friend class AgentComponent;
    protected:
        std::vector<rcl_publisher_t*> publishers;
        std::vector<rcl_subscription_t*> subscriptions;
        std::vector<rcl_service_t*> services;

        rcl_node_t* parent_node;
    };
    

    
} // namespace hubbie
