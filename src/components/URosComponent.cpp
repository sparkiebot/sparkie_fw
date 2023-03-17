#include "URosComponent.hpp"

using namespace hubbie;


URosComponent::URosComponent(
    const char *name, 
    UBaseType_t coreid, 
    UBaseType_t priority) : Component(name, coreid, priority)
{
    this->publishers = std::vector<rcl_publisher_t*>();
    this->subscriptions = std::vector<rcl_subscription_t*>();
    this->services = std::vector<rcl_service_t*>();
}

URosComponent::~URosComponent()
{
    if(this->parent_node != NULL)
    {
        for (auto &&pub : this->publishers)
        {
            rcl_publisher_fini(pub, this->parent_node);
        }
        this->publishers.clear();

        for (auto &&sub : this->subscriptions)
        {
            rcl_subscription_fini(sub, this->parent_node);
        }
        this->subscriptions.clear();

        for (auto &&srv : this->services)
        {
            rcl_service_fini(srv, this->parent_node);
        }
        this->services.clear();
    }
}