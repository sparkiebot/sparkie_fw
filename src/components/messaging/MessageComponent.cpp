#include "MessageComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../logger/LoggerComponent.hpp"

using namespace sparkie;

MessageComponent::MessageComponent(
    const std::string& name,
    UBaseType_t coreid,
    UBaseType_t priority,
    float update_rate) : Component(name, coreid, priority)
{
    this->publishers = std::vector<Publisher_t>();
    this->subscriptions = std::vector<Subscriber_t>();
    this->update_rate = update_rate;
    this->last_message_size = 0;
}

uint8_t MessageComponent::getHandlesNum()
{
    return 0;
}

void MessageComponent::addPublisher(
            const std::string& topic,
            const MessageType* msg_type,
            bool reliable
        )
{
    Publisher_t pub;
    pub.pub_queue = xQueueCreate(1, sizeof(PubMsg_t));
    pub.msg.pub = nullptr;  // Will be set by the broker
    pub.msg.msg = nullptr;
    pub.topic = topic;
    pub.msg_type = msg_type;

    LoggerComponent::log(
        LogLevel::Info,
        "Created publisher for " + this->name + " on topic " + topic
    );

    this->publishers.push_back(pub);
}

void MessageComponent::addSubscription(
            const std::string& topic,
            const MessageType* msg_type,
            MsgCallback_t callback,
            bool reliable
        )
{
    if (this->subscriptions.size() >= this->getHandlesNum())
    {
        LoggerComponent::log(
            LogLevel::Error,
            "Error adding subscription to " + this->name + ": Max number of subscriptions reached");
        return;
    }

    Subscriber_t sub;
    sub.sub_queue = xQueueCreate(1, sizeof(SubMsg_t));
    sub.callback = callback;
    sub.topic = topic;
    sub.msg_type = msg_type;

    LoggerComponent::log(
        LogLevel::Info,
        "Created subscription for " + this->name + " on topic " + topic
    );

    this->subscriptions.push_back(sub);

    // The MessageBroker will register this subscription when the component is initialized
}

void MessageComponent::onMessage(const void* msg_in, void* context)
{
    auto sub = (Subscriber_t*) context;
    sub->msg.data = msg_in;
    xQueueOverwrite(sub->sub_queue, &sub->msg);
}

void MessageComponent::sendMessage(uint pub_index, const void* msg, size_t size)
{
    if (pub_index >= this->publishers.size())
    {
        LoggerComponent::log(
            LogLevel::Error,
            "Invalid publisher index: " + std::to_string(pub_index)
        );
        return;
    }

    auto pub = this->publishers[pub_index];
    pub.msg.msg = msg;

    // Store the message size for the broker to use
    this->last_message_size = size;

    xQueueOverwrite(pub.pub_queue, &pub.msg);
}

void MessageComponent::pollMessages()
{
    for (auto&& sub : this->subscriptions)
    {
        SubMsg_t msg_data;
        if (xQueueReceive(sub.sub_queue, &msg_data, 0) == pdTRUE)
        {
            sub.callback(this, msg_data.data);
        }
    }
}

void MessageComponent::msgDestroy()
{
    for (auto&& pub : this->publishers)
    {
        vQueueDelete(pub.pub_queue);
    }
    this->publishers.clear();

    for (auto&& sub : this->subscriptions)
    {
        vQueueDelete(sub.sub_queue);
    }
    this->subscriptions.clear();
}

void MessageComponent::run()
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
