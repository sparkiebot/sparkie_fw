#pragma once

#include "../Component.hpp"
#include "../agent/AgentComponent.hpp"

#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rcl/service.h>

#include <memory>
#include <vector>

namespace sparkie
{
    /**
     * @brief Ros Publisher representation struct.
     *
     * Every component publisher has:
     * - Message Queue
     * - Message data
     * - Raw pointer to the ros publisher.
     *
     * Once a component publishes a message, data is sent to the AgentComponent through publisher queue.
     * AgentComponent then loops through all associated components' publishers and actually sending all the messages.
     * In this way there will be no problem with multiple cores writing on the same buffers.
     */
    typedef struct
    {
        QueueHandle_t pub_queue;
        PubMsg_t msg;
        rcl_publisher_t* pub; 
    } UPub_t;


    /**
     * @brief Callback function type invoked when a new message is received.
     * 
     * Example callback:
     * void callback(URosComponent* caller, const void* msg);
     * 
     * Message will presumably need to be casted with the correct type.
    */
    typedef void (* USubCallback_t)(URosComponent *, const void*);

     /**
     * @brief Ros Subscription representation struct
     * 
     * Every component subscription has: 
     * - Message Queue 
     * - Callback
     * - Message data
     * - A raw pointer to the ros Subscription.
     * 
     * 
     * Once AgentComponents receives a new message, URosComponent::onMessage is called, which is a raw ros callback. 
     * It will broadcast message to the correct subscription queue.
     * After that, at the same time particular URosComponent will look for new messages and run the correct callback.
    */
    typedef struct
    {
        QueueHandle_t sub_queue;
        USubCallback_t callback;
        SubMsg_t msg;
        rcl_subscription_t* sub; 
    } USub_t;


    /**
     * @brief Component with uros related features.
     * 
     * This component is meant to be used as a base for all components that interact with ros network.
    */
    class URosComponent : public Component
    {
    public:

        /**
         * @param name used for rtos task
         * @param coreid core identifier, CORE0=0x01, CORE1=0x02. It can also run on two core using 0x00 identifier.
         * @param priority avoid using same priorities for tasks running on the same core.
         * @param update_rate if set to 0, component will run as fast as possible.
        */
        URosComponent(
            const std::string& name, 
            UBaseType_t coreid = 0x01, 
            UBaseType_t priority = tskIDLE_PRIORITY,
            float update_rate = 0.0);
        
        // AgentComponent will be able to access private members of URosComponent
        friend class AgentComponent;

        /**
         * @brief It represents the number of subscribers which will be polling messages.
         * It is needed to change this value accordingly to the number of needed subscriptions.
        */
        virtual uint8_t getHandlesNum();
    protected:
        
        /**
         * @brief Initializes every ros structure needed by the component.
         * 
         * It is called once the AgentComponent successfully starts a session with main computer.
         * This method should contain only ros related instructions.
        */
        virtual void rosInit() = 0;

        /**
         * @brief Destroys every initialized ros struct.
         * 
         * This is automatically called when the AgentComponent stops the session with the main computer.
        */
        void rosDestroy();

        /**
         * @brief Creates and adds a new publisher to component.
         *
         * Adding a publisher, means not only initializing a ros struct representing it, <br>
         * but also creating a new queue in which every message will be added to.
         * See UPub_t for more informations.
         *
         * @param topic It will be relative to the main namespace unless it starts '/'
         * @param msg_type Message type can be obtained with the appropriate macro provided by ros library
         * @param best_effort if set to true, messages will be sent as best effort, so messages will be sent as quickly as possible, otherwise reliable but slower.
         */
        void addPublisher(
            const std::string &topic,
            const rosidl_message_type_support_t *msg_type,
            bool best_effort = true);

        /**
         * @brief Pushes a new message to the corresponding publisher queue.
         * 
         * Every message is not sent directly by the caller component's task; <br> 
         * A bridge queue between task and AgentComponent's task is used to correctly publish a message. <br>
         * Once sent via the queue, AgentComponent will free queue and send the message as quickly as possible <br>
         * 
         * @param pub_index index must be in the range of the total initialized publishers.
         * @param msg Message must be the same type as the publisher message type. 
        */
        void sendMessage(uint pub_index, const void* msg);
        
        /**
         * @brief Creates and adds a new subscription to component.
         * 
         * This method is very similar to URosComponent::addPublisher .
         * It also needs an already allocated ros message with the same type as msg_type <br> 
         * and a corresponding callback.
         * See USub_t for more informations.
         * 
         * @param topic It will be relative to the main namespace unless it starts '/'
         * @param msg_type Message type can be obtained with the appropriate macro provided by ros library
         * @param callback Callback function that will be called when a new message is received.
         * @param msg Allocated message with the same type as msg_type
         * @param best_effort if set to true, messages will be sent as best effort, so messages will be sent as quickly as possible, otherwise reliable but slower.
        */
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

        /**
         * @brief Looks up for new received messages and calls the appropriate callbacks
         * 
         * This method is called every time in URosComponent::update .
         * Also in this case, a queue architecture is used to prevent any multi core issues.
         * Thus, calling this method, will make it look for any new message in every subscription queues.
         * If an entry is found in queue, the corresponding callback is called.
         * See USub_t for a more detailed description of the architecture.  
        */
        void pollMessages();

        /**
         * @brief Raw callback for every subscription.
         * Meant to be used only by the AgentComponent.
        */
        static void onMessage(const void * msg_in, void * context);

        /**
         * @brief Initializes the component.
         * 
         * It is called once the component is started by the AgentComponent.
         * It should contain only non-ros related instructions.
        */
        virtual void init() = 0;
        
        /**
         * It should contain any code that needs to run forever at a specific rate.
        */
        virtual void loop(TickType_t* xLastWakeTime) = 0;
        
        /**
         * It firstly initializes the component, then starts an infinite loop in which <br>
         * message polling, any possible publishing is done and <br>
         * finally if a rate has been specified in constructor, component sleeps for that particular time period.
        */
        virtual void run();

        float update_rate;
    };
    

    
} // namespace sparkie
