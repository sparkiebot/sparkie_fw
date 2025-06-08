#pragma once

#include "../Component.hpp"
#include <vector>
#include <string>
#include "../../transports/packets/packet.hpp"

namespace sparkie
{
    // Forward declaration to avoid circular dependency
    class MessageBroker;

    /**
     * @brief Message type ID structure
     * Replaces rosidl_message_type_support_t with a simpler structure
     */
    struct MessageType {
        const char* name;
        uint8_t id;
    };

    /**
     * @brief Message data structure for the publish/subscribe system
     */
    struct MessageData {
        const void* data;
        size_t size;
        uint8_t type_id;
    };

    /**
     * @brief Publisher message for the queue
     */
    typedef struct {
        void* pub;
        const void* msg;
    } PubMsg_t;

    /**
     * @brief Subscriber message for the queue
     */
    typedef struct {
        const void* data;
    } SubMsg_t;

    /**
     * @brief Publisher representation struct.
     *
     * Every component publisher has:
     * - Message Queue
     * - Message data
     * - Topic name and message type
     */
    typedef struct {
        QueueHandle_t pub_queue;
        PubMsg_t msg;
        std::string topic;
        const MessageType* msg_type;
    } Publisher_t;

    // Forward declaration
    class MessageComponent;

    /**
     * @brief Callback function type invoked when a new message is received.
     *
     * Example callback:
     * void callback(MessageComponent* caller, const void* msg);
     */
    typedef void (* MsgCallback_t)(MessageComponent*, const void*);

    /**
     * @brief Subscription representation struct
     *
     * Every component subscription has:
     * - Message Queue
     * - Callback
     * - Message data
     * - Topic name and message type
     */
    typedef struct {
        QueueHandle_t sub_queue;
        MsgCallback_t callback;
        SubMsg_t msg;
        std::string topic;
        const MessageType* msg_type;
    } Subscriber_t;

    /**
     * @brief Component with messaging features.
     *
     * This component is meant to be used as a base for all components that need to communicate via messages.
     */
    class MessageComponent : public Component
    {
    public:
        /**
         * @param name used for rtos task
         * @param coreid core identifier, CORE0=0x01, CORE1=0x02. It can also run on two core using 0x00 identifier.
         * @param priority avoid using same priorities for tasks running on the same core.
         * @param update_rate if set to 0, component will run as fast as possible.
         */
        MessageComponent(
            const std::string& name,
            UBaseType_t coreid = 0x01,
            UBaseType_t priority = tskIDLE_PRIORITY,
            float update_rate = 0.0);

        // MessageBroker will be able to access private members of MessageComponent
        friend class MessageBroker;

        /**
         * @brief Get the size of the last message sent
         * @return Size of the last message
         */
        size_t getLastMessageSize() const { return last_message_size; }

        /**
         * @brief It represents the number of subscribers which will be polling messages.
         * It is needed to change this value accordingly to the number of needed subscriptions.
         */
        virtual uint8_t getHandlesNum();

    protected:
        /**
         * @brief Initializes every messaging structure needed by the component.
         *
         * This method should contain only messaging related instructions.
         */
        virtual void msgInit() = 0;

        /**
         * @brief Destroys every initialized messaging struct.
         */
        void msgDestroy();

        /**
         * @brief Creates and adds a new publisher to component.
         *
         * @param topic The topic name to publish on
         * @param msg_type Message type structure
         * @param reliable If true, reliable delivery is guaranteed (slower), if false best effort delivery (faster)
         */
        void addPublisher(
            const std::string &topic,
            const MessageType *msg_type,
            bool reliable = false);

        /**
         * @brief Pushes a new message to the corresponding publisher queue.
         *
         * @param pub_index index must be in the range of the total initialized publishers.
         * @param msg Message must be the same type as the publisher message type.
         * @param size Size of the message in bytes
         */
        void sendMessage(uint pub_index, const void* msg, size_t size);

        /**
         * @brief Creates and adds a new subscription to component.
         *
         * @param topic Topic name to subscribe to
         * @param msg_type Message type structure
         * @param callback Callback function that will be called when a new message is received.
         * @param reliable If true, reliable delivery is guaranteed (slower), if false best effort delivery (faster)
         */
        void addSubscription(
            const std::string& topic,
            const MessageType* msg_type,
            MsgCallback_t callback,
            bool reliable = false
        );

        std::vector<Publisher_t> publishers;
        std::vector<Subscriber_t> subscriptions;

    private:
        /**
         * @brief Looks up for new received messages and calls the appropriate callbacks
         */
        void pollMessages();

        /**
         * @brief Raw callback for every subscription.
         * Meant to be used only by the MessageBroker.
         */
        static void onMessage(const void* msg_in, void* context);

        /**
         * @brief Initializes the component.
         */
        virtual void init() = 0;

        /**
         * @brief Contains any code that needs to run forever at a specific rate.
         */
        virtual void loop(TickType_t* xLastWakeTime) = 0;

        /**
         * @brief Runs the component's main loop.
         */
        virtual void run();

        float update_rate;
        size_t last_message_size;
    };
} // namespace sparkie
