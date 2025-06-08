#pragma once

#include "../Component.hpp"
#include "MessageComponent.hpp"
#include "../../transports/tusb_transport.hpp"
#include "components/system/SystemComponent.hpp"
#include <map>
#include <vector>
#include <string>
#include <cstring> 

namespace sparkie
{
    // Forward declaration
    class MessageComponent; // Dichiarazione forward per risolvere la dipendenza circolare

    // Constants for ping messages
    static const std::string PING_TOPIC = "__ping";
    static const uint8_t PING_TYPE_ID = 0xFF;
    static const uint32_t PING_INTERVAL_MS = 1000;  // Send ping every 1 second
    static const uint32_t PING_TIMEOUT_MS = 5000;   // Connection timeout after 5 seconds

    // Ping message structure
    struct PingMessage {
        uint32_t timestamp;     // Timestamp when the ping was sent
        uint16_t sequence;      // Sequence number
        uint8_t is_response;    // 0 = request, 1 = response
    };

    // Ping statistics
    struct PingStats {
        uint32_t last_ping_time;    // Time of last successful ping
        uint32_t last_rtt;          // Last round-trip time in ms
        uint32_t min_rtt;           // Minimum round-trip time
        uint32_t max_rtt;           // Maximum round-trip time
        uint32_t avg_rtt;           // Average round-trip time
        uint16_t ping_count;        // Number of pings sent
        uint16_t lost_pings;        // Number of lost pings
        bool timeout;               // Connection timeout flag
    };
    
    /**
     * @brief Central message broker that handles message distribution
     *
     * This component acts as the central hub for all messaging components,
     * handling message routing between publishers and subscribers.
     * Messages are sent and received via TinyUSB.
     */
    class MessageBroker : public Component
    {
    public:
        /**
         * @brief Get the MessageBroker instance (singleton)
         */
        static MessageBroker* getInstance();

        /**
         * @brief Register a messaging component with the broker
         *
         * @param component The component to register
         */
        void registerComponent(MessageComponent* component);

        /**
         * @brief Unregister a messaging component from the broker
         *
         * @param component The component to unregister
         */
        void unregisterComponent(MessageComponent* component);

        /**
         * @brief Check if the messaging system is connected and ready
         */
        static bool isConnected();

        /**
         * @brief Initialize the USB transport
         */
        void initTransport();

        /**
         * @brief Process an incoming packet from TinyUSB
         *
         * @param packet The received packet
         */
        void processIncomingPacket(const Packet& packet);
        
        /**
        * @brief Get the current ping statistics
        * 
        * @return The current ping statistics
        */
        const PingStats& getPingStats() const;

    private:
        MessageBroker();
        ~MessageBroker();

        /**
         * @brief Initialize the broker
         */
        void init();

        /**
         * @brief Main loop for processing messages
         */
        void loop(TickType_t* xLastWakeTime);

        /**
         * @brief Run the main task
         *
         * This method is called by the FreeRTOS scheduler.
         */
        void run() override;  
        
        /**
         * @brief Process all queued messages from publishers
         */
        void processPublisherQueues();

        /**
         * @brief Distribute a message to all subscribers of a topic
         *
         * @param topic Topic to distribute to
         * @param data Message data
         * @param type Message type
         * @param size Size of the message data
         */
        void distributeMessage(const std::string& topic, const void* data, const MessageType* type, size_t size);

        /**
         * @brief Send a message to external systems via TinyUSB
         *
         * @param topic Topic name
         * @param data Message data
         * @param type_id Message type ID
         * @param size Size of the message data
         */
        void sendMessageOverUSB(const std::string& topic, const void* data, uint8_t type_id, size_t size);

        /**
         * @brief Register a subscription
         */
        void registerSubscription(const std::string& topic, Subscriber_t* sub);

        /**
         * @brief Task for handling incoming USB packets
         */
        static void usbReceiveTask(void* pvParameters);
        
        /**
        * @brief Send a ping request
        */
        void sendPing();

        /**
        * @brief Handle a ping message (request or response)
        * 
        * @param data The ping message data
        * @param size Size of the ping message
        */
        void handlePingMessage(const void* data, size_t size);

        /**
        * @brief Start the ping task
        */
        void startPingTask();

        /**
        * @brief Task for sending ping messages periodically
        */
        static void pingTask(void* pvParameters);

        /**
        * @brief Check if the connection has timed out
        * 
        * @return True if the connection has timed out
        */
        bool checkPingTimeout();

        std::vector<MessageComponent*> components;
        std::map<std::string, std::vector<Subscriber_t*>> topic_subscribers;
        TinyUSBTransport usbTransport;
        static MessageBroker* instance;
        static bool connected;

        // Queue for handling USB packet processing in the main task
        QueueHandle_t packetQueue;
        
        // Ping related members
        PingStats pingStats;
        uint16_t pingSequence;
        TaskHandle_t pingTaskHandle;
        static const MessageType PING_MSG_TYPE;
        
        friend class SystemComponent; // Allow SystemComponent to access private members
    };

    /**
     * @brief Helper class to serialize/deserialize messages to/from packets
     */
    class MessagePacketizer {
    public:
        /**
         * @brief Serialize a message into one or more packets
         *
         * @param topic Topic name
         * @param data Message data
         * @param type_id Message type ID
         * @param size Size of the message data
         * @param packets Vector to store the resulting packets
         * @return Number of packets created
         */
        static uint8_t serializeMessage(
            const std::string& topic,
            const void* data,
            uint8_t type_id,
            size_t size,
            std::vector<Packet>& packets);

        /**
         * @brief Deserialize a packet into topic, data and type information
         *
         * @param packet The packet to deserialize
         * @param topic Output: Topic name
         * @param type_id Output: Message type ID
         * @param data_buffer Output: Buffer to store the message data
         * @param buffer_size Size of the data buffer
         * @return Size of the deserialized data, 0 if failed
         */
        static size_t deserializePacket(
            const Packet& packet,
            std::string& topic,
            uint8_t& type_id,
            void* data_buffer,
            size_t buffer_size);

    private:
        // Packet constants
        static const uint8_t PACKET_START = 0xA5;
        static const uint8_t PACKET_END = 0x5A;
        static const uint8_t MAX_PAYLOAD_SIZE = 56;  // Max size that fits in a packet

        // Message header format inside packet payload:
        // - Topic length (1 byte)
        // - Type ID (1 byte)
        // - Topic name (variable length)
        // - Data (remaining bytes)
        static const uint8_t HEADER_MIN_SIZE = 2;  // Minimum bytes for topic length and type ID
    };
} // namespace sparkie
