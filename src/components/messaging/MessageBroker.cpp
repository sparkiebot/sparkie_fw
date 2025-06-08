#include "MessageBroker.hpp"
#include "../logger/LoggerComponent.hpp"
#include <cstring>
#include <algorithm>

using namespace sparkie;

// Initialize static members
MessageBroker* MessageBroker::instance = nullptr;
bool MessageBroker::connected = false;
QueueHandle_t TinyUSBTransport::inQueue = nullptr;
QueueHandle_t TinyUSBTransport::outQueue = nullptr;

MessageBroker* MessageBroker::getInstance()
{
    if (instance == nullptr)
    {
        instance = new MessageBroker();
    }
    return instance;
}

MessageBroker::MessageBroker() : Component("MessageBroker", 0x01, tskIDLE_PRIORITY + 1)
{
    this->components = std::vector<MessageComponent*>();
    this->topic_subscribers = std::map<std::string, std::vector<Subscriber_t*>>();
    connected = false;
    
    // Create queue for handling incoming USB packets in the main task
    this->packetQueue = xQueueCreate(10, sizeof(Packet));
    
    // Initialize ping statistics
    this->pingStats = {0};
    this->pingStats.min_rtt = UINT32_MAX;
    this->pingSequence = 0;
    this->pingTaskHandle = nullptr;
}

MessageBroker::~MessageBroker()
{
    connected = false;
    vQueueDelete(this->packetQueue);
    
    // Clean up ping task if exists
    if (this->pingTaskHandle != nullptr) {
        vTaskDelete(this->pingTaskHandle);
    }
}

void MessageBroker::init()
{
    LoggerComponent::log(LogLevel::Info, "Initializing MessageBroker");
    
    // Initialize TinyUSB transport
    initTransport();
    
    // Create task for handling incoming USB packets
    TaskHandle_t usbTaskHandle;
    xTaskCreate(
        usbReceiveTask,
        "UsbReceive",
        2048,
        this,
        tskIDLE_PRIORITY + 1,
        &usbTaskHandle
    );
    vTaskCoreAffinitySet(usbTaskHandle, 0x01); // Set the task to run on core 0
    
    connected = true;
    
    // Initialize all registered components
    for (auto& component : components)
    {
        component->msgInit();
        
        // Register all subscriptions
        for (auto& sub : component->subscriptions)
        {
            registerSubscription(sub.topic, &sub);
        }
    }
    
    this->startPingTask();
    
    LoggerComponent::log(LogLevel::Info, "MessageBroker initialized successfully");
}

void MessageBroker::initTransport()
{
    // Initialize the queues if they don't exist
    if (TinyUSBTransport::inQueue == nullptr)
    {
        TinyUSBTransport::inQueue = xQueueCreate(10, sizeof(Packet));
    }
    
    if (TinyUSBTransport::outQueue == nullptr)
    {
        TinyUSBTransport::outQueue = xQueueCreate(10, sizeof(Packet));
    }
    
    // Start the TinyUSB transport
    usbTransport.start();
    
    LoggerComponent::log(LogLevel::Info, "TinyUSB transport initialized");
}

void MessageBroker::usbReceiveTask(void* pvParameters)
{
    MessageBroker* broker = static_cast<MessageBroker*>(pvParameters);
    Packet packet;
    
    while (true)
    {
        // Wait for an incoming packet from the USB transport
        if (xQueueReceive(TinyUSBTransport::inQueue, &packet, portMAX_DELAY) == pdTRUE)
        {
            // Forward the packet to the main task for processing
            xQueueSend(broker->packetQueue, &packet, portMAX_DELAY);
        }
    }
}

void MessageBroker::loop(TickType_t* xLastWakeTime)
{
    // Process incoming USB packets
    Packet packet;
    while (xQueueReceive(this->packetQueue, &packet, 0) == pdTRUE)
    {
        processIncomingPacket(packet);
    }
    
    // Process publisher queues
    processPublisherQueues();
    
    // Check for ping timeout
    if (checkPingTimeout() && connected) {
        LoggerComponent::log(LogLevel::Warning, "Connection timeout detected");
        // You might want to implement reconnection logic here
    }
}

void MessageBroker::run()
{
    this->init();
    
    this->running = true;
    
    auto xLastWakeTime = xTaskGetTickCount();
    
    while(this->running)
    {
        this->loop(&xLastWakeTime);
        vTaskDelay(pdMS_TO_TICKS(5)); // Prevent CPU hogging
    }
}


void MessageBroker::processIncomingPacket(const Packet& packet)
{
    // Temporary buffer for message data
    uint8_t data_buffer[256];
    std::string topic;
    uint8_t type_id;
    
    // Deserialize the packet
    size_t data_size = MessagePacketizer::deserializePacket(
        packet, topic, type_id, data_buffer, sizeof(data_buffer));
    
    if (data_size == 0)
    {
        LoggerComponent::log(LogLevel::Error, "Failed to deserialize packet");
        return;
    }
    
    // Check if this is a ping message
    if (topic == PING_TOPIC && type_id == PING_TYPE_ID)
    {
        handlePingMessage(data_buffer, data_size);
        return;
    }
    
    // Find subscribers for this topic
    if (topic_subscribers.find(topic) == topic_subscribers.end())
    {
        // No subscribers for this topic
        return;
    }
    
    // Create a temporary MessageType for type checking
    MessageType tempType;
    tempType.id = type_id;
    tempType.name = "";  // Name not needed for comparison
    
    // Distribute the message to subscribers
    for (auto& sub : topic_subscribers[topic])
    {
        // Check if message type matches
        if (sub->msg_type->id == type_id)
        {
            // Create a copy of the data for the subscriber
            void* data_copy = pvPortMalloc(data_size);
            if (data_copy == nullptr)
            {
                LoggerComponent::log(LogLevel::Error, "Failed to allocate memory for message");
                continue;
            }
            
            // Copy the data
            memcpy(data_copy, data_buffer, data_size);
            
            // Use the onMessage callback to queue the message
            MessageComponent::onMessage(data_copy, sub);
        }
    }
}

void MessageBroker::processPublisherQueues()
{
    for (auto& component : components)
    {
        for (uint32_t i = 0; i < component->publishers.size(); i++)
        {
            auto& pub = component->publishers[i];
            PubMsg_t msg;
            if (xQueueReceive(pub.pub_queue, &msg, 0) == pdTRUE)
            {
                // Calculate message size (this requires changes to MessageComponent::sendMessage)
                size_t msg_size = component->getLastMessageSize();
                
                // Distribute the message locally
                distributeMessage(pub.topic, msg.msg, pub.msg_type, msg_size);
                
                // Also send the message over USB
                sendMessageOverUSB(pub.topic, msg.msg, pub.msg_type->id, msg_size);
            }
        }
    }
}

void MessageBroker::distributeMessage(const std::string& topic, const void* data, const MessageType* type, size_t size)
{
    if (topic_subscribers.find(topic) == topic_subscribers.end())
    {
        // No subscribers for this topic
        return;
    }
    
    for (auto& sub : topic_subscribers[topic])
    {
        // Check if message type matches
        if (sub->msg_type->id == type->id)
        {
            // Create a copy of the data for each subscriber
            void* data_copy = pvPortMalloc(size);
            if (data_copy == nullptr)
            {
                LoggerComponent::log(LogLevel::Error, "Failed to allocate memory for message");
                continue;
            }
            
            // Copy the data
            memcpy(data_copy, data, size);
            
            // Use the onMessage callback to queue the message
            MessageComponent::onMessage(data_copy, sub);
        }
    }
}

void MessageBroker::sendMessageOverUSB(const std::string& topic, const void* data, uint8_t type_id, size_t size)
{
    // Serialize the message into packets
    std::vector<Packet> packets;
    uint8_t packet_count = MessagePacketizer::serializeMessage(topic, data, type_id, size, packets);
    
    // Send each packet over USB
    for (uint8_t i = 0; i < packet_count; i++)
    {
        xQueueSend(TinyUSBTransport::outQueue, &packets[i], portMAX_DELAY);
    }
}

void MessageBroker::registerComponent(MessageComponent* component)
{
    LoggerComponent::log(LogLevel::Info, "Registering component: " + component->name);
    components.push_back(component);
    
    if (connected)
    {
        // If broker is already running, initialize the component
        component->msgInit();
        
        // Register all subscriptions
        for (auto& sub : component->subscriptions)
        {
            registerSubscription(sub.topic, &sub);
        }
    }
}

void MessageBroker::unregisterComponent(MessageComponent* component)
{
    LoggerComponent::log(LogLevel::Info, "Unregistering component: " + component->name);
    
    // Remove component from the list
    auto it = std::find(components.begin(), components.end(), component);
    if (it != components.end())
    {
        components.erase(it);
    }
    
    // Remove all subscriptions
    for (auto& sub : component->subscriptions)
    {
        auto& subs = topic_subscribers[sub.topic];
        auto sub_it = std::find(subs.begin(), subs.end(), &sub);
        if (sub_it != subs.end())
        {
            subs.erase(sub_it);
        }
    }
    
    // Clean up component resources
    component->msgDestroy();
}

void MessageBroker::registerSubscription(const std::string& topic, Subscriber_t* sub)
{
    LoggerComponent::log(LogLevel::Info, "Registering subscription for topic: " + topic);
    
    // Create topic entry if it doesn't exist
    if (topic_subscribers.find(topic) == topic_subscribers.end())
    {
        topic_subscribers[topic] = std::vector<Subscriber_t*>();
    }
    
    // Add subscription to the topic
    topic_subscribers[topic].push_back(sub);
}

bool MessageBroker::isConnected()
{
    return connected;
}

void MessageBroker::sendPing()
{
    // Get current time
    uint32_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
    
    // Create ping message
    PingMessage ping;
    ping.timestamp = currentTime;
    ping.sequence = pingSequence++;
    ping.is_response = 0;
    
    // Send ping over USB
    sendMessageOverUSB(PING_TOPIC, &ping, PING_TYPE_ID, sizeof(PingMessage));
    
    // Update statistics
    pingStats.ping_count++;
    
    LoggerComponent::log(LogLevel::Debug, "Sent ping: seq=" + std::to_string(ping.sequence));
}

void MessageBroker::handlePingMessage(const void* data, size_t size)
{
    if (size != sizeof(PingMessage)) {
        LoggerComponent::log(LogLevel::Error, "Invalid ping message size");
        return;
    }
    
    const PingMessage* ping = static_cast<const PingMessage*>(data);
    
    if (ping->is_response == 0) {
        // This is a ping request, send response
        PingMessage response;
        response.timestamp = ping->timestamp;  // Keep original timestamp
        response.sequence = ping->sequence;
        response.is_response = 1;
        
        sendMessageOverUSB(PING_TOPIC, &response, PING_TYPE_ID, sizeof(PingMessage));
        
        LoggerComponent::log(LogLevel::Debug, "Replied to ping: seq=" + std::to_string(ping->sequence));
    } else {
        // This is a ping response
        uint32_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
        uint32_t rtt = currentTime - ping->timestamp;
        
        // Update ping statistics
        pingStats.last_ping_time = currentTime;
        pingStats.last_rtt = rtt;
        pingStats.min_rtt = std::min(pingStats.min_rtt, rtt);
        pingStats.max_rtt = std::max(pingStats.max_rtt, rtt);
        
        // Update average RTT
        if (pingStats.ping_count > 0) {
            pingStats.avg_rtt = ((pingStats.avg_rtt * (pingStats.ping_count - 1)) + rtt) / pingStats.ping_count;
        }
        
        // Reset timeout flag
        pingStats.timeout = false;
        
        LoggerComponent::log(LogLevel::Debug, "Received ping response: seq=" + std::to_string(ping->sequence) + 
                                             ", rtt=" + std::to_string(rtt) + "ms");
    }
}

void MessageBroker::startPingTask()
{
    xTaskCreate(
        pingTask,
        "PingTask",
        1024,
        this,
        tskIDLE_PRIORITY,
        &pingTaskHandle
    );
    vTaskCoreAffinitySet(pingTaskHandle, 0x01); // Set the task to run on core 0
    
    LoggerComponent::log(LogLevel::Info, "Started ping task");
}

void MessageBroker::pingTask(void* pvParameters)
{
    MessageBroker* broker = static_cast<MessageBroker*>(pvParameters);
    
    while (true) {
        if (connected) {
            broker->sendPing();
        }
        
        vTaskDelay(pdMS_TO_TICKS(PING_INTERVAL_MS));
    }
}

bool MessageBroker::checkPingTimeout()
{
    if (!connected) {
        return false;
    }
    
    uint32_t currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
    
    // If we haven't received a ping response for PING_TIMEOUT_MS, consider the connection timed out
    if (currentTime - pingStats.last_ping_time > PING_TIMEOUT_MS) {
        pingStats.timeout = true;
        pingStats.lost_pings++;
        return true;
    }
    
    return false;
}

const PingStats& MessageBroker::getPingStats() const
{
    return pingStats;
}

// MessagePacketizer implementation
uint8_t MessagePacketizer::serializeMessage(
    const std::string& topic, 
    const void* data, 
    uint8_t type_id, 
    size_t size,
    std::vector<Packet>& packets)
{
    // Calculate space needed for message header
    uint8_t topic_length = static_cast<uint8_t>(topic.length());
    uint16_t header_size = HEADER_MIN_SIZE + topic_length;
    
    // Calculate how many packets we need
    uint16_t data_per_packet = MAX_PAYLOAD_SIZE - header_size;
    uint8_t total_packets = (size + data_per_packet - 1) / data_per_packet;
    if (total_packets == 0) total_packets = 1;  // At least one packet even if no data
    
    // Prepare packets
    packets.clear();
    packets.reserve(total_packets);
    
    // Generate a packet ID (can use a static counter for simplicity)
    static uint8_t next_packet_id = 0;
    uint8_t packet_id = next_packet_id++;
    
    // Split data across packets
    size_t remaining_data = size;
    const uint8_t* data_ptr = static_cast<const uint8_t*>(data);
    
    for (uint8_t part = 0; part < total_packets; part++)
    {
        Packet packet;
        
        // Basic packet structure
        packet.start = PACKET_START;
        packet.id = packet_id;
        packet.num = part;
        packet.parts = total_packets;
        
        // Allocate memory for packet data
        uint8_t* payload = static_cast<uint8_t*>(pvPortMalloc(MAX_PAYLOAD_SIZE));
        if (payload == nullptr)
        {
            LoggerComponent::log(LogLevel::Error, "Failed to allocate memory for packet payload");
            return 0;
        }
        
        // Fill the header in the payload
        payload[0] = topic_length;
        payload[1] = type_id;
        memcpy(payload + 2, topic.c_str(), topic_length);
        
        // Calculate data size for this packet
        size_t data_in_packet = (remaining_data < data_per_packet) ? remaining_data : data_per_packet;
        
        // Copy data for this packet
        if (data_in_packet > 0 && data_ptr != nullptr)
        {
            memcpy(payload + header_size, data_ptr, data_in_packet);
            data_ptr += data_in_packet;
            remaining_data -= data_in_packet;
        }
        
        // Set packet length
        packet.len = header_size + data_in_packet;
        packet.data = payload;
        
        // Calculate CRC (simple checksum for example)
        packet.crc = 0;
        for (int i = 0; i < packet.len; i++)
        {
            packet.crc += payload[i];
        }
        
        packet.end = PACKET_END;
        
        // Add packet to the list
        packets.push_back(packet);
    }
    
    return total_packets;
}

size_t MessagePacketizer::deserializePacket(
    const Packet& packet, 
    std::string& topic, 
    uint8_t& type_id, 
    void* data_buffer, 
    size_t buffer_size)
{
    // Verify packet structure
    if (packet.start != PACKET_START || packet.end != PACKET_END)
    {
        return 0;
    }
    
    // Verify CRC
    uint8_t calculated_crc = 0;
    for (int i = 0; i < packet.len; i++)
    {
        calculated_crc += packet.data[i];
    }
    
    if (calculated_crc != packet.crc)
    {
        return 0;
    }
    
    // Extract header info
    if (packet.len < HEADER_MIN_SIZE)
    {
        return 0;
    }
    
    uint8_t topic_length = packet.data[0];
    type_id = packet.data[1];
    
    // Validate topic length
    if (packet.len < HEADER_MIN_SIZE + topic_length)
    {
        return 0;
    }
    
    // Extract topic
    topic.assign(reinterpret_cast<const char*>(packet.data + 2), topic_length);
    
    // Calculate data size
    size_t data_size = packet.len - (HEADER_MIN_SIZE + topic_length);
    
    // Check buffer size
    if (data_size > buffer_size)
    {
        return 0;
    }
    
    // Extract data
    if (data_size > 0)
    {
        memcpy(data_buffer, packet.data + HEADER_MIN_SIZE + topic_length, data_size);
    }
    
    return data_size;
}
