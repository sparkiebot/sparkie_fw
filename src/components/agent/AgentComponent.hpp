#pragma once

#include <memory>
#include <vector>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/service.h>
#include <rclc/publisher.h>
#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/u_int8.h>

#include "Component.hpp"
#include <semphr.h>
#include <timers.h>


#include <source_location>
namespace sparkie
{
    class URosComponent;
    class SystemComponent;
    class LoggerComponent;

    typedef struct _PubMsg 
    {
        rcl_publisher_t* pub; 
        const void* msg;
    } PubMsg_t;

    typedef struct _SubMsg 
    { 
        const void* data;
    } SubMsg_t;

    enum class ResetRequest
    {
        /* Only reboots board */
        Normal,
        /* Reboots in programming mode and needs to be interfaced with "picotool" */
        Prog,
        None
    };

    /**
     * 
     * A separate component is used to detect any new message so it doesn't block
     * the main agent component. <br>
     * This has been done as a serial communication is being used 
     * for the first iteration of "sparkie_board".
     * 
    */
    class URosExecutor : public Component
    {
        public:
            /** @brief Initializes a component only for receiving messages
             * 
             * @param executor Pointer to allocated and initialized rclc_executor
            */
            URosExecutor(rclc_executor_t* executor);
            virtual void run();
        private:
            rclc_executor_t* executor;
    };

    /**
     * @brief Task for communicating with main computer.
     * 
     * After waiting for the main computer agent, 
     * it initializes all the publishers and subscribers previously added. <br>
     * When actually runnning, it iterates through all URosComponent and 
     * checks if there is any new message to be pubblished.
     * It also handles any disconnection and always tries to reconnect.
    */
    class AgentComponent : public Component
    {
    public:
        AgentComponent();
        friend class URosComponent;
        friend class SystemComponent;
        
        /**
         * @brief Adds the component and assings a parent node to it.
         * @param component an allocated URosComponent 
        */
        void addComponent(URosComponent* component);
        
        /**
         * @brief Returns instance of the AgentComponent. <br>
         * Instance is valid only after the actual component is running. 
        */
        static AgentComponent* getInstance();
        
        static void sendResetRequest(ResetRequest request);
        
        
        static void disconnect();
        
        static bool isConnected();
    protected:
        
        virtual void run();
        
        PubMsg_t pub_msg;

    private:
        /**
         * @brief ResetRequest commands callback
        */
        static void system_service(const void * msg_in);
        
        /**
         * @brief Starts a new connection with the main agent <br>
         * 
         * Syncs internal time, initializes the main node and <br> 
         * all the pubblishers and subscriptions <br>
        */
        void initConnection();
        
        /**
         * Destroys current micro ros session and deinitialiazes all ros structures.
        */
        void destroyConnection();

        TimerHandle_t blinkTimer;

        PubMsg_t recv_msg;

        std::vector<URosComponent*> components;
        uint8_t handles_num;
        bool connected, initialized;

        ResetRequest request;

        rcl_node_t node;
        rcl_allocator_t allocator;
        rclc_support_t support;
        rclc_executor_t executor;

        rcl_subscription_t sys_service;
        std_msgs__msg__UInt8 sys_msg;

        LoggerComponent* logger;

        static AgentComponent* instance;
    };
} // namespace sparkie
