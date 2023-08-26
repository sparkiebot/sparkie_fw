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
        Normal,
        Prog,
        None
    };

    class URosExecutor : public Component
    {
        public:
            URosExecutor(rclc_executor_t* executor);
            virtual void run();
        private:
            rclc_executor_t* executor;
    };



    class AgentComponent : public Component
    {
    public:
        AgentComponent();
        friend class URosComponent;
        friend class SystemComponent;
        void addComponent(URosComponent* component);
        static AgentComponent* getInstance();
        
        static void sendResetRequest(ResetRequest request);
        static void disconnect();
        static bool isConnected();
    protected:
        virtual void run();
        
        PubMsg_t pub_msg;

    private:
        static void system_service(const void * msg_in);
        void initSysService();

        void initConnection();
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
