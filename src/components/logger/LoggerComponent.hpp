#pragma once

#include "../uros/URosComponent.hpp"
#include "../../config.hpp"

namespace sparkie
{

    enum LogLevel
    {
        Debug = 10,
        Info = 20,
        Warning = 30,
        Error = 40,
        Fatal = 50
    };

    class LoggerComponent : public URosComponent
    {
    public:
        friend class AgentComponent;
        
        LoggerComponent();

        static void log(LogLevel level, std::string msg, 
            const std::source_location& location = std::source_location::current());
    protected:
        void rosInit();
    private:
        /**
         * These task related functions are not really used as logger component will publish messages on request.
        */
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        rcl_interfaces__msg__Log log_msg;

        static LoggerComponent* instance;
    };    
} // namespace sparkie
