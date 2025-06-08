#pragma once


#include <string>

namespace sparkie 
{

    class Message
    {
    public:
        Message(const std::string& topic);
        ~Message();
        
        std::string& getTopic();
        long getTimestampNs();

    private:
        std::string topic;
        long timestamp_ns;
    };
    
}