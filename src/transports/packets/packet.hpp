#ifndef PACKETS
#define PACKETS

#include <FreeRTOS.h>
#include <queue.h>
#include <stdio.h>
#include <stdint.h>
#include <tuple>
#include <uxr/client/profile/transport/custom/custom_transport.h>

namespace sparkie 
{    
    struct Packet 
    {
        uint8_t start;
        uint8_t id;
        
        uint8_t num;
        uint8_t parts;
        
        uint8_t len;
        uint8_t* data; // Max size of packet payload will be 56 bytes
        
        uint8_t crc;
        uint8_t end;
    };
}

#endif // PACKETS
