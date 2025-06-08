#pragma once

#include <pico/stdlib.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>


#include <tusb.h>
#include "class/cdc/cdc_device.h"

#include "FreeRTOS.h"
#include "queue.h"

extern "C" {
    void tud_cdc_rx_cb(uint8_t itf);
}

namespace sparkie
{
    class TinyUSBTransport 
    {
        public:

            void start();
            
            static void task(void *pvParameters);
            static QueueHandle_t inQueue, outQueue;
        
    };
}
