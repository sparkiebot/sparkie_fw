#include "tusb_transport.hpp"
#include "device/usbd.h"
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "transports/packets/packet.hpp"


void tud_cdc_rx_cb(uint8_t itf)
{
    if(itf != 1)
        return;
    
    uint8_t buf[64];
    
    uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));
    
    if (count > 0)
    {
        sparkie::Packet packet;
        memcpy(&packet, buf, sizeof(packet));
        xQueueSend(sparkie::TinyUSBTransport::inQueue, &packet, portMAX_DELAY);
    }
}


void sparkie::TinyUSBTransport::start()
{
    TaskHandle_t task_handle;
    // Start the transport task
    xTaskCreate(sparkie::TinyUSBTransport::task, "tusb", 2048, NULL, 1, &task_handle);
    
    vTaskCoreAffinitySet(task_handle, 0x01); // Set the task to run on core 0
}


void sparkie::TinyUSBTransport::task(void *pvParameters)
{
    Packet packet;
    
    while(true) 
    {
        tud_task();
        
        while(xQueueReceive(outQueue, &packet, 0) == pdTRUE)
        {
            tud_cdc_n_write(1, (const void*) &packet, sizeof(packet));
            tud_cdc_n_write_flush(1);
        }
    }
}