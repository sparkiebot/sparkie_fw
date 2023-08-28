#pragma once

#include "URosComponent.hpp"
#include "DHTComponent.hpp"
#include <std_msgs/msg/u_int16.h>
#include <queue.h>

namespace sparkie
{
    class AirQualityComponent : public URosComponent
    {
    public:
        friend class DHTComponent;
        AirQualityComponent();
    protected:
        virtual void rosInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        void reset();
        uint8_t getDeviceID();
        void setMeasMode(uint8_t mode);
        bool isReady();
        uint16_t getECo2();
        void setEnvData(float temp, float hum);
        
    
        bool read_from(uint8_t reg, uint8_t* buff, uint8_t len);
        bool write_into(uint8_t reg, uint8_t* buff, uint8_t len);

        bool init_error;
        static QueueHandle_t dht_queue;
        DhtData dht_data;
        std_msgs__msg__UInt16 co2_msg;
    };    
} // namespace sparkie
