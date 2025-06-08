#pragma once

#include "../aht/AHTComponent.hpp"
#include "components/messaging/MessageComponent.hpp"
#include <queue.h>

namespace sparkie
{
    /**
     * AirQuality sensor component, that sends co2 ppm data and 
     * auto adjusts itself based on temperature and humidity data.
    */
    class AirQualityComponent : public MessageComponent
    {
    public:
        friend class AHTComponent;
        AirQualityComponent();
    protected:
        virtual void msgInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        /*
        Sensor commands
        */

        /**
         * Resets sensor to factory default application state.
        */
        void reset();

        uint8_t getDeviceID();

        /**
         * Changes sensor output frequency 
        */
        void setMeasMode(uint8_t mode);

        /**
         * It's true when an actual measurement has been done.
         * Otherwise the interrupt pin can be used.
        */
        bool isReady();

        /**
         * Returns the calculated ECO2 value measured in ppm.
        */
        void obtainECo2();

        /**
         * Affine algorithm calculations by sending the current ambient parameters.
         * @param temp expressed in C
         * @param hum  expressed in % (0.0f - 100.0f)
        */
        void setEnvData(float temp, float hum);
        
        // I2C read/write operations


        bool init_error;
        static QueueHandle_t aht_queue;
        AhtData aht_data;
        uint16_t co2_ppm;
    };    
} // namespace sparkie
