#pragma once

#include "../messaging/MessageComponent.hpp"
#include "hw/aht.hpp"

namespace sparkie
{
    typedef struct _aht_data
    {
        float temp;
        float hum;
    } AhtData;
    
    /**
     * AHT Message
    */
    

    /**
     * Component for publishing temperature and humidity data.
     */
    class AHTComponent : public MessageComponent
    {
    public:
        AHTComponent();

    protected:
        virtual void rosInit();

    private:
        virtual void init();
        virtual void loop(TickType_t *xLastWakeTime);

        AHT sensor;
        AhtData aht_data;
        float temp, hum;
    };
} // namespace sparkie
