#pragma once

#include "../messaging/MessageComponent.hpp"
#include <stdint.h>

namespace sparkie
{
    /**
     * Message structure for odometry data
     */
    typedef struct _odometry_data
    {
        // Position
        struct {
            float x;
            float y;
            float z;
        } position;
        
        // Orientation as quaternion
        struct {
            float x;
            float y;
            float z;
            float w;
        } orientation;
        
        // Linear velocity
        struct {
            float x;
            float y;
            float z;
        } linear_velocity;
        
        // Angular velocity
        struct {
            float x;
            float y;
            float z;
        } angular_velocity;
        
        // Timestamp in milliseconds
        uint32_t timestamp_ms;
    } OdometryData;

    /**
     * @brief Component that handles odometry calculations and publishes odometry messages. <br>
     */
    class OdometryComponent : public MessageComponent
    {
    public:
        OdometryComponent();
        virtual uint8_t getHandlesNum();

    protected:
        void msgInit();

    private:
        virtual void init();
        virtual void loop(TickType_t *xLastWakeTime);

        double x, y;
        double theta;
        double linear_velocity, angular_velocity;
        double raw_motor_data[2];

        OdometryData odom_data;
    };    
} // namespace sparkie
