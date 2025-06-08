#pragma once

#include "../messaging/MessageComponent.hpp"
#include "hw/gy85.hpp"

namespace sparkie
{
    /**
     * Message structure for IMU data
     */
    typedef struct _imu_data
    {
        // Orientation quaternion
        struct {
            float x;
            float y;
            float z;
            float w;
        } orientation;
        
        // Angular velocity in rad/s
        struct {
            float x;
            float y;
            float z;
        } angular_velocity;
        
        // Linear acceleration in m/s^2
        struct {
            float x;
            float y;
            float z;
        } linear_acceleration;
        
        // Timestamp in milliseconds
        uint32_t timestamp_ms;
    } ImuData;

    /**
     * Message structure for magnetic field data
     */
    typedef struct _magnetic_field_data
    {
        // Magnetic field in Tesla
        struct {
            float x;
            float y;
            float z;
        } magnetic_field;
        
        // Timestamp in milliseconds
        uint32_t timestamp_ms;
    } MagneticFieldData;

    class ImuComponent : public MessageComponent
    {
    public:
        ImuComponent();
    protected:
        virtual void msgInit();
    private:
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);

        gy85 imu;
        ImuData imu_data;
        MagneticFieldData mag_data;
    };
} // namespace sparkie
