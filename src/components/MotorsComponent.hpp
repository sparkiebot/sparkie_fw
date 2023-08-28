#pragma once

#include "URosComponent.hpp"
#include "../config.hpp"
#include "../misc/Pid.hpp"

#include <geometry_msgs/msg/twist.h>
#include <irobot_create_msgs/msg/wheel_vels.h>
#include <std_msgs/msg/bool.h>

#include <string>
#include <vector>


namespace sparkie
{
    class MotorsComponent;
    

    typedef struct 
    {
        uint pin_a;
        uint pin_b;
        int pulses;
    } MotorEncoder;
    
    enum MotorState
    {
        FORWARD,
        BACK,
        STILL
    };

    class Motor
    {
    public:
        friend class MotorsComponent;

        Motor(uint pin_pwm, uint pin_a, uint pin_b, uint enc_a, uint enc_b);
        MotorState getCurrentState();
        void setSpeed(double rpm);
        void update(double delta_time);
        static void onInterrupt(uint gpio, uint32_t event_mask);
    private:

        void setRawSpeed(double rpm);

        double rotation; // revolutions
        double curr_speed;
        bool dir_change;
        double goal_speed;
        double pid_rpm;

        const double pwm_per_rpm = UINT16_MAX / MOTOR_MAX_RPM;

        uint pin_pwm, pin_a, pin_b;
        MotorEncoder enc;

        Pid pid;
        MotorState state;
    };

    class MotorsComponent : public URosComponent
    {
    public:
        friend class OdometryComponent;

        MotorsComponent();
        virtual uint8_t getHandlesNum();
        
        void setState(bool state);
        static Motor* getMotorFromEncoderPin(uint pin);

    protected:
        void rosInit();

    private:
        
        static void onVelMessage(URosComponent* component, const void* msg_in);
        static void onStateMessage(URosComponent* component, const void* msg_in);

        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);
        virtual void safeStop();

        std::vector<Motor> motors;
        bool enabled;
        TickType_t lastUpdate;

        std_msgs__msg__Bool state_msg;
        geometry_msgs__msg__Twist cmd_msg;
        irobot_create_msgs__msg__WheelVels wheel_vels_msg;
        
        static MotorsComponent* instance;
    };    
} // namespace sparkie
