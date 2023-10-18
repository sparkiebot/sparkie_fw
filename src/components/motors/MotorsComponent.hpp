#pragma once

#include "../uros/URosComponent.hpp"
#include "../../config.hpp"
#include "../../misc/Pid.hpp"

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

    /**
     * @brief Abstraction of a real hardware motor
     * 
     * This class monitors and controls current motor speed.
     * A simple PID controller is used to keep the desired speed (in rpm).
    */
    class Motor
    {
    public:
        friend class MotorsComponent;

        /**
         * @brief Initializes pwm io and gpio io stuff.
         * 
         * It also enables one of the hall sensor pin interrupt so when signal goes either high or low, <br> 
         * Motor::onInterrupt function is called.
        */
        Motor(uint pin_pwm, uint pin_a, uint pin_b, uint enc_a, uint enc_b);

        /**
         * @brief changes PID goal rpm and resets current PID error values.
        */
        void setSpeed(double rpm);

        /**
         * @brief Calculates current motor rpm by checking motor pulses and updates PID rpm value
        */
        void update(double delta_time);

        /**
         * @brief Interrupt function called every time one of the hall effect sensors is changing signal output.
        */
        static void onInterrupt(uint gpio, uint32_t event_mask);
    private:

        /**
         * @brief Changes pwm duty cycle and turns motor pins on and off accordingly to rpm sign.  
        */
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

    /**
     * @brief Component for managing the two used motors.
     * 
     * It will send a message containing motor's velocities and also receives /cmd_vel messages.
    */
    class MotorsComponent : public URosComponent
    {
    public:
        friend class OdometryComponent;

        MotorsComponent();
        virtual uint8_t getHandlesNum();
        
        /**
         * @brief Searches for a motor that has a particular encoder pin associated with.
        */
        static Motor* getMotorFromEncoderPin(uint pin);

    protected:
        void rosInit();
    private:
        
        static void onVelMessage(URosComponent* component, const void* msg_in);
        
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
