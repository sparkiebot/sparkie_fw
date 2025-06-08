#pragma once

#include "../messaging/MessageComponent.hpp"
#include "../../config.hpp"
#include "../../misc/Pid.hpp"

#include <string>
#include <vector>

namespace sparkie
{
    /**
     * Twist message structure for velocity commands
     */
    typedef struct _twist_data
    {
        struct {
            float x;
            float y; 
            float z;
        } linear;
        
        struct {
            float x;
            float y;
            float z;
        } angular;
    } TwistData;

    /**
     * Wheel velocities message structure
     */
    typedef struct _wheel_vels_data
    {
        float velocity_left;
        float velocity_right;
        uint32_t timestamp_ms;
    } WheelVelsData;

    /**
     * Joint state message structure
     */
    typedef struct _joint_state_data
    {
        float position[2];    // Position in radians
        float velocity[2];    // Velocity in rad/s
        float effort[2];      // Effort
        uint32_t timestamp_ms;
    } JointStateData;

    class MotorsComponent;
    
    typedef struct 
    {
        uint pin_a;
        uint pin_b;
        volatile int pulses;
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
        Motor(uint pin_a, uint pin_b, uint enc_a, uint enc_b, bool left = true);

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
         * @brief Changes pwm duty cycle and turns motor pins on and off accordingly to directions.  
        */
        void setRawSpeed(uint16_t rpm);

        double curr_speed;
        bool dir_change;
        int dir;
        double goal_speed;
        double pid_pwm;

        const double pwm_per_rpm = UINT16_MAX / MOTOR_MAX_RPM;

        uint32_t wrap;
        uint pin_a, pin_b;
        MotorEncoder enc;

        Pid pid;
        MotorState state;
    };

    /**
     * @brief Component for managing the two used motors.
     * 
     * It will send a message containing motor's velocities and also receives /cmd_vel messages.
    */
    class MotorsComponent : public MessageComponent
    {
    public:

        MotorsComponent();
        virtual uint8_t getHandlesNum();
        
        /**
         * @brief Searches for a motor that has a particular encoder pin associated with.
        */
        static Motor* getMotorFromEncoderPin(uint pin);

        static QueueHandle_t getMotorsQueue();

    protected:
        void msgInit();
    private:
        
        static void onVelMessage(MessageComponent* component, const void* msg_in);
        
        virtual void init();
        virtual void loop(TickType_t* xLastWakeTime);
        virtual void safeStop();

        std::vector<Motor> motors;
        bool enabled;
        TickType_t lastUpdate;
        double raw_motor_data[2];

        TwistData cmd_data;
        WheelVelsData wheel_vels_data, goal_vels_data;
        JointStateData joint_state_data;
        
        static QueueHandle_t motors_queue;

        static MotorsComponent* instance;
    };    
} // namespace sparkie
