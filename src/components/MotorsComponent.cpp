#include "MotorsComponent.hpp"
#include <iostream>
#include "../sparkie_defs.hpp"
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>
#include <cmath>
#include <numbers>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <algorithm>

#include "LoggerComponent.hpp"

using namespace sparkie;

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// Motor Class

Motor::Motor(uint pin_pwm, uint pin_a, uint pin_b, uint enc_a, uint enc_b) 
    : pid(MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD, -MOTOR_MAX_RPM, MOTOR_MAX_RPM )
{
    this->pin_a = pin_a;
    this->pin_b = pin_b;
    this->pin_pwm = pin_pwm;
    this->enc.pin_a = enc_a;
    this->enc.pin_b = enc_b;
    this->enc.pulses = 0;

    this->dir_change = false;
    this->rotation = 0.0;
    this->pid_rpm = 0.0;
    this->curr_speed = 0.0;
    this->goal_speed = 0.0;
    this->state = MotorState::STILL;

    gpio_init(this->pin_a);
    gpio_set_dir(this->pin_a, GPIO_OUT);

    gpio_init(this->pin_b);
    gpio_set_dir(this->pin_b, GPIO_OUT);
     
    gpio_init(this->enc.pin_a);
    gpio_set_dir(this->enc.pin_a, GPIO_IN);
    gpio_set_irq_enabled_with_callback(
        this->enc.pin_a, 
        GPIO_IRQ_EDGE_RISE, true, 
        Motor::onInterrupt
    );

    gpio_init(this->enc.pin_b);
    gpio_set_dir(this->enc.pin_b, GPIO_IN);

    gpio_set_function(this->pin_pwm, GPIO_FUNC_PWM);
    auto pwm_slice = pwm_gpio_to_slice_num(this->pin_pwm);
    auto pwm_config = pwm_get_default_config(); 
    pwm_init(pwm_slice, &pwm_config, true);

}

void Motor::setSpeed(double rpm)
{
    auto goal_rpm = clamp(rpm, -MOTOR_SOFT_MAX_RPM, MOTOR_SOFT_MAX_RPM);

    if(this->goal_speed == goal_rpm)
        return;
    
    this->goal_speed = goal_rpm;
    
    if(std::abs(goal_rpm) < MOTOR_MIN_RPM)
    {
        this->pid.reset();
        this->goal_speed = 0.0;
        this->state = MotorState::STILL;
    }
    else if(goal_rpm > 0 && this->state != MotorState::FORWARD)
    {
        if(this->curr_speed < 0)
        {
            this->pid.reset();
            this->dir_change = true;
        }

        this->state = MotorState::FORWARD;
    }
    else if(goal_rpm < 0 && this->state != MotorState::BACK)
    {
        if(this->curr_speed > 0)
        {
            this->pid.reset();
            this->dir_change = true;
        }

        this->state = MotorState::BACK;
    }
}

void Motor::setRawSpeed(double rpm)
{

    if(std::abs(rpm) < MOTOR_MIN_RPM)
    {
        gpio_put(this->pin_a, 0);
        gpio_put(this->pin_b, 0);
        return;
    }
    else if(rpm > 0)
    {
        gpio_put(this->pin_a, 1);
        gpio_put(this->pin_b, 0);
    }
    else if(rpm < 0)
    {
        gpio_put(this->pin_a, 0);
        gpio_put(this->pin_b, 1);
    }

    pwm_set_gpio_level(this->pin_pwm, std::abs(rpm) * this->pwm_per_rpm);
}

void Motor::update(double delta_time)
{
    
    // Calculate data
    auto revolutions = MOTOR_REDUCTION_RATE * (this->enc.pulses / MOTOR_PULSES_PER_REVOLUTIONS);

    this->curr_speed = revolutions / (60.0 * delta_time);
    
    if(this->rotation + revolutions >= 1)
    {
        this->rotation = revolutions;
    }
    else
    {
        this->rotation += revolutions;
    }

    this->enc.pulses = 0;
    
    if(this->goal_speed == 0)
    {
        this->setRawSpeed(0);
    }
    else
    {
        if(this->dir_change && this->curr_speed != 0)
        {
            this->pid_rpm = 0;
        }
        else
        {
            this->dir_change = false;
            this->pid_rpm = this->pid.compute(
                this->goal_speed, 
                this->curr_speed, delta_time);
        }

        this->setRawSpeed(this->pid_rpm);
    }

}

void Motor::onInterrupt(uint gpio, uint32_t event_mask)
{
    auto motor = MotorsComponent::getMotorFromEncoderPin(gpio);

    if(motor == nullptr)
        return;

    if(gpio_get(motor->enc.pin_b))
        motor->enc.pulses++;
    else
        motor->enc.pulses--;
            
}

// MotorsComponent Class

MotorsComponent* MotorsComponent::instance;

MotorsComponent::MotorsComponent() 
    : URosComponent("motors", CORE1, MOTORS_PRIORITY, UROS_MOTORS_RATE)
{
    instance = this;
}

void MotorsComponent::init()
{    
    this->lastUpdate = 0;
    
    gpio_init(MOTORS_ENABLE_PIN);
    gpio_set_dir(MOTORS_ENABLE_PIN, GPIO_OUT);
    gpio_put(MOTORS_ENABLE_PIN, GPIO_HIGH);

    // 0 - Left
    // 1 - Right
    this->motors.push_back(Motor(
        MOTOR_A_PWM_PIN, 
        MOTOR_A0_PIN, MOTOR_A1_PIN, 
        MOTOR_A_ENC0_PIN, MOTOR_A_ENC1_PIN
    ));

    this->motors.push_back(Motor(
        MOTOR_B_PWM_PIN, 
        MOTOR_B0_PIN, MOTOR_B1_PIN, 
        MOTOR_B_ENC0_PIN, MOTOR_B_ENC1_PIN
    ));
}

Motor* MotorsComponent::getMotorFromEncoderPin(uint pin)
{
    auto motors_comp = MotorsComponent::instance;

    for (size_t i = 0; i < MOTORS_NUM; i++)
    {
        if(motors_comp->motors[i].enc.pin_a == pin)
            return &motors_comp->motors[i];
    }

    return nullptr;
}

uint8_t MotorsComponent::getHandlesNum()
{
    return 1;
}

void MotorsComponent::rosInit()
{
    this->addSubscription(
        "/cmd_vel",
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        MotorsComponent::onVelMessage,
        &this->cmd_msg,
        false
    );
    
    this->addPublisher(
        "wheels/vel",
        ROSIDL_GET_MSG_TYPE_SUPPORT(irobot_create_msgs, msg, WheelVels)
    );

    this->wheel_vels_msg.header.frame_id =
        micro_ros_string_utilities_init(UROS_BASE_FRAME);
    
    this->wheel_vels_msg.velocity_left = 0;
    this->wheel_vels_msg.velocity_right = 0;
    
}

void MotorsComponent::onVelMessage(URosComponent* component, const void* msg_in)
{
    auto msg = (const geometry_msgs__msg__Twist*) msg_in;
    auto motors_comp = (MotorsComponent*) component;
    
    auto linear = msg->linear.x;
    auto angular = msg->angular.z;

    double left_speed = 
        ((2.0 * linear) - (angular * MOTORS_WHEEL_SEPARATION)) / (2.0 * MOTORS_WHEEL_RADIUS);

    double right_speed = 
        ((2.0 * linear) + (angular * MOTORS_WHEEL_SEPARATION)) / (2.0 * MOTORS_WHEEL_RADIUS);

    auto left_rpm = -left_speed / RPM_TO_RADS;
    auto right_rpm = right_speed / RPM_TO_RADS;
    
    motors_comp->motors[0].setSpeed(left_rpm);
    motors_comp->motors[1].setSpeed(right_rpm);
}

void MotorsComponent::loop(TickType_t* xLastWakeTime)
{
    auto current_time = to_ms_since_boot(get_absolute_time());
    if(this->lastUpdate != 0)
    {
        auto delta_time = (current_time - lastUpdate) / (1000.0);
        
        for (size_t i = 0; i < MOTORS_NUM; i++)
        {
            this->motors[i].update(delta_time);
        }        
    }

    this->lastUpdate = current_time;

    this->wheel_vels_msg.header.stamp.nanosec = (uint32_t) rmw_uros_epoch_nanos();
    this->wheel_vels_msg.header.stamp.sec = (int32_t) (rmw_uros_epoch_millis() / 1000);
    this->wheel_vels_msg.velocity_left = this->motors[0].curr_speed;
    this->wheel_vels_msg.velocity_right = this->motors[1].curr_speed;

    this->sendMessage(0, &this->wheel_vels_msg);
}

void MotorsComponent::safeStop()
{
    for (size_t i = 0; i < MOTORS_NUM; i++)
    {
        this->motors[i].setRawSpeed(0);
    }
}