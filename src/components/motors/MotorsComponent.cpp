#include "MotorsComponent.hpp"
#include <iostream>
#include "../../sparkie_defs.hpp"
#include <cmath>
#include <numbers>
#include <pico/stdlib.h>
#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <algorithm>
#include "../buzzer/BuzzerComponent.hpp"
#include "../logger/LoggerComponent.hpp"
#include "../messaging/MessageBroker.hpp"

#define TOP_MAX 65534

using namespace sparkie;

// Define message types
const MessageType MSG_CMD_VEL = {"cmd_vel", 0};
const MessageType MSG_WHEEL_VELS = {"wheels/vel", 1};
const MessageType MSG_WHEEL_GOAL = {"wheels/goal", 2};
const MessageType MSG_JOINT_STATES = {"wheels/joint_states", 3};

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// PWM Functions

uint32_t pwm_set_frequency(uint pin, uint freq)
{
    auto slice_num = pwm_gpio_to_slice_num(pin);
    uint32_t source_hz = clock_get_hz(clk_sys);
    uint32_t div16_top = 16 * source_hz / freq;
    uint32_t top = 1;
    while (1)
    {
        // Try a few small prime factors to get close to the desired frequency.
        if (div16_top >= 16 * 5 && div16_top % 5 == 0 && top * 5 <= TOP_MAX)
        {
            div16_top /= 5;
            top *= 5;
        }
        else if (div16_top >= 16 * 3 && div16_top % 3 == 0 && top * 3 <= TOP_MAX)
        {
            div16_top /= 3;
            top *= 3;
        }
        else if (div16_top >= 16 * 2 && top * 2 <= TOP_MAX)
        {
            div16_top /= 2;
            top *= 2;
        }
        else
        {
            break;
        }
    }
    if (div16_top < 16)
    {
        return 0;
    }
    else if (div16_top >= 256 * 16)
    {
        return 0;
    }

    pwm_set_clkdiv_int_frac(slice_num, div16_top / 16, div16_top & 0xF);
    pwm_set_wrap(slice_num, top - 1);

    return top;
}

// Motor Class

Motor::Motor(uint pin_a, uint pin_b, uint enc_a, uint enc_b, bool left) 
    : pid(left ? MOTOR_L_PID_KP : MOTOR_R_PID_KP, 
        left ? MOTOR_L_PID_KI : MOTOR_R_PID_KI, 
        left ? MOTOR_L_PID_KD : MOTOR_R_PID_KD, MOTOR_DEADBAND_PWM,  0, UINT16_MAX)
{
    
    this->pin_a = pin_a;
    this->pin_b = pin_b;
    this->enc.pin_a = enc_a;
    this->enc.pin_b = enc_b;
    this->enc.pulses = 0;

    this->dir = 0;
    this->pid_pwm = 0.0;
    this->curr_speed = 0.0;
    this->goal_speed = 0.0;
     
    gpio_init(this->enc.pin_a);
    gpio_set_dir(this->enc.pin_a, GPIO_IN);
    gpio_pull_up(this->enc.pin_a);
    gpio_set_irq_enabled_with_callback(
        this->enc.pin_a,
        GPIO_IRQ_EDGE_RISE, true,
        Motor::onInterrupt);

    gpio_init(this->enc.pin_b);
    gpio_pull_up(this->enc.pin_b);
    gpio_set_dir(this->enc.pin_b, GPIO_IN);

    gpio_set_function(this->pin_a, GPIO_FUNC_PWM);
    auto pwm_slice = pwm_gpio_to_slice_num(this->pin_a);
    auto pwm_config = pwm_get_default_config(); 
    pwm_init(pwm_slice, &pwm_config, true);

    pwm_set_frequency(this->pin_a, 500);

    gpio_set_function(this->pin_b, GPIO_FUNC_PWM);
    pwm_slice = pwm_gpio_to_slice_num(this->pin_b);
    pwm_config = pwm_get_default_config();
    pwm_init(pwm_slice, &pwm_config, true);

    this->wrap = pwm_set_frequency(this->pin_b, 500);
}

void Motor::setSpeed(double rpm)
{
    auto goal_rpm = clamp(rpm, -MOTOR_SOFT_MAX_RPM, MOTOR_SOFT_MAX_RPM);

    if (std::abs(goal_rpm) < MOTOR_MIN_RPM)
    {
        goal_rpm = 0;
        this->pid.reset();
    }

    if (this->goal_speed == std::abs(goal_rpm) && this->dir == sign(goal_rpm))
        return;
    
    this->goal_speed = abs(goal_rpm);

    if (sign(goal_rpm) != this->dir)
    {
        this->pid.reset();
    }

    this->dir = sign(goal_rpm);
}

void Motor::setRawSpeed(uint16_t pwm)
{

    if(std::abs(pwm) < MOTOR_DEADBAND_PWM)
    {
        pwm_set_gpio_level(this->pin_a, 0);
        pwm_set_gpio_level(this->pin_b, 0);
    }
    else if(this->dir > 0)
    {
        pwm_set_gpio_level(this->pin_a, pwm * (this->wrap) / UINT16_MAX);
        pwm_set_gpio_level(this->pin_b, 0);
    }
    else if (this->dir < 0)
    {
        pwm_set_gpio_level(this->pin_a, 0);
        pwm_set_gpio_level(this->pin_b, pwm * (this->wrap) / UINT16_MAX);
    }
    else
    {
        pwm_set_gpio_level(this->pin_a, 0);
        pwm_set_gpio_level(this->pin_b, 0);
    }

}

void Motor::update(double delta_time)
{
    
    // Calculate data
    auto revolutions = MOTOR_REDUCTION_RATE * (this->enc.pulses / MOTOR_PULSES_PER_REVOLUTIONS);

    this->curr_speed = revolutions / (60.0 * delta_time);
    
    this->enc.pulses = 0;

    this->pid_pwm = this->pid.compute(
        this->goal_speed,
        std::abs(this->curr_speed));

    this->setRawSpeed(this->pid_pwm);
}

void Motor::onInterrupt(uint gpio, uint32_t event_mask)
{
    auto motor = MotorsComponent::getMotorFromEncoderPin(gpio);

    if(motor == nullptr)
        return;

    if (gpio_get(motor->enc.pin_b))
        motor->enc.pulses = motor->enc.pulses + 1;
    else
        motor->enc.pulses = motor->enc.pulses - 1;
}

// MotorsComponent Class

MotorsComponent* MotorsComponent::instance;
QueueHandle_t MotorsComponent::motors_queue = nullptr;


MotorsComponent::MotorsComponent() 
    : MessageComponent("motors", CORE1, MOTORS_PRIORITY, UROS_MOTORS_RATE)
{
    instance = this;
}

void MotorsComponent::init()
{    
    this->raw_motor_data[0] = 0;
    this->raw_motor_data[1] = 0;
    this->motors_queue = xQueueCreate(1, sizeof(double)*2);
    this->lastUpdate = 0;

    // Initialize the data structures
    this->wheel_vels_data.velocity_left = 0;
    this->wheel_vels_data.velocity_right = 0;
    this->wheel_vels_data.timestamp_ms = 0;

    this->goal_vels_data.velocity_left = 0;
    this->goal_vels_data.velocity_right = 0;
    this->goal_vels_data.timestamp_ms = 0;

    this->joint_state_data.position[0] = 0;
    this->joint_state_data.position[1] = 0;
    this->joint_state_data.velocity[0] = 0;
    this->joint_state_data.velocity[1] = 0;
    this->joint_state_data.effort[0] = 0;
    this->joint_state_data.effort[1] = 0;
    this->joint_state_data.timestamp_ms = 0;

    // 0 - Left
    // 1 - Right
    this->motors.push_back(Motor(
        MOTOR_A0_PIN, MOTOR_A1_PIN, 
        MOTOR_A_ENC0_PIN, MOTOR_A_ENC1_PIN
    ));

    this->motors.push_back(Motor(
        MOTOR_B0_PIN, MOTOR_B1_PIN, 
        MOTOR_B_ENC0_PIN, MOTOR_B_ENC1_PIN,
        false
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

QueueHandle_t MotorsComponent::getMotorsQueue()
{
    return MotorsComponent::motors_queue;
}

uint8_t MotorsComponent::getHandlesNum()
{
    return 2;
}

void MotorsComponent::msgInit()
{
    // Add subscription for velocity commands
    this->addSubscription(
        "/cmd_vel",
        &MSG_CMD_VEL,
        MotorsComponent::onVelMessage,
        false
    );
    
    // Add publishers for wheel velocities, goals, and joint states
    this->addPublisher("/wheels/vel", &MSG_WHEEL_VELS);
    this->addPublisher("/wheels/goal", &MSG_WHEEL_GOAL);
    this->addPublisher("/wheels/joint_states", &MSG_JOINT_STATES);
}

void MotorsComponent::onVelMessage(MessageComponent* component, const void* msg_in)
{
    auto msg = (const TwistData*) msg_in;
    auto motors_comp = (MotorsComponent*) component;
    
    if(msg->linear.y > 0)
    {
        auto id = msg->linear.y - 1;
        auto p = msg->linear.z;
        auto i = msg->angular.x;
        auto d = msg->angular.y;
        
        sparkie::BuzzerComponent::play(sparkie::BuzzerAction::CONNECTED);

        motors_comp->motors[id].pid.set(p, i, d);
        return;
    }

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

    // Update the goal velocities message
    motors_comp->goal_vels_data.timestamp_ms = to_ms_since_boot(get_absolute_time());
    motors_comp->goal_vels_data.velocity_left = motors_comp->motors[0].goal_speed;
    motors_comp->goal_vels_data.velocity_right = motors_comp->motors[1].goal_speed;

    // Send the goal velocities message
    motors_comp->sendMessage(1, &motors_comp->goal_vels_data, sizeof(motors_comp->goal_vels_data));
}

void MotorsComponent::loop(TickType_t * xLastWakeTime)
{
    auto current_time = to_ms_since_boot(get_absolute_time());
    if (this->lastUpdate != 0)
    {
        auto delta_time = (current_time - lastUpdate) / (1000.0);

        for (size_t i = 0; i < MOTORS_NUM; ++i)
        {
            this->motors[i].update(delta_time);
        }
    }

    this->lastUpdate = current_time;

    // Update the wheel velocities message
    this->wheel_vels_data.timestamp_ms = current_time;
    this->wheel_vels_data.velocity_left = this->motors[0].curr_speed;
    this->wheel_vels_data.velocity_right = this->motors[1].curr_speed;

    // Send the wheel velocities message
    this->sendMessage(0, &this->wheel_vels_data, sizeof(this->wheel_vels_data));

    // Update the joint state message
    this->joint_state_data.timestamp_ms = current_time;
    this->joint_state_data.position[0] += (this->motors[0].curr_speed * RPM_TO_RADS) * (1 / UROS_MOTORS_RATE);
    this->joint_state_data.position[1] += (this->motors[1].curr_speed * RPM_TO_RADS) * (1 / UROS_MOTORS_RATE);
    this->joint_state_data.velocity[0] = this->motors[0].curr_speed * RPM_TO_RADS;
    this->joint_state_data.velocity[1] = this->motors[1].curr_speed * RPM_TO_RADS;

    // Send the joint state message
    this->sendMessage(2, &this->joint_state_data, sizeof(this->joint_state_data));

    // Odometry data queue
    this->raw_motor_data[0] = this->motors[0].curr_speed;
    this->raw_motor_data[1] = this->motors[1].curr_speed;

    xQueueOverwrite(this->motors_queue, &this->raw_motor_data);
}

void MotorsComponent::safeStop()
{
    for (size_t i = 0; i < MOTORS_NUM; i++)
    {
        this->motors[i].setRawSpeed(0);
    }
}
