#include "UltrasonicComponent.hpp"
#include "../../sparkie_defs.hpp"

#include <iostream>
#include "pico/stdlib.h"

#include "../../misc/us_echo.pio.h"
#include "../../misc/us_trig.pio.h"

using namespace sparkie;

// Define message types for ultrasonic sensors
const MessageType MSG_RANGE_LEFT = {"us/left", 0};
const MessageType MSG_RANGE_FRONT = {"us/front", 1};
const MessageType MSG_RANGE_RIGHT = {"us/right", 2};

UltrasonicComponent::UltrasonicComponent() 
    : MessageComponent("ultrasonic_sensors", CORE1, ULTRASONIC_PRIORITY)
{
}

void UltrasonicComponent::initTriggerPio()
{
    // Init trigger pin pio
    auto sm = 3; // State Machine index
    
    pio_gpio_init(pio1, US_TRIG_PIN);
    auto offset = pio_add_program(pio1, &us_trig_program);

    auto conf = us_trig_program_get_default_config(offset);
    // set the output pin to output
    pio_sm_set_consecutive_pindirs(pio1, sm, US_TRIG_PIN, 1, true);
    // set the 'set' pins
    sm_config_set_set_pins(&conf, US_TRIG_PIN, 1);
    // set shift direction
    sm_config_set_in_shift(&conf, false, false, 0);
    // init the pio sm with the config
    pio_sm_init(pio1, sm, offset, &conf);
    // enable the sm

    irq_set_enabled(PIO1_IRQ_0, true);

    pio_sm_set_enabled(pio1, sm, true);
}

void UltrasonicComponent::initEchoPio(PIO pio, uint sm, uint echo_pin)
{
    pio_gpio_init(pio, echo_pin);
    auto offset = pio_add_program(pio, &us_echo_program);

    auto conf = us_echo_program_get_default_config(offset);
    // set the 'in' pins, also used for 'wait'
    sm_config_set_in_pins(&conf, echo_pin);
    // set the 'jmp' pin
    sm_config_set_jmp_pin(&conf, echo_pin);
    // set shift direction
    sm_config_set_in_shift(&conf, false, false, 0);
    // init the pio sm with the config
    pio_sm_init(pio, sm, offset, &conf);
    // enable the sm
    pio_sm_set_enabled(pio, sm, true);
}

void UltrasonicComponent::initSensor(UltrasonicSensor* sensor, int index, const std::string& name)
{
    const MessageType* msg_types[] = {&MSG_RANGE_LEFT, &MSG_RANGE_FRONT, &MSG_RANGE_RIGHT};
    
    this->addPublisher("/us/" + name, msg_types[index]);

    sensor->data = 0;
    sensor->frame = std::string(UROS_ULTRASONIC_FRAME) + name;
}

void UltrasonicComponent::readData(TickType_t* lastWakeTime)
{
    for (size_t i = 0; i < US_NUM; i++)
    {
        pio_sm_clear_fifos(pio1, i);
    }
    
    pio_interrupt_clear(pio1, 0);
    
    xTaskDelayUntil(lastWakeTime, pdMS_TO_TICKS(40));

    uint clock_cycles = 0;

    for (size_t i = 0; i < US_NUM; i++)
    {
        if(pio_sm_is_rx_fifo_empty(pio1, i))
        {
            this->sensors[i].data = 0;
            continue;
        }

        clock_cycles = 2 * pio_sm_get(pio1, i);
        
        // using
        // - the time for 1 pio clock tick (1/125000000 s)
        // - speed of sound in air is about 340 m/s
        // - the sound travels from the HCSR04 to the object and back (twice the distance)
        // the distance in cm is calculated by multiplying with 0.000136
        this->sensors[i].data = (float)(clock_cycles) * 0.000136f;
    }
}

void UltrasonicComponent::init()
{
    // Init trig pio
    this->initEchoPio(pio1, 0, US_LEFT_PIN + 0);
    this->initEchoPio(pio1, 1, US_LEFT_PIN + 1);
    this->initEchoPio(pio1, 2, US_LEFT_PIN + 2);
    this->initTriggerPio();

    // Initialize range data structs
    for (size_t i = 0; i < US_NUM; i++)
    {
        this->range_data[i].radiation_type = 0; // Ultrasonic type
        this->range_data[i].field_of_view = US_FOV;
        this->range_data[i].min_range = US_MIN_RANGE;
        this->range_data[i].max_range = US_MAX_RANGE;
        this->range_data[i].range = 0.0f;
        this->range_data[i].timestamp_ms = 0;
    }
}

void UltrasonicComponent::msgInit()
{
    this->initSensor(&this->sensors[0], 0, "left");
    this->initSensor(&this->sensors[1], 1, "front");
    this->initSensor(&this->sensors[2], 2, "right");
}

void UltrasonicComponent::loop(TickType_t* xLastWakeTime)
{
    this->readData(xLastWakeTime);

    // Get current timestamp
    uint32_t current_time_ms = to_ms_since_boot(get_absolute_time());
    
    for (size_t i = 0; i < US_NUM; i++)
    {
        this->range_data[i].timestamp_ms = current_time_ms;
        this->range_data[i].range = this->sensors[i].data / 100.0; // Convert from cm to meters
        
        this->sendMessage(i, &this->range_data[i], sizeof(RangeData));
    }

    // No need to specify the delay time, as the delay is already done in readData.
    // Sensor runs at 25Hz
}
