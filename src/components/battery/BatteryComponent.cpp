#include "BatteryComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../../config.hpp"
#include <iostream>
#include <cmath>

#include <FreeRTOS.h>
#include <timers.h>

#include "hardware/adc.h"

#include "../led_strip/LedStripComponent.hpp"
#include "../messaging/MessageBroker.hpp"
#include "projdefs.h"

using namespace sparkie;

// Define the message type for battery data
const MessageType MSG_BATTERY_DATA = {"battery", 0};

double round_up(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::ceil(value * multiplier) / multiplier;
}

// It will approximately calculate current battery percentage 
// following a special curve made with MyCurveFit using Ampow table.
// See: 
// - https://electronics.stackexchange.com/a/551667
// - https://mycurvefit.com/
// - https://blog.ampow.com/lipo-voltage-chart/ 
// TODO: Lately a current sensor will be added and a different algorithm will be used.
float lipo_percentage(float voltage)
{
    return -0.2746 * voltage * voltage + 7.0680*voltage - 44.4878;
}

QueueHandle_t BatteryComponent::charge_state_queue;
bool BatteryComponent::previous_state;

BatteryComponent::BatteryComponent() 
    : MessageComponent("battery_sensor", CORE1, BATT_PRIORITY, UROS_BATTERY_RATE)
{
}

BatteryStatus timer_charging_state;

/*
Checks every 500ms if charging module is charging or not. Also handles abnormal charging.
https://it.aliexpress.com/item/1005005104963569.html
*/
void BatteryComponent::onChargingCheck(TimerHandle_t timer)
{
    auto curr_state = gpio_get(BATT_RECHARGE_PIN);

    if(previous_state != curr_state)
    {
        timer_charging_state = BatteryStatus::AbnormalCharging;        
    }
    else
    {
        timer_charging_state 
            = static_cast<BatteryStatus>(1 + (gpio_get(BATT_RECHARGE_PIN) == 0));
    }

    previous_state = curr_state;

    xQueueSend(BatteryComponent::charge_state_queue, &timer_charging_state, 0);
}

void BatteryComponent::msgInit()
{
    // Add the battery data publisher
    this->addPublisher("/battery", &MSG_BATTERY_DATA);
}

void BatteryComponent::init()
{
    this->low_power = false;
    this->curr_status = BatteryStatus::Unknown;
    this->curr_read = 0.0;
    
    // Initialize the battery data structure
    this->battery_data.voltage = 0.0;
    this->battery_data.percentage = 0.0;
    this->battery_data.status = 0; // Unknown
    this->battery_data.charge = 0.0;
    this->battery_data.capacity = BATT_CAPACITY;

    gpio_init(BATT_RECHARGE_PIN);
    gpio_set_dir(BATT_RECHARGE_PIN, GPIO_IN);

    adc_init();
    adc_gpio_init(BATT_VOLT_PIN);
    adc_select_input(0);

    previous_state = gpio_get(BATT_RECHARGE_PIN);
    this->charge_state_queue = xQueueCreate(1, sizeof(BatteryStatus));
    
    this->charging_check = xTimerCreate(
        "batt_charge_check",
        500 * portTICK_PERIOD_MS,
        pdTRUE,
        NULL,
        BatteryComponent::onChargingCheck
    );

    xTimerStart(this->charging_check, 0);

    const float conv_factor = 1.0f / (1 << 12);
    
    for (size_t i = 0; i < BATT_READS_COUNT; i++)
    {
        this->reads.data[i] = adc_read() * conv_factor;
        vTaskDelay(10);
    }
}

void BatteryComponent::loop(TickType_t* xLastWakeTime)
{
    /*
    A simple running average algorithm is used to smooth raw adc measurements
    */

    if(this->reads.current_index >= BATT_READS_COUNT)
        this->reads.current_index = 0;

    const float conv_factor = 1.0f / (1 << 12);
    this->reads.data[this->reads.current_index++] = adc_read() * conv_factor;

    auto sum = 0.0;
    for (size_t i = 0; i < BATT_READS_COUNT; i++)
    {
        sum += this->reads.data[i];
    }
    
    this->curr_read = round_up(sum / BATT_READS_COUNT, 4);

    // 3.3 is the max voltage of the Pico ADC
    // 0.22 is the voltage divider ratio based on the schematic
    this->battery_data.voltage = this->curr_read * (3.3f / 0.22f);
    this->battery_data.percentage = round_up(lipo_percentage(this->battery_data.voltage), 4);
    this->battery_data.charge = this->battery_data.percentage * this->battery_data.capacity;
    
    BatteryStatus charging_state;

    if(xQueueReceive(this->charge_state_queue, &charging_state, 0) == pdFALSE)
    {
        charging_state = BatteryStatus::Unknown;
    }

    if(round_up(this->battery_data.percentage, 2) >= 0.95 && charging_state == BatteryStatus::Charging)
    {
        this->battery_data.status = (uint8_t)BatteryStatus::Full;
    }
    else
    {
        this->battery_data.status = (uint8_t)charging_state;
    }

    // Send the battery data message
    this->sendMessage(0, &this->battery_data, sizeof(this->battery_data));

    // Led notifications
    auto mode = LedStripMode::Off;

    if (this->battery_data.percentage < 0.2 && !this->low_power)
    {
        this->low_power = true;
    }
    
    auto battery_status = (BatteryStatus) this->battery_data.status;

    if(battery_status == BatteryStatus::Full)
    {
        mode = LedStripMode::ChargeComplete;
    }
    else if(battery_status == BatteryStatus::Charging)
    {
        mode = LedStripMode::Charging;
        this->low_power = false;
    }
    else if(battery_status == BatteryStatus::AbnormalCharging)
    {
        mode = LedStripMode::AbnormalCharging;
    }

    if(this->low_power)
        mode = LedStripMode::LowBattery;

#ifdef ENABLE_BATTERY_LED_NOTIFICATIONS
    if (LedStripComponent::getInstance() != nullptr)
        LedStripComponent::getInstance()->setMode(mode, LED_BATTERY_LAYER);
#endif
}
