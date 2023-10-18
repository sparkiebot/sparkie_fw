#include "BatteryComponent.hpp"
#include "../../sparkie_defs.hpp"
#include "../../config.hpp"
#include <micro_ros_utilities/string_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <iostream>
#include <cmath>
#include "hardware/adc.h"

#include "../led_strip/LedStripComponent.hpp"

using namespace sparkie;

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
float lipo_percentage(float value)
{
    float voltage = value * 12.6;
    return -0.2746 * voltage * voltage + 7.0680*voltage - 44.4878;
}

QueueHandle_t BatteryComponent::charge_state_queue;
bool BatteryComponent::previous_state;

BatteryComponent::BatteryComponent() 
    : URosComponent("battery_sensor", CORE1, BATT_PRIORITY, UROS_BATTERY_RATE)
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


void BatteryComponent::init()
{
    this->low_power = false;
    this->curr_status = BatteryStatus::Unknown;
    this->curr_read = 0.0;

    this->battery_msg.header.frame_id = 
        micro_ros_string_utilities_init(UROS_BOARD_FRAME);
    
    this->battery_msg.voltage = 0.0;
    this->battery_msg.current = NAN;
    this->battery_msg.charge = NAN;
    this->battery_msg.capacity = BATT_CAPACITY;
    this->battery_msg.design_capacity = BATT_CAPACITY;
    this->battery_msg.percentage = 0.0;
    this->battery_msg.power_supply_status = 0; // Unknown
    this->battery_msg.power_supply_health = 0; // Unknown
    this->battery_msg.power_supply_technology = 3; // LiPo
    this->battery_msg.present = true;

    this->battery_msg.cell_voltage.data = nullptr;
    this->battery_msg.cell_voltage.capacity = 0;
    this->battery_msg.cell_voltage.size = 0;

    this->battery_msg.cell_temperature.data = nullptr;
    this->battery_msg.cell_temperature.capacity = 0;
    this->battery_msg.cell_temperature.size = 0;

    this->battery_msg.temperature = NAN;
    
    this->battery_msg.location = micro_ros_string_utilities_init("Base");
    this->battery_msg.serial_number = micro_ros_string_utilities_init("NULL");

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
        true,
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

void BatteryComponent::rosInit()
{
    this->addPublisher(
        "battery", 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        false
    );
}

void BatteryComponent::loop(TickType_t* xLastWakeTime)
{
    auto ns = (uint32_t) rmw_uros_epoch_nanos();
    auto s = (int32_t) (rmw_uros_epoch_millis() / 1000);
    
    
    this->battery_msg.header.stamp.nanosec = ns;
    this->battery_msg.header.stamp.sec = s;
    

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

    this->battery_msg.voltage = this->curr_read * 12.6;
    this->battery_msg.percentage = round_up(lipo_percentage(this->curr_read), 4);
    this->battery_msg.charge = this->battery_msg.percentage * this->battery_msg.capacity;
    
    BatteryStatus charging_state;

    if(xQueueReceive(this->charge_state_queue, &charging_state, 0) == pdFALSE)
    {
        charging_state = BatteryStatus::Unknown;
    }

    if(round_up(this->battery_msg.percentage, 2) >= 0.95 && charging_state == BatteryStatus::Charging)
    {
        this->battery_msg.power_supply_status = (uint8_t)BatteryStatus::Full;
    }
    else
    {
        this->battery_msg.power_supply_status = (uint8_t)charging_state;
    }

    this->sendMessage(0, &this->battery_msg);


    // Led notifications

    auto mode = LedStripMode::Off;

    if (this->battery_msg.percentage < 0.2 && !this->low_power)
    {
        this->low_power = true;
    }
    
    auto battery_status = (BatteryStatus) this->battery_msg.power_supply_status;

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

    LedStripComponent::getInstance()->setMode(mode, LED_BATTERY_LAYER);

}