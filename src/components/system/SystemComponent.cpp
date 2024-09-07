#include "SystemComponent.hpp"

#include "../buzzer/BuzzerComponent.hpp"
#include "../../config.hpp"
#include "../../sparkie_defs.hpp"

#include <pico/bootrom.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>

using namespace sparkie;

SystemComponent::SystemComponent() 
    : URosComponent("system_manager", CORE1, SYS_MGNT_PRIORITY, UROS_SYSTEM_MANAGER_RATE)
{

} 

uint8_t SystemComponent::getHandlesNum()
{
    return 1;
}

uint32_t btn_click_time = 0;

void sparkie::SystemComponent::onActionBtn(uint gpio, uint32_t events)
{
    if(events & GPIO_IRQ_EDGE_FALL)
        btn_click_time = time_us_32();
    else if(events & GPIO_IRQ_EDGE_RISE)
    {
        uint32_t elapsed = time_us_32() - btn_click_time;
        elapsed /= 1000;

        if(elapsed < 1000 && elapsed > 500)
        {
            sparkie::BuzzerComponent::play(BuzzerAction::ERROR);
            // It will reboot the board.
            watchdog_reboot(0, 0, 0);
        }
        else if(elapsed > 3000)
        {
            sparkie::BuzzerComponent::play(BuzzerAction::ERROR);
            // It goes into programming mode.
            // Slightly modified from https://forums.raspberrypi.com/viewtopic.php?t=326333
            vTaskSuspendAll();
            taskENTER_CRITICAL();
            reset_usb_boot(0,0);
            taskEXIT_CRITICAL();	// should never get here!!!!
            xTaskResumeAll();
        }
    }
    watchdog_reboot(0, 0, 0);
}

void SystemComponent::rosInit()
{
    this->addSubscription(
        "system",
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        SystemComponent::onMessage,
        &this->data_msg,
        false
    );
}

void SystemComponent::init()
{

}

void SystemComponent::onMessage(URosComponent* component, const void* msg_in)
{
    const std_msgs__msg__UInt8* msg = (const std_msgs__msg__UInt8*) msg_in;
    
    if(msg->data > 1)
        return;

    // Stops anything critical.
    for (auto &&comp : AgentComponent::getInstance()->components)
    {
        comp->safeStop();
    }

    if(msg->data == 0)
    {
        sparkie::BuzzerComponent::play(BuzzerAction::ERROR);
        // It will reboot the board.
        watchdog_reboot(0, 0, 0);
    }
    else
    {   
        sparkie::BuzzerComponent::play(BuzzerAction::ERROR);
        vTaskDelay(200 / portTICK_PERIOD_MS);            
        // It goes into programming mode.
        // Slightly modified from https://forums.raspberrypi.com/viewtopic.php?t=326333
        vTaskSuspendAll();
        taskENTER_CRITICAL();
        reset_usb_boot(0,0);
        taskEXIT_CRITICAL();	// should never get here!!!!
        xTaskResumeAll();
    }
    
}

void SystemComponent::loop(TickType_t* xLastWakeTime)
{
}