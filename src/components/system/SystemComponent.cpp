#include "SystemComponent.hpp"

#include "../buzzer/BuzzerComponent.hpp"
#include "../../config.hpp"
#include "../../sparkie_defs.hpp"
#include "components/messaging/MessageBroker.hpp"

#include <pico/bootrom.h>
#include <hardware/watchdog.h>
#include <pico/multicore.h>

using namespace sparkie;

// Define message type for system control
const MessageType MSG_SYSTEM_CONTROL = {"system", 0};

SystemComponent::SystemComponent() 
    : MessageComponent("system_manager", CORE0, SYS_MGNT_PRIORITY, UROS_SYSTEM_MANAGER_RATE)
{
} 

uint8_t SystemComponent::getHandlesNum()
{
    return 1;
}

void sparkie::SystemComponent::onActionBtn(uint gpio, uint32_t events)
{
    sparkie::BuzzerComponent::play(BuzzerAction::ERROR);
    // It will reboot the board.
    watchdog_reboot(0, 0, 0);
}

void SystemComponent::msgInit()
{
    this->addSubscription(
        "/system",
        &MSG_SYSTEM_CONTROL,
        SystemComponent::onMessage
    );
}

void SystemComponent::init()
{
    gpio_init(ACTN_BTN_PIN);
    gpio_set_dir(ACTN_BTN_PIN, GPIO_IN);
    gpio_pull_up(ACTN_BTN_PIN);
    gpio_set_irq_enabled_with_callback(ACTN_BTN_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, SystemComponent::onActionBtn);
}

void SystemComponent::onMessage(MessageComponent* component, const void* msg_in)
{
    const SystemData* msg = (const SystemData*) msg_in;
    
    if(msg->command > 1)
        return;

    // Stops anything critical.
    for (auto &&comp : MessageBroker::getInstance()->components)
    {
        comp->safeStop();
    }

    if(msg->command == 0)
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
    // No regular updates needed for system component
}
