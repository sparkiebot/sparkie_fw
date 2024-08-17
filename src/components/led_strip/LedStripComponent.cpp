#include "LedStripComponent.hpp"
#include "../logger/LoggerComponent.hpp"
#include <iostream>
#include "../../sparkie_defs.hpp"
#include "../../led_effects/colors.hpp"
#include "../../led_effects/FadeInOut.hpp"
#include "../../led_effects/Static.hpp"
#include "../../led_effects/Ring.hpp"

using namespace sparkie;

LedStripComponent* LedStripComponent::instance = nullptr;

LedStripComponent::LedStripComponent() 
    : URosComponent("ledstrip", CORE1, LEDSTRIP_PRIORITY, UROS_LEDSTRIP_RATE), 
      controller(PicoLed::PicoLedController(nullptr))
{
    instance = this;
}

LedStripComponent* LedStripComponent::getInstance()
{
    return instance;
}

void LedStripComponent::setMode(const LedStripMode mode, uint8_t index)
{
    if(!this->modes.contains(mode))
        return;

    if(index > this->curr_effects.max_size())
        return;

    if(this->curr_effects[index].second == mode)
        return;

    auto effect = this->modes[mode];

    if(mode != LedStripMode::Off)
    {
        effect->reset();
    }

    this->curr_effects[index].first = effect;
    this->curr_effects[index].second = mode;
}

const LedStripMode LedStripComponent::getMode(uint8_t index)
{
    return this->curr_effects[index].second;
}

void LedStripComponent::init()
{
    this->sm_index = pio_claim_unused_sm(pio0, true);
    this->controller = PicoLed::addLeds<PicoLed::WS2812B>(pio0, this->sm_index, LEDSTRIP_PIN, LED_LENGTH, PicoLed::FORMAT_GRB);
    this->controller.setBrightness(16);

    // Initialize the effects..
    
    this->modes.insert(std::make_pair(LedStripMode::Off, nullptr));

    // Battery Effects
    this->modes.insert(std::make_pair(LedStripMode::Charging, new visuals::FadeInOut(visuals::colors::ORANGE, 2.0f, true)));
    this->modes.insert(std::make_pair(LedStripMode::ChargeComplete, new visuals::FadeInOut(visuals::colors::GREEN, 2.0f, true)));
    this->modes.insert(std::make_pair(LedStripMode::LowBattery, new visuals::FadeInOut(visuals::colors::RED, 3.0f, true)));
    this->modes.insert(std::make_pair(LedStripMode::AbnormalCharging, new visuals::FadeInOut(visuals::colors::VIOLET, 3.0f, true)));
    
    // Notification effects
    this->modes.insert(std::make_pair(LedStripMode::Idle, new visuals::Static(visuals::colors::WHITE)));
    this->modes.insert(std::make_pair(LedStripMode::Good, new visuals::FadeInOut(visuals::colors::GREEN, 3.5f)));
    this->modes.insert(std::make_pair(LedStripMode::Bad, new visuals::FadeInOut(visuals::colors::RED, 3.5f)));
    this->modes.insert(std::make_pair(LedStripMode::StartingUp, new visuals::FadeInOut(visuals::colors::BLUE, 2.0f, true)));
    this->modes.insert(std::make_pair(LedStripMode::Calculating, new visuals::FadeInOut(visuals::colors::BLUE, 4.0f, true)));
    this->modes.insert(std::make_pair(LedStripMode::Connecting, new visuals::Ring(visuals::colors::BLUE, 5, 1.0f)));

    this->setMode(LedStripMode::Off, 0);
    this->setMode(LedStripMode::Off, 1);
    this->setMode(LedStripMode::Off, 2);
}

uint8_t LedStripComponent::getHandlesNum()
{
    return 1;
}

void LedStripComponent::rosInit()
{
    this->addSubscription(
        "ledstrip",
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        LedStripComponent::onMessage,
        &this->data_msg,
        false
    );
}

void LedStripComponent::onMessage(URosComponent* component, const void* msg_in)
{
    auto msg = (const std_msgs__msg__UInt8*) msg_in;
    auto ledstrip =  (LedStripComponent*) component;
    auto mode = static_cast<LedStripMode>(msg->data);
    
    if(msg->data < 10 || msg->data > 19)
        mode = LedStripMode::Off;
    
    ledstrip->setMode(mode, LED_NOTIFY_LAYER);
}

void LedStripComponent::loop(TickType_t* xLastWakeTime)
{

    this->controller.clear();

    for (size_t i = 0; i < this->curr_effects.size(); i++)
    {
        auto effect_pair = this->curr_effects[i];

        if(effect_pair.second == LedStripMode::Off)
            continue;

        auto effect = effect_pair.first;

        if(effect->done())
        {
            this->curr_effects[i].first = nullptr;
            this->curr_effects[i].second = LedStripMode::Off;
            continue;
        } 

        effect->update(this->controller);
    }

    this->controller.show();
}

void LedStripComponent::safeStop()
{
    this->controller.clear();
    this->controller.show();
}