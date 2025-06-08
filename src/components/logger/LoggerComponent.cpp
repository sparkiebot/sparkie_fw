#include "LoggerComponent.hpp"
#include <iostream>
#include "../../sparkie_defs.hpp"
#include "../messaging/MessageBroker.hpp"

using namespace sparkie;

LoggerComponent::LoggerComponent() : MessageComponent("logger", 0x01, tskIDLE_PRIORITY + 1)
{
    // Nessuna istanza static necessaria dato che non pubblichiamo messaggi
}

void LoggerComponent::msgInit()
{
    // Registra il componente con il MessageBroker
    MessageBroker::getInstance()->registerComponent(this);
    
    // Non creiamo alcun publisher dato che vogliamo solo usare std::cout
    log(LogLevel::Info, "LoggerComponent initialized");
}

uint8_t LoggerComponent::getHandlesNum()
{
    // Numero di handle di sottoscrizione (0 per il logger)
    return 0;
}

void LoggerComponent::log(LogLevel level, std::string msg, const std::source_location& location)
{
    // Stampa il messaggio sulla console
    std::cout << "[" << logLevelToString(level) << "] " << msg << std::endl;
    
    // Stampa informazioni aggiuntive di debug se il livello è Debug
    if(level == LogLevel::Debug)
    {
        std::cout << "\t[" << location.function_name() << " in " << location.file_name() << ":" << location.line() << "] " << std::endl;
    }
}

void LoggerComponent::init()
{
    // Inizializzazione generale del componente
}

void LoggerComponent::loop(TickType_t* xLastWakeTime)
{
    // Non è necessario fare nulla nel loop principale
    vTaskDelay(pdMS_TO_TICKS(100));  // Semplice ritardo per non consumare CPU
}
