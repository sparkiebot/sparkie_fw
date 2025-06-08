#pragma once

#include "../messaging/MessageComponent.hpp"
#include <source_location>
#include <string>

namespace sparkie
{
    // Enum per i livelli di log
    enum LogLevel
    {
        Debug = 10,
        Info = 20,
        Warning = 30,
        Error = 40,
        Fatal = 50
    };
    
    // Funzione di utilità per convertire LogLevel in stringa
    inline std::string logLevelToString(LogLevel level)
    {
        switch(level)
        {
            case Debug: return "Debug";
            case Info: return "Info";
            case Warning: return "Warning";
            case Error: return "Error";
            case Fatal: return "Fatal";
            default: return "Unknown";
        }
    }

    class LoggerComponent : public MessageComponent
    {
    public:
        LoggerComponent();

        // Metodo statico per loggare messaggi
        static void log(LogLevel level, std::string msg, 
            const std::source_location& location = std::source_location::current());
            
        // Numero di handle di sottoscrizione
        uint8_t getHandlesNum() override;

    protected:
        // Inizializzazione del sistema di messaggistica
        void msgInit() override;

    private:
        // Implementazione dei metodi astratti ereditati
        void init() override;
        void loop(TickType_t* xLastWakeTime) override;
    };    
} // namespace sparkie
