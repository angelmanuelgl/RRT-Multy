// logger.h
#pragma once

#include <iostream>
#include <string>


/*
 *  INFO < DEBUG < TRACE
 *  WARN < ERROR < FATAL
 *  SUCCES
*/
enum class LogLevel {
    Trace, // DEBUG INTENSO LINEA POR LINEA
    Debug,
    Info,
    Success,
    Warning, // aviso que no afectans
    Error, // error suave
    Fatal // cosas que no deberian pasar
};

// La palabra clave 'inline' es CRUCIAL en funciones libres dentro de headers
template<typename... Args>
inline void logMessage(LogLevel level, const char* file, int line, Args&&... args) {
    const char* color = "\033[0m";
    const char* prefix = "[INFO]";

    switch (level) {
    case LogLevel::Trace:   color = "\033[90m"; prefix = "[TRACE]"; break;
    case LogLevel::Debug:   color = "\033[36m"; prefix = "[DEBUG]"; break;
    case LogLevel::Info:    color = "\033[0m";  prefix = "[INFO]"; break;
    case LogLevel::Success: color = "\033[32m"; prefix = "[SUCCESS]"; break;
    case LogLevel::Warning: color = "\033[33m"; prefix = "[WARNING]"; break;
    case LogLevel::Error:   color = "\033[31m"; prefix = "[ERROR]"; break;
    case LogLevel::Fatal:   color = "\033[41;1;37m"; prefix = "[FATAL]"; break;
    }

    std::cout << color << prefix << " \033[90m(" << file << ":" << line << ")\033[0m " << color;
    (std::cout << ... << args);
    std::cout << "\033[0m" << std::endl;
}
#define LOG_INFO(...)    logMessage(LogLevel::Info,    __FILE__, __LINE__, __VA_ARGS__)
#define LOG_DEBUG(...)   logMessage(LogLevel::Debug,   __FILE__, __LINE__, __VA_ARGS__)
#define LOG_TRACE(...)   ((void)0)
// #define LOG_TRACE(...)   logMessage(LogLevel::Trace,   __FILE__, __LINE__, __VA_ARGS__)

#define LOG_SUCCESS(...) logMessage(LogLevel::Success, __FILE__, __LINE__, __VA_ARGS__)

#define LOG_WARN(...)    logMessage(LogLevel::Warning, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...)   logMessage(LogLevel::Error,   __FILE__, __LINE__, __VA_ARGS__)
#define LOG_FATAL(...)   logMessage(LogLevel::Fatal,   __FILE__, __LINE__, __VA_ARGS__)
