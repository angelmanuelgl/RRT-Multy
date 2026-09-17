#pragma once

#include <vector>
#include <iostream>



//mediante esta estructura se define un nodo en el arbol del RRT
struct Config {
    float x, y, theta;//Pose del robot
};

struct Velocities {
    float Vx, Vy, Wang;//velocidades robots
};//new

//mediante esta estructura se define un nodo en el arbol del RRT
struct PlanNode {
    std::vector<Config> q;//coordenadas del nodo
    int parent;//indice del nodo padre

    PlanNode(const std::vector<Config>& q_, int p_)
        : q(q_), parent(p_)
    {
    }
};


// DEBUGGER

// Niveles de log
enum class LogLevel {
    Info,
    Warning,
    Error
};

//
template<typename... Args>
void logMessage(LogLevel level, const char* file, int line, Args&&... args) {
    // colores enANSCI
    const char* color = "\033[0m";     // Reset por defecto
    const char* prefix = "[INFO]";

    switch (level) {
    case LogLevel::Info:
        // color = "\033[32m"; // Verde
        prefix = "[INFO]";
        break;
    case LogLevel::Warning:
        color = "\033[33m"; // Amarillo
        prefix = "[WARNING]";
        break;
    case LogLevel::Error:
        color = "\033[31m"; // Rojo
        prefix = "[ERROR]";
        break;
    }

    // prefijo con color y metadatos de archivo/línea
    std::cout << color << prefix << " \033[90m(" << file << ":" << line << ")\033[0m " << color;

    // los argumentos usando fold expressions (C++17)
    (std::cout << ... << args);

    // resetear color y saltar de línea
    std::cout << "\033[0m" << std::endl;
}

// Macros de conveniencia para mantener la sintaxis limpia
#define LOG_INFO(...)    logMessage(LogLevel::Info, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...)    logMessage(LogLevel::Warning, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...)   logMessage(LogLevel::Error, __FILE__, __LINE__, __VA_ARGS__)




