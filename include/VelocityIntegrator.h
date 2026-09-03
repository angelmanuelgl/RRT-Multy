#pragma once

#include "Config.h"

#include <vector>

class VelocityIntegrator {
public:
    void setState(const std::vector<Config>& state);
    void setVelocities(const std::vector<Velocities>& velocities);
    void beginSegment(const std::vector<Config>& start,
                      const std::vector<Config>& target,
                      float tau);
    void computeVelocities(float tau);
    void advance(float deltaTime);

    const std::vector<Config>& getNext() const;
    const std::vector<Config>& getCurrent() const;
    const std::vector<Config>& getPrevious() const;
    int getNumRobots() const;
    bool needsSegment() const;

private:
    std::vector<Velocities> velocities_ = {
        {0.1f,0.1f,0.00f} //Vx, Vy, Wang
    };

    //para el Euler//Falta el numero de lineas
    std::vector<Config> next_;//new
    std::vector<Config> current_;//new
    std::vector<Config> previous_;//new
    bool segmentReady_ = false;//new //Requerimos calcular las velocidades entre dos puntos para un cierto tiempo?

    //int NEulerTimes=10;//división del tramo//new
};
