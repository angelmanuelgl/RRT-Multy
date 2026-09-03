/*
    VelocityIntegrator.cpp 
    se encarga de mover los robots entre dos configuraciones
    
    
    Configuracion inicial + configuracion destino
                    |
                    V
    calcular velocidades necesarias
                    |
                    V
    avanzar pequeños intervalos de tiempo con Euler
                    |
                    V
    nuevas posiciones de los robots
*/
#include "VelocityIntegrator.h"

#include <cstddef>

void VelocityIntegrator::setState(const std::vector<Config>& state)
{
    next_ = state;
    current_ = state;
    previous_ = state;
    velocities_.resize(state.size());
    segmentReady_ = false;
}

void VelocityIntegrator::setVelocities(const std::vector<Velocities>& velocities)
{
    velocities_ = velocities;
}

void VelocityIntegrator::beginSegment(const std::vector<Config>& start,
                                      const std::vector<Config>& target,
                                      float tau)
{
    if (start.empty() || start.size() != target.size()) {
        return;
    }

    current_ = start;
    previous_ = current_;
    next_ = target;
    computeVelocities(tau);
}

void VelocityIntegrator::computeVelocities(float tau)
{
    if (tau == 0.0f || next_.size() != current_.size()) {
        return;
    }

    velocities_.resize(current_.size());
    for (std::size_t i = 0; i < current_.size(); ++i) {
        velocities_[i].Vx=(next_[i].x-current_[i].x)/tau;
        velocities_[i].Vy=(next_[i].y-current_[i].y)/tau;
        //velocities_[i].Wang=(next_[i].theta-current_[i].theta)/tau;
    }

    //compuvels
    //computeVelocities(1,numRobots);//tau,NumRobots //calcular velocidad=deltaX/tau
    segmentReady_=true;//desactivar ya que solo calculamos por cada qnew
}

void VelocityIntegrator::advance(float deltaTime)
{
    if (!segmentReady_ || current_.size() != velocities_.size()) {
        return;
    }

    next_.resize(current_.size());
    //previous_ = current_;   // posición antes del paso Euler
    for (std::size_t i = 0; i < current_.size(); ++i) {
        next_[i].x=current_[i].x+velocities_[i].Vx*deltaTime;
        next_[i].y=current_[i].y+velocities_[i].Vy*deltaTime;
        next_[i].theta=current_[i].theta+velocities_[i].Wang*deltaTime;
    }
    current_=next_;
}

const std::vector<Config>& VelocityIntegrator::getNext() const
{
    return next_;
}

const std::vector<Config>& VelocityIntegrator::getCurrent() const
{
    return current_;
}

const std::vector<Config>& VelocityIntegrator::getPrevious() const
{
    return previous_;
}

int VelocityIntegrator::getNumRobots() const
{
    return static_cast<int>(current_.size());
}

bool VelocityIntegrator::needsSegment() const
{
    return !segmentReady_;
}
