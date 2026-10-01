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
#include "logger.h"

#include <algorithm>
#include <cstddef>

void VelocityIntegrator::setState(const std::vector<Config>& state)
{
    next_ = state;
    current_ = state;
    previous_ = state;
    target_ = state; // amgl // visual
    velocities_.resize(state.size());
    segmentReady_ = false;
    remainingTime_ = 0.0f; // amgl // visual
    LOG_DEBUG("Estado del integrador actualizado: robots=", state.size());
}

void VelocityIntegrator::setVelocities(const std::vector<Velocities>& velocities)
{
    velocities_ = velocities;
    LOG_DEBUG("Velocidades configuradas: robots=", velocities.size());
    for (std::size_t i = 0; i < velocities.size(); ++i)
        LOG_TRACE("Velocidad[", i, "]=(", velocities[i].Vx, ",",
                  velocities[i].Vy, ",", velocities[i].Wang, ")");
}

void VelocityIntegrator::beginSegment(const std::vector<Config>& start,
                                      const std::vector<Config>& target,
                                      float tau)
{
    if (start.empty() || start.size() != target.size()) {
        LOG_ERROR("No se pudo iniciar segmento Euler: origen=", start.size(),
                  ", destino=", target.size());
        return;
    }

    current_ = start;
    previous_ = current_;
    next_ = target;
    target_ = target; // amgl // visual
    LOG_DEBUG("Segmento Euler iniciado: robots=", start.size(), ", tau=", tau);
    computeVelocities(tau);
}

void VelocityIntegrator::computeVelocities(float tau)
{
    if (tau <= 0.0f || next_.size() != current_.size()) {
        LOG_ERROR("No se pudieron calcular velocidades Euler: tau=", tau,
                  ", actual=", current_.size(), ", destino=", next_.size());
        segmentReady_ = false; // amgl // visual
        remainingTime_ = 0.0f; // amgl // visual
        return;
    }

    velocities_.resize(current_.size());
    for (std::size_t i = 0; i < current_.size(); ++i) {
        velocities_[i].Vx=(next_[i].x-current_[i].x)/tau;
        velocities_[i].Vy=(next_[i].y-current_[i].y)/tau;
        velocities_[i].Wang=(next_[i].theta-current_[i].theta)/tau; // amgl // visual
        LOG_TRACE("Velocidad Euler[", i, "]=(", velocities_[i].Vx, ",",
                  velocities_[i].Vy, ",", velocities_[i].Wang, ")");
    }

    //compuvels
    //computeVelocities(1,numRobots);//tau,NumRobots //calcular velocidad=deltaX/tau
    segmentReady_=true;//desactivar ya que solo calculamos por cada qnew
    remainingTime_ = tau; // amgl // visual
    LOG_DEBUG("Velocidades Euler calculadas: robots=", velocities_.size(),
              ", duracion=", remainingTime_);
}

void VelocityIntegrator::advance(float deltaTime)
{
    if (!segmentReady_ || current_.size() != velocities_.size()) {
        return;
    }

    const float appliedTime = std::min(deltaTime, remainingTime_); // amgl // visual
    LOG_TRACE("Paso Euler: dt_solicitado=", deltaTime,
              ", dt_aplicado=", appliedTime,
              ", tiempo_restante=", remainingTime_);
    previous_ = current_; // amgl // visual
    next_.resize(current_.size());
    for (std::size_t i = 0; i < current_.size(); ++i) {
        next_[i].x=current_[i].x+velocities_[i].Vx*appliedTime;
        next_[i].y=current_[i].y+velocities_[i].Vy*appliedTime;
        next_[i].theta=current_[i].theta+velocities_[i].Wang*appliedTime;
    }
    current_=next_;

    // Cierra el segmento en el destino y permite solicitar la siguiente arista. // amgl // visual
    remainingTime_ -= appliedTime;
    if (remainingTime_ <= 0.000001f) {
        current_ = target_;
        next_ = target_;
        remainingTime_ = 0.0f;
        segmentReady_ = false;
        LOG_DEBUG("Segmento Euler completado: robots=", current_.size());
    }
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
