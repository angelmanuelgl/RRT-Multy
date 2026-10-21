#include "animmov.h"

#include "logger.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kMotionEpsilon = 1e-6f;

bool validMotion(const EdgeMotion& motion, std::size_t robotCount,
                 float& totalDuration)
{
    totalDuration = 0.0f;
    if (motion.phases.empty())
        return false;

    for (const auto& phase : motion.phases) {
        if (!std::isfinite(phase.duration) || phase.duration <= 0.0f
            || phase.velocities.size() != robotCount)
            return false;
        for (const auto& velocity : phase.velocities) {
            if (!std::isfinite(velocity.Vx) || !std::isfinite(velocity.Vy)
                || !std::isfinite(velocity.Wang))
                return false;
        }
        totalDuration += phase.duration;
    }
    return std::isfinite(totalDuration) && totalDuration > 0.0f;
}

} // namespace

bool interpolateEdgeMotion(const std::vector<Config>& start,
                           const std::vector<Config>& target,
                           const EdgeMotion& motion, float elapsed,
                           std::vector<Config>& state)
{
    if (start.empty() || start.size() != target.size()
        || !std::isfinite(elapsed) || elapsed < 0.0f) {
        LOG_ERROR("Estado o tiempo invalido al interpolar una arista");
        return false;
    }

    float duration = 0.0f;
    if (!validMotion(motion, start.size(), duration)) {
        LOG_ERROR("Perfil de velocidad invalido en una arista");
        return false;
    }

    if (elapsed >= duration - kMotionEpsilon) {
        state = target;
        return true;
    }

    state = start;
    float remaining = elapsed;
    for (const auto& phase : motion.phases) {
        const float appliedTime = std::min(remaining, phase.duration);
        for (std::size_t robot = 0; robot < state.size(); ++robot) {
            state[robot].x += phase.velocities[robot].Vx * appliedTime;
            state[robot].y += phase.velocities[robot].Vy * appliedTime;
            state[robot].theta += phase.velocities[robot].Wang * appliedTime;
        }
        remaining -= appliedTime;
        if (remaining <= 0.0f)
            break;
    }
    return true;
}

bool AnimMov::setPath(const std::vector<PlanNode>& tree,
                      const std::vector<int>& path)
{
    reset();
    segments_.clear();
    state_.clear();
    pathReady_ = false;

    if (tree.empty() || path.empty()) {
        LOG_WARN("No se puede preparar la animacion: ruta o arbol vacio");
        return false;
    }

    const int firstIndex = path.front();
    if (firstIndex < 0 || firstIndex >= static_cast<int>(tree.size())
        || tree[firstIndex].q.empty()) {
        LOG_ERROR("Indice inicial invalido en la ruta de animacion");
        return false;
    }
    state_ = tree[firstIndex].q;

    for (std::size_t i = 1; i < path.size(); ++i) {
        const int parentIndex = path[i - 1];
        const int childIndex = path[i];
        if (parentIndex < 0 || childIndex < 0
            || parentIndex >= static_cast<int>(tree.size())
            || childIndex >= static_cast<int>(tree.size())
            || tree[childIndex].parent != parentIndex
            || tree[parentIndex].q.size() != tree[childIndex].q.size()) {
            LOG_ERROR("Arista invalida en la ruta de animacion: paso=", i);
            segments_.clear();
            state_.clear();
            return false;
        }

        float duration = 0.0f;
        const auto& motion = tree[childIndex].incomingMotion;
        if (!validMotion(motion, tree[parentIndex].q.size(), duration)) {
            LOG_ERROR("La arista de la ruta no tiene un perfil de velocidad valido: nodo=",
                      childIndex);
            segments_.clear();
            state_.clear();
            return false;
        }

        segments_.push_back({tree[parentIndex].q, tree[childIndex].q,
                             motion, duration});
    }

    pathReady_ = true;
    LOG_SUCCESS("Ruta preparada para animacion: tramos=", segments_.size(),
                ", robots=", state_.size());
    return true;
}

bool AnimMov::start()
{
    if (!pathReady_) {
        LOG_WARN("No se puede iniciar la animacion sin una ruta valida");
        return false;
    }

    segmentIndex_ = 0;
    segmentElapsed_ = 0.0f;
    started_ = true;
    finished_ = segments_.empty();
    if (finished_) {
        started_ = false;
        LOG_INFO("Ruta de animacion sin tramos; se conserva la configuracion inicial");
        return false;
    }

    state_ = segments_.front().start;
    LOG_INFO("Animacion final iniciada: tramos=", segments_.size());
    return true;
}

void AnimMov::reset()
{
    segmentIndex_ = 0;
    segmentElapsed_ = 0.0f;
    started_ = false;
    finished_ = false;
    if (!segments_.empty())
        state_ = segments_.front().start;
}

void AnimMov::clear()
{
    segments_.clear();
    state_.clear();
    segmentIndex_ = 0;
    segmentElapsed_ = 0.0f;
    pathReady_ = false;
    started_ = false;
    finished_ = false;
}

bool AnimMov::advance(float deltaTime)
{
    if (!isActive() || !std::isfinite(deltaTime) || deltaTime < 0.0f)
        return finished_;

    float remainingTime = deltaTime;
    while (isActive() && remainingTime > 0.0f) {
        auto& segment = segments_[segmentIndex_];
        const float segmentRemaining = segment.duration - segmentElapsed_;
        const float appliedTime = std::min(remainingTime, segmentRemaining);
        segmentElapsed_ += appliedTime;
        remainingTime -= appliedTime;

        if (segmentElapsed_ >= segment.duration - kMotionEpsilon) {
            state_ = segment.target;
            ++segmentIndex_;
            segmentElapsed_ = 0.0f;
            if (segmentIndex_ >= segments_.size()) {
                finished_ = true;
                started_ = false;
                LOG_SUCCESS("Animacion final completada");
                break;
            }
            state_ = segments_[segmentIndex_].start;
        } else if (!interpolateEdgeMotion(segment.start, segment.target,
                                          segment.motion, segmentElapsed_, state_)) {
            started_ = false;
            finished_ = true;
            return true;
        }
    }
    return finished_;
}

const std::vector<Config>& AnimMov::state() const
{
    return state_;
}

bool AnimMov::hasPath() const
{
    return pathReady_;
}

bool AnimMov::isActive() const
{
    return started_ && !finished_;
}

bool AnimMov::isFinished() const
{
    return finished_;
}
