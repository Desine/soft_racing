//file ConstraintSolver.hpp
#include "soft_body.hpp"

void SolveDistanceConstraints(PointMasses &pm, std::vector<DistanceConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        auto &x1 = pm.positions[c.i1];
        auto &x2 = pm.positions[c.i2];

        float w1 = pm.inverseMasses[c.i1];
        float w2 = pm.inverseMasses[c.i2];

        glm::vec2 delta = x1 - x2;
        float len = glm::length(delta);
        if (len < 1e-6f)
            continue;

        float C = len - c.restDistance;
        glm::vec2 grad = delta / len;

        float alphaTilde = c.compliance / (dt * dt);
        float denom = w1 + w2 + alphaTilde;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;

        glm::vec2 corr = deltaLambda * grad;

        x1 += w1 * corr;
        x2 -= w2 * corr;
        c.lambda += deltaLambda;
    }
}

void SolveVolumeConstraints(PointMasses &pm, std::vector<VolumeConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        const auto &indices = c.indices;
        size_t N = indices.size();

        float volume = 0.0f;
        for (size_t i = 0; i < N; ++i)
        {
            const glm::vec2 &p0 = pm.positions[indices[i]];
            const glm::vec2 &p1 = pm.positions[indices[(i + 1) % N]];
            volume += glm::cross(glm::vec3(p0, 0.0f), glm::vec3(p1, 0.0f)).z;
        }
        volume *= 0.5f;

        float C = volume - c.restVolume;
        if (std::abs(C) < 1e-6f) continue;

        float alphaTilde = c.compliance / (dt * dt);
        float denom = 0.0f;
        std::vector<glm::vec2> grads(N);

        for (size_t i = 0; i < N; ++i)
        {
            const glm::vec2 &pi_prev = pm.positions[indices[(i + N - 1) % N]];
            const glm::vec2 &pi_next = pm.positions[indices[(i + 1) % N]];

            glm::vec2 grad = 0.5f * (glm::vec2(pi_next.y, -pi_next.x) - glm::vec2(pi_prev.y, -pi_prev.x)); // perp(next - prev)
            grads[i] = grad;
            denom += pm.inverseMasses[indices[i]] * glm::dot(grad, grad);
        }

        denom += alphaTilde;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;
        c.lambda += deltaLambda;

        for (size_t i = 0; i < N; ++i)
        {
            uint32_t idx = indices[i];
            float w = pm.inverseMasses[idx];
            if (w == 0.0f) continue;

            pm.positions[idx] += w * deltaLambda * grads[i];
        }
    }
}

void SolvePinConstraints(PointMasses& pm, std::vector<PinConstraint>& constraints, float dt)
{
    for (auto& c : constraints)
    {
        glm::vec2& xi = pm.positions[c.index];
        float wi = pm.inverseMasses[c.index];
        if (wi == 0.0f) continue;

        glm::vec2 grad = xi - c.targetPosition;
        float C = glm::length(grad);
        if (C < 1e-6f) continue;

        grad /= C;

        float alphaTilde = c.compliance / (dt * dt);
        float denom = wi + alphaTilde;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;

        xi += wi * deltaLambda * (-grad);
        c.lambda += deltaLambda;
    }
}