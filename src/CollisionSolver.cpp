#include "CollisionSolver.hpp"
#include <algorithm>

void GenerateCollisionConstraints(SoftBody& softBody, float groundY)
{
    auto& pm = softBody.pointMasses;
    softBody.collisionConstraints.clear();

    for (uint32_t i = 0; i < pm.positions.size(); ++i)
    {
        if (pm.inverseMasses[i] == 0.0f) continue;

        const glm::vec2& p = pm.positions[i];
        float dist = p.y - groundY;

        if (dist < 0.0f) // под землей
        {
            CollisionConstraint c;
            c.pointIndex = i;
            c.normal = glm::vec2(0.0f, 1.0f); // земля вверх
            c.contactPoint = glm::vec2(p.x, groundY); // точка на поверхности
            c.compliance = 0.0f;
            c.lambda = 0.0f;
            softBody.collisionConstraints.push_back(c);
        }
    }
}

void SolveCollisionConstraints(PointMasses& pm, std::vector<CollisionConstraint>& constraints, float dt)
{
    for (auto& c : constraints)
    {
        uint32_t i = c.pointIndex;
        glm::vec2& x = pm.positions[i];
        float w = pm.inverseMasses[i];
        if (w == 0.0f) continue;

        // Правильная функция ограничения: C(x) = n·(x - p) ≥ 0
        float C = glm::dot(c.normal, x - c.contactPoint);

        if (C >= 0.0f)
            continue;

        glm::vec2 grad = c.normal;
        float alphaTilde = c.compliance / (dt * dt);
        float denom = w + alphaTilde;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;

        float newLambda = std::max(c.lambda + deltaLambda, 0.0f);
        deltaLambda = newLambda - c.lambda;
        c.lambda = newLambda;

        glm::vec2 correction = deltaLambda * grad;
        x += w * correction;

        // XPBD-style friction
        glm::vec2 v = pm.velocities[i];
        glm::vec2 tangent = glm::vec2(c.normal.y, -c.normal.x); // перпендикуляр

        float vTangent = glm::dot(v, tangent);
        float frictionCoef = 0.5f;
        float normalImpulse = deltaLambda / dt;
        float maxFrictionImpulse = frictionCoef * normalImpulse;
        float frictionImpulse = std::clamp(-vTangent * w, -maxFrictionImpulse, maxFrictionImpulse);

        pm.velocities[i] += frictionImpulse * tangent;
    }
}

