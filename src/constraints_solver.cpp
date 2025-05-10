#include "constraints_solver.hpp"
#include "soft_body.hpp"
#include "utils.hpp"
#include "renderer.hpp"

#include <iostream>
#include <SFML/Graphics.hpp>
#include <glm/gtx/norm.hpp>

void SolveDistanceConstraints(PointMasses &pm, std::vector<DistanceConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        auto &p1 = pm.positions[c.i1];
        auto &p2 = pm.positions[c.i2];

        float w1 = pm.inverseMasses[c.i1];
        float w2 = pm.inverseMasses[c.i2];

        glm::vec2 delta = p1 - p2;
        float len = glm::length(delta);
        if (len < 1e-6f)
            continue;

        float C = len - c.restDistance;
        glm::vec2 grad = delta / len;

        float alphaTilde = c.compliance / (dt * dt);
        float denom = w1 + w2 + alphaTilde;
        if (denom < 1e-6f)
            continue;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;
        c.lambda += deltaLambda;

        p1 += w1 * deltaLambda * grad;
        p2 -= w2 * deltaLambda * grad;
    }
}

void SolveVolumeConstraints(PointMasses &pm, std::vector<VolumeConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        const auto &indices = c.indices;
        size_t N = indices.size();

        float volume = ComputePolygonArea(pm.positions, c.indices);
        float C = volume - c.restVolume;

        float denom = 0.0f;
        std::vector<glm::vec2> grads(N);

        for (size_t i = 0; i < N; ++i)
        {
            const glm::vec2 &pi_prev = pm.positions[indices[(i + N - 1) % N]];
            const glm::vec2 &pi_next = pm.positions[indices[(i + 1) % N]];

            glm::vec2 grad = 0.5f * Perp2D(pi_next, pi_prev);
            grads[i] = grad;
            denom += pm.inverseMasses[indices[i]] * glm::dot(grad, grad);
        }

        float alphaTilde = c.compliance / (dt * dt);
        denom += alphaTilde;
        if (denom < 1e-6f)
            continue;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;
        c.lambda += deltaLambda;

        for (size_t i = 0; i < N; ++i)
        {
            uint32_t idx = indices[i];

            pm.positions[idx] += pm.inverseMasses[idx] * deltaLambda * grads[i];
        }
    }
}

void SolveAngleConstraints(PointMasses &pm, std::vector<AngleConstraint> &constraints, float dt)
{
    for (auto &constraint : constraints)
    {
        uint32_t i1 = constraint.i1;
        uint32_t i2 = constraint.i2;
        uint32_t i3 = constraint.i3;

        glm::vec2 &p1 = pm.positions[i1];
        glm::vec2 &p2 = pm.positions[i2];
        glm::vec2 &p3 = pm.positions[i3];

        float w1 = pm.inverseMasses[i1];
        float w2 = pm.inverseMasses[i2];
        float w3 = pm.inverseMasses[i3];

        glm::vec2 d1 = glm::normalize(p1 - p2);
        glm::vec2 d2 = glm::normalize(p3 - p2);

        float C = CalculateAngle(p1, p2, p3) - constraint.restAngle;

        glm::vec2 grad_p1 = (1.0f / glm::length(p1 - p2)) * glm::vec2(-d2.y, d2.x);
        glm::vec2 grad_p3 = (1.0f / glm::length(p3 - p2)) * glm::vec2(d1.y, -d1.x);
        glm::vec2 grad_p2 = -(grad_p1 + grad_p3);

        float invMass = w1 * glm::dot(grad_p1, grad_p1) +
                        w2 * glm::dot(grad_p2, grad_p2) +
                        w3 * glm::dot(grad_p3, grad_p3);

        float alpha = constraint.compliance / (dt * dt);
        float deltaLambda = (-C - alpha * constraint.lambda) / (invMass + alpha);
        constraint.lambda += deltaLambda;

        p1 += w1 * deltaLambda * grad_p1;
        p2 += w2 * deltaLambda * grad_p2;
        p3 += w3 * deltaLambda * grad_p3;
    }
}

void SolvePinConstraints(PointMasses &pm, std::vector<PinConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        glm::vec2 &x = pm.positions[c.index];
        float w = pm.inverseMasses[c.index];
        if (w == 0.0f)
            continue;

        glm::vec2 grad = c.targetPosition - x;
        float C = glm::length(grad);
        if (C < 1e-6f)
            continue;

        grad /= C;

        float alphaTilde = c.compliance / (dt * dt);
        float denom = w + alphaTilde;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;
        c.lambda += deltaLambda;

        x -= w * deltaLambda * grad;
    }
}

float ComputeAverageAngle(const std::vector<glm::vec2> &startPositions,
                          const std::vector<glm::vec2> &currentPositions,
                          const glm::vec2 &startCenter,
                          const glm::vec2 &currentCenter)
{
    float angleSum = 0.0f;
    size_t count = startPositions.size();

    for (size_t i = 0; i < count; ++i)
    {
        glm::vec2 qi = glm::normalize(startPositions[i] - startCenter);
        glm::vec2 pi = glm::normalize(currentPositions[i] - currentCenter);

        float angle = atan2(pi.y, pi.x) - atan2(qi.y, qi.x);
        // Приводим угол к [-π, π]
        angle = fmod(angle + glm::pi<float>(), glm::two_pi<float>()) - glm::pi<float>();

        angleSum += angle;
    }

    return angleSum / static_cast<float>(count);
}

void SolveShapeMatchingConstraints(PointMasses &pm,
                                   std::vector<ShapeMatchingConstraint> &constraints,
                                   float dt)
{
    for (auto &с : constraints)
    {
        std::vector<glm::vec2> currentPositions;
        for (auto index : с.indices)
            currentPositions.push_back(pm.positions[index]);

        glm::vec2 currentCenter = ComputeGeometryCenter(currentPositions);
        glm::vec2 startCenter = ComputeGeometryCenter(с.startPositions);

        float avgAngle = ComputeAverageAngle(с.startPositions, currentPositions, startCenter, currentCenter);
        glm::mat2 R = RotationMatrixFromAngle2D(-avgAngle);

        float wSum = 0;
        for (auto index : с.indices)
            wSum += pm.inverseMasses[index];

        for (size_t i = 0; i < с.indices.size(); ++i)
        {
            uint32_t index = с.indices[i];
            glm::vec2 goal = R * (с.startPositions[i] - startCenter) + currentCenter;
            Renderer::DrawCircle(pm.positions[index], 2, sf::Color::Red);
            Renderer::DrawCircle(goal, 2, sf::Color::Green);
            Renderer::DrawLine(pm.positions[index], goal, sf::Color::Red);

            glm::vec2 correction = goal - pm.positions[index];

            float C = glm::length(correction);
            if (C > 1e-6f)
            {
                glm::vec2 grad = correction / C;
                float alphaTilde = с.compliance / (dt * dt);
                float deltaLambda = (-C - alphaTilde * с.lambda) / (wSum + alphaTilde);
                // с.lambda += deltaLambda;

                pm.positions[index] -= pm.inverseMasses[index] * deltaLambda * grad;
            }
        }
    }
}

void SolveAccelerationConstraints(PointMasses &pm, std::vector<AccelerationConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        for (auto idx : c.indices)
        {
            if (pm.inverseMasses[idx] == 0.0f)
                continue;
            glm::vec2 accel = c.acceleration;
            pm.positions[idx] += accel * dt * dt;
        }
    }
}

void SolveForceConstraints(PointMasses &pm, std::vector<ForceConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        for (auto idx : c.indices)
        {
            if (pm.inverseMasses[idx] == 0.0f)
                continue;
            glm::vec2 acceleration = c.force * pm.inverseMasses[idx];
            pm.positions[idx] += acceleration * dt * dt;
        }
    }
}

void SolveVelocityConstraints(PointMasses &pm, std::vector<VelocityConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        for (auto idx : c.indices)
        {
            if (pm.inverseMasses[idx] == 0.0f)
                continue;
            glm::vec2 desiredPos = pm.prevPositions[idx] + c.velocity * dt;
            glm::vec2 correction = desiredPos - pm.positions[idx];
            pm.positions[idx] += correction;
        }
    }
}

void SolveAngularAccelerationConstraints(PointMasses &pm, std::vector<AngularAccelerationConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        for (auto idx : c.indices)
        {
            if (pm.inverseMasses[idx] == 0.0f)
                continue;

            float angle = 0.5f * c.acceleration * dt * dt;

            glm::mat2 rotation = glm::mat2(
                glm::cos(angle), -glm::sin(angle),
                glm::sin(angle), glm::cos(angle));

            glm::vec2 relativePos = pm.positions[idx] - c.position;
            pm.positions[idx] = c.position + rotation * relativePos;
        }
    }
}

void SolveAngularForceConstraints(PointMasses &pm, std::vector<AngularForceConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        for (auto idx : c.indices)
        {
            if (pm.inverseMasses[idx] == 0.0f)
                continue;

            glm::vec2 relativePos = pm.positions[idx] - c.position;
            float r_len2 = glm::dot(relativePos, relativePos);
            if (r_len2 == 0.0f)
                continue;

            float mass = 1.0f / pm.inverseMasses[idx];
            float inertia = mass * r_len2;
            float angularAccel = c.force / inertia;

            float angle = 0.5f * angularAccel * dt * dt;

            glm::mat2 rotation = glm::mat2(
                glm::cos(angle), -glm::sin(angle),
                glm::sin(angle), glm::cos(angle));

            pm.positions[idx] = c.position + rotation * relativePos;
        }
    }
}

void SolveAngularVelocityConstraints(PointMasses &pm, std::vector<AngularVelocityConstraint> &constraints, float dt)
{
    for (auto &c : constraints)
    {
        for (auto idx : c.indices)
        {
            if (pm.inverseMasses[idx] == 0.0f)
                continue;

            float angle = c.velocity * dt;

            glm::mat2 rotation = glm::mat2(
                glm::cos(angle), -glm::sin(angle),
                glm::sin(angle), glm::cos(angle));

            glm::vec2 relativePos = pm.positions[idx] - c.position;
            pm.positions[idx] = c.position + rotation * relativePos;
        }
    }
}