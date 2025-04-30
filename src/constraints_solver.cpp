#include "constraints_solver.hpp"
#include "soft_body.hpp"
#include "utils.hpp"

#include <iostream>
#include <SFML/Graphics.hpp>
#include "renderer.hpp"

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
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;
        c.lambda += deltaLambda;

        glm::vec2 corr = deltaLambda * grad;

        p1 += w1 * corr;
        p2 -= w2 * corr;
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

        float alphaTilde = c.compliance / (dt * dt);
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

        denom += alphaTilde;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;
        c.lambda += deltaLambda;

        for (size_t i = 0; i < N; ++i)
        {
            uint32_t idx = indices[i];
            float w = pm.inverseMasses[idx];
            if (w == 0.0f)
                continue;

            pm.positions[idx] += w * deltaLambda * grads[i];
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
        glm::vec2 &xi = pm.positions[c.index];
        float wi = pm.inverseMasses[c.index];
        if (wi == 0.0f)
            continue;

        glm::vec2 grad = c.targetPosition - xi;
        float C = glm::length(grad);
        if (C < 1e-6f)
            continue;

        grad /= C;

        float alphaTilde = c.compliance / (dt * dt);
        float denom = wi + alphaTilde;
        float deltaLambda = (-C - alphaTilde * c.lambda) / denom;
        c.lambda += deltaLambda;

        xi += wi * deltaLambda * (-grad);
    }
}

glm::mat2 ComputeOptimalRotation2D(const glm::mat2 &A)
{
    float det = glm::determinant(A);
    if (det <= 0.0f)
        return glm::mat2(1.0f);

    glm::vec2 col0 = A[0];
    glm::vec2 col1 = A[1];

    glm::vec2 u = glm::normalize(col0);
    glm::vec2 v = col1 - glm::dot(col1, u) * u;
    v = glm::normalize(v);

    return glm::mat2(u, v);
}

void SolveShapeMatchingConstraints(PointMasses &pm,
                                   std::vector<ShapeMatchingConstraint> &constraints,
                                   float dt)
{
    for (auto &c : constraints)
    {
        const auto &indices = c.indices;

        glm::vec2 currCenter(0.0f);
        float totalMass = 0.0f;

        for (auto i : indices)
        {
            float mass = 1.0f / pm.inverseMasses[i];
            currCenter += pm.positions[i] * mass;
            totalMass += mass;
        }
        currCenter /= totalMass;

        glm::mat2 A(0.0f);
        for (size_t k = 0; k < indices.size(); ++k)
        {
            uint32_t i = indices[k];
            float mass = 1.0f / pm.inverseMasses[i];
            glm::vec2 qi = c.startPositions[k] - c.startCenterMass;
            glm::vec2 pi = pm.positions[i] - currCenter;
            A += mass * glm::outerProduct(pi, qi);
        }

        glm::mat2 R = ComputeOptimalRotation2D(A);

        std::vector<glm::vec2> goalPositions(indices.size());
        for (size_t k = 0; k < indices.size(); ++k)
        {
            goalPositions[k] = R * (c.startPositions[k] - c.startCenterMass) + currCenter;

            // Renderer::DrawCircle(goalPositions[k], 3.0f, sf::Color(0, 255, 0, 100));
            // Renderer::DrawLine(pm.positions[indices[k]], goalPositions[k], sf::Color(255, 100, 100, 100));
        }

        float alphaTilde = c.compliance / (dt * dt);

        for (size_t k = 0; k < indices.size(); ++k)
        {
            uint32_t i = indices[k];
            float w = pm.inverseMasses[i];
            if (w == 0.0f)
                continue;

            glm::vec2 delta = goalPositions[k] - pm.positions[i];
            float C = glm::length(delta);
            if (C < 1e-6f)
                continue;

            glm::vec2 gradC = glm::normalize(delta);
            float denom = w + alphaTilde;
            float deltaLambda = (-C - alphaTilde * c.lambda) / denom;
            c.lambda += deltaLambda;

            pm.positions[i] += deltaLambda * w * gradC;
        }
    }
}
