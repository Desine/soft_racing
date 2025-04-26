// file ConstraintSolver.hpp
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

            pm.positions[idx] -= w * deltaLambda * grads[i];
        }
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

// glm::mat2 ComputeOptimalRotation2D(const glm::mat2 &A)
// {
//     // Находим ближайшую матрицу вращения через полярное разложение
//     glm::vec2 a0 = A[0];
//     glm::vec2 a1 = A[1];

//     glm::vec2 r0 = a0;
//     glm::vec2 r1 = a1 - glm::dot(a1, r0) / glm::dot(r0, r0) * r0;

//     r0 = glm::normalize(r0);
//     r1 = glm::normalize(r1);

//     glm::mat2 R;
//     R[0] = r0;
//     R[1] = glm::vec2(-r0.y, r0.x); // ортогональный вектор (в 2D достаточно этого)

//     return R;
// }

void SolveShapeMatchingConstraints(PointMasses &pm,
                                   std::vector<ShapeMatchingConstraint> &constraints,
                                   float dt)
{
    for (auto &c : constraints)
    {
    }
}