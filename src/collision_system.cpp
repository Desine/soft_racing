#include "collision_system.hpp"
#include "simulation.hpp"
#include "renderer.hpp"
#include "utils.hpp"

#include <algorithm>
#include <iostream>

void DetectSoftSoftCollisions(
    const SoftBody &bodyA,
    const SoftBody &bodyB,
    float compliance,
    float minDistanceThreshold,
    std::vector<SoftSoftCollisionConstraint> &outConstraints)
{
    // find A's points inside B
    std::vector<uint32_t> insideB;
    for (uint32_t i = 0; i < bodyA.pointMasses.positions.size(); ++i)
    {
        const glm::vec2 &point = bodyA.pointMasses.positions[i];
        glm::vec2 rayDir(1.0f, 0.0f);
        auto hits = RaycastAllIntersections(point, rayDir, bodyB);

        if (hits.size() % 2 == 1)
            insideB.push_back(i);
    }

    // create SoftSoftCollisionConstraint
    for (uint32_t indexA : insideB)
    {
        const glm::vec2 &pointA = bodyA.pointMasses.positions[indexA];
        const auto &shapeB = bodyB.collisionShape;
        const auto &positionsB = bodyB.pointMasses.positions;

        float minEdgeDist = std::numeric_limits<float>::max();
        uint32_t nearestEdge = 0;
        glm::vec2 nearestNormal;

        for (size_t i = 0; i < shapeB.size(); ++i)
        {
            const glm::vec2 &p1 = positionsB[shapeB[i]];
            const glm::vec2 &p2 = positionsB[shapeB[(i + 1) % shapeB.size()]];

            glm::vec2 edge = p2 - p1;
            if (glm::length(edge) < 1e-6f)
                continue;
            glm::vec2 edgeDir = glm::normalize(edge);
            glm::vec2 toPoint = pointA - p1;
            float proj = glm::dot(toPoint, edgeDir);
            proj = glm::clamp(proj, 0.0f, glm::length(edge));
            glm::vec2 closest = p1 + edgeDir * proj;
            float edgeDist = glm::length(pointA - closest);

            if (edgeDist < minEdgeDist)
            {
                minEdgeDist = edgeDist;
                nearestEdge = i;
                glm::vec2 edgeNormal = glm::normalize(Perp2D(edge));
                if (glm::dot(edgeNormal, pointA - closest) < 0)
                    edgeNormal = -edgeNormal;
                nearestNormal = edgeNormal;
            }
        }

        if (minEdgeDist < minDistanceThreshold)
        {
            SoftSoftCollisionConstraint constraint;
            constraint.bodyA = const_cast<SoftBody *>(&bodyA);
            constraint.bodyB = const_cast<SoftBody *>(&bodyB);
            constraint.pointIndexA = indexA;
            constraint.edgeIndexB = nearestEdge;
            constraint.normal = nearestNormal;
            constraint.distance = minDistanceThreshold - minEdgeDist;
            constraint.compliance = compliance;
            outConstraints.push_back(constraint);
        }
    }
}

void SolveSoftSoftCollisionConstraint(
    SoftSoftCollisionConstraint &constraint,
    float dt)
{
    auto &positionsA = constraint.bodyA->pointMasses.positions;
    auto &invMassesA = constraint.bodyA->pointMasses.inverseMasses;
    auto &positionsB = constraint.bodyB->pointMasses.positions;
    auto &invMassesB = constraint.bodyB->pointMasses.inverseMasses;
    auto &shapeB = constraint.bodyB->collisionShape;

    glm::vec2 &pA = positionsA[constraint.pointIndexA];
    float wA = invMassesA[constraint.pointIndexA];

    uint32_t i0 = shapeB[constraint.edgeIndexB];
    uint32_t i1 = shapeB[(constraint.edgeIndexB + 1) % shapeB.size()];

    glm::vec2 &p0 = positionsB[i0];
    glm::vec2 &p1 = positionsB[i1];
    float w0 = invMassesB[i0];
    float w1 = invMassesB[i1];

    // closest point to pA on edge
    glm::vec2 edge = p1 - p0;
    float len = glm::length(edge);
    if (len < 1e-6f)
        return;

    glm::vec2 edgeDir = edge / len;
    float t = glm::dot(pA - p0, edgeDir);
    t = glm::clamp(t, 0.0f, len);

    glm::vec2 closest = p0 + edgeDir * t;
    float wProj0 = 1.0f - (t / len);
    float wProj1 = t / len;

    glm::vec2 n = constraint.normal;
    float C = constraint.distance;

    // compliance handling
    float alpha = constraint.compliance / (dt * dt);

    float wSum = wA + wProj0 * wProj0 * w0 + wProj1 * wProj1 * w1;
    if (wSum < 1e-6f)
        return;

    float dlambda = (-C - alpha * constraint.lambda) / (wSum + alpha);
    glm::vec2 delta = dlambda * n;

    pA += wA * delta;
    p0 -= wProj0 * w0 * delta;
    p1 -= wProj1 * w1 * delta;

    constraint.lambda += dlambda;
}
