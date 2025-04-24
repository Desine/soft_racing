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
    std::vector<SoftSoftCollisionConstraint> &outConstraints)
{
    std::vector<uint32_t> insideB;
    for (uint32_t i = 0; i < bodyA.pointMasses.positions.size(); ++i)
    {
        const glm::vec2 &point = bodyA.pointMasses.positions[i];
        glm::vec2 rayDir(1.0f, 0.0f);
        auto hits = RaycastAllIntersections(point, rayDir, bodyB);

        if (hits.size() % 2 == 1)
            insideB.push_back(i);
    }

    for (uint32_t indexA : insideB)
    {
        const glm::vec2 &pointA = bodyA.pointMasses.positions[indexA];
        const auto &shapeB = bodyB.collisionShape;
        const auto &positionsB = bodyB.pointMasses.positions;

        if (shapeB.empty())
            continue;

        float minEdgeDist = std::numeric_limits<float>::max();
        uint32_t nearestEdge = 0;
        int shapeBSize = static_cast<int>(shapeB.size());

        for (int i = 0; i < shapeBSize; ++i)
        {
            const glm::vec2 &e1 = positionsB[shapeB[i]];
            const glm::vec2 &e2 = positionsB[shapeB[(i + 1) % shapeBSize]];
            glm::vec2 edge = e2 - e1;

            float edgeLen = glm::length(edge);
            if (edgeLen < 1e-6f)
                continue;

            glm::vec2 edgeDir = glm::normalize(edge);
            glm::vec2 toPoint = pointA - e1;
            float proj = glm::dot(toPoint, edgeDir);
            proj = glm::clamp(proj, 0.0f, edgeLen);
            glm::vec2 closest = e1 + edgeDir * proj;
            float edgeDist = glm::length(pointA - closest);

            if (edgeDist < minEdgeDist)
            {
                minEdgeDist = edgeDist;
                nearestEdge = i;
            }
        }

        SoftSoftCollisionConstraint constraint;
        constraint.bodyA = const_cast<SoftBody *>(&bodyA);
        constraint.bodyB = const_cast<SoftBody *>(&bodyB);
        constraint.pointIndex = indexA;
        constraint.edgePointIndex0 = nearestEdge;
        constraint.edgePointIndex1 = (nearestEdge + 1) % shapeBSize;
        constraint.compliance = compliance;
        outConstraints.push_back(constraint);
    }
}

void SolveSoftSoftCollisionConstraint(
    SoftSoftCollisionConstraint &constraint,
    float dt)
{
    auto &p = constraint.bodyA->pointMasses.positions[constraint.pointIndex];
    auto &w_p = constraint.bodyA->pointMasses.inverseMasses[constraint.pointIndex];

    auto &e0 = constraint.bodyB->pointMasses.positions[constraint.edgePointIndex0];
    auto &e1 = constraint.bodyB->pointMasses.positions[constraint.edgePointIndex1];
    auto &w_e0 = constraint.bodyB->pointMasses.inverseMasses[constraint.edgePointIndex0];
    auto &w_e1 = constraint.bodyB->pointMasses.inverseMasses[constraint.edgePointIndex1];
    
     glm::vec2 edge = e1 - e0;
     float edgeLengthSq = glm::dot(edge, edge);
     if (edgeLengthSq < 1e-6f) return;
 
     float t = glm::clamp(glm::dot(p - e0, edge) / edgeLengthSq, 0.0f, 1.0f);
     glm::vec2 closestPoint = e0 + t * edge;
 
     glm::vec2 n = p - closestPoint;
     float dist = glm::length(n);
     if (dist < 1e-6f) return;
     n /= dist;
 
     float C = dist;
 
     glm::vec2 grad_p = n;
     glm::vec2 grad_e0 = -n * (1 - t);
     glm::vec2 grad_e1 = -n * t;
 
     float w_sum =
         w_p * glm::dot(grad_p, grad_p) +
         w_e0 * glm::dot(grad_e0, grad_e0) +
         w_e1 * glm::dot(grad_e1, grad_e1);
 
     float alphaTilde = constraint.compliance / (dt * dt);
     float deltaLambda = (-C - alphaTilde * constraint.lambda) / (w_sum + alphaTilde);
     constraint.lambda += deltaLambda;
 
     p += w_p * deltaLambda * grad_p;
     e0 += w_e0 * deltaLambda * grad_e0;
     e1 += w_e1 * deltaLambda * grad_e1;
}