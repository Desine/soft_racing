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
