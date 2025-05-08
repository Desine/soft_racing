#include "collision_system.hpp"
#include "simulation.hpp"
#include "renderer.hpp"
#include "utils.hpp"

#include <algorithm>
#include <iostream>

void DetectSoftSoftCollisions(
    SoftBody &bodyA,
    SoftBody &bodyB,
    float compliance,
    float frictionStatic,
    float frictionKinetic,
    std::vector<SoftSoftCollisionConstraint> &outConstraints)
{
    const auto &positionsA = bodyA.pointMasses.positions;
    const auto &positionsB = bodyB.pointMasses.positions;
    const auto &shapeB = bodyB.collisionShape;

    if (shapeB.size() < 2)
        return;

    std::vector<uint32_t> insideB;
    for (uint32_t index : bodyA.collisionPoints)
    {
        auto hits = RaycastAllIntersections(positionsA[index], {1.0f, 0.0f}, bodyB);
        if (hits.size() % 2 == 1)
            insideB.push_back(index);
    }

    for (uint32_t indexA : insideB)
    {
        const glm::vec2 &pointA = positionsA[indexA];
        float minDist = std::numeric_limits<float>::max();
        uint32_t nearestEdge = 0;

        int shapeSize = shapeB.size();
        for (int i = 0; i < shapeSize; ++i)
        {
            const glm::vec2 &e1 = positionsB[shapeB[i]];
            const glm::vec2 &e2 = positionsB[shapeB[(i + 1) % shapeSize]];

            glm::vec2 edge = e2 - e1;
            float len = glm::length(edge);
            if (len < 1e-6f)
                continue;

            glm::vec2 dir = edge / len;
            glm::vec2 toPoint = pointA - e1;
            float proj = glm::clamp(glm::dot(toPoint, dir), 0.0f, len);
            glm::vec2 closest = e1 + dir * proj;
            float dist = glm::length(pointA - closest);

            if (dist < minDist)
            {
                minDist = dist;
                nearestEdge = i;
            }
        }

        SoftSoftCollisionConstraint constraint;
        constraint.softBodyA = &bodyA;
        constraint.softBodyB = &bodyB;
        constraint.pointIndex = indexA;
        constraint.edgePointIndex0 = shapeB[nearestEdge];
        constraint.edgePointIndex1 = shapeB[(nearestEdge + 1) % shapeB.size()];
        constraint.compliance = compliance;
        constraint.frictionStatic = frictionStatic,
        constraint.frictionKinetic = frictionKinetic,
        outConstraints.push_back(constraint);
    }
}

void SolveSoftSoftCollisionConstraint(
    SoftSoftCollisionConstraint &constraint,
    float dt)
{
    auto &p = constraint.softBodyA->pointMasses.positions[constraint.pointIndex];
    auto &p_prev = constraint.softBodyA->pointMasses.prevPositions[constraint.pointIndex];
    auto p_w = constraint.softBodyA->pointMasses.inverseMasses[constraint.pointIndex];

    auto &e0 = constraint.softBodyB->pointMasses.positions[constraint.edgePointIndex0];
    auto &e1 = constraint.softBodyB->pointMasses.positions[constraint.edgePointIndex1];
    auto e0_w = constraint.softBodyB->pointMasses.inverseMasses[constraint.edgePointIndex0];
    auto e1_w = constraint.softBodyB->pointMasses.inverseMasses[constraint.edgePointIndex1];

    glm::vec2 edge = e1 - e0;
    float edgeLengthSq = glm::dot(edge, edge);
    if (edgeLengthSq < 1e-6f)
        return;

    float t = glm::clamp(glm::dot(p - e0, edge) / edgeLengthSq, 0.0f, 1.0f);
    glm::vec2 closestPoint = e0 + t * edge;

    glm::vec2 n = p - closestPoint;
    float С = glm::length(n);
    if (С < 1e-6f)
        return;
    n /= С;

    glm::vec2 grad_p = n;
    glm::vec2 grad_e0 = -n * (1.0f - t);
    glm::vec2 grad_e1 = -n * t;

    float w_sum = p_w + e0_w * (1.0f - t) * (1.0f - t) + e1_w * t * t;
    if (w_sum < 1e-6f)
        return;

    float alphaTilde = constraint.compliance / (dt * dt);
    float deltaLambda = (-С - alphaTilde * constraint.lambda) / (w_sum + alphaTilde);
    constraint.lambda += deltaLambda;

    p += p_w * deltaLambda * grad_p;
    e0 += e0_w * deltaLambda * grad_e0;
    e1 += e1_w * deltaLambda * grad_e1;

    // Static friction
    glm::vec2 tangent = glm::vec2(-n.y, n.x);
    glm::vec2 p_disp = p - p_prev;

    glm::vec2 e0_prev = constraint.softBodyB->pointMasses.prevPositions[constraint.edgePointIndex0];
    glm::vec2 e1_prev = constraint.softBodyB->pointMasses.prevPositions[constraint.edgePointIndex1];
    glm::vec2 e0_disp = e0 - e0_prev;
    glm::vec2 e1_disp = e1 - e1_prev;

    glm::vec2 e_disp = e0_disp * (1.0f - t) + e1_disp * t;
    glm::vec2 relative_disp = p_disp - e_disp;
    float tangential_disp = glm::dot(relative_disp, tangent);

    float max_static_friction = constraint.frictionStatic * fabs(deltaLambda);

    float tangential_correction = glm::clamp(-tangential_disp, -max_static_friction, max_static_friction);

    p += p_w / w_sum * tangential_correction * tangent;
    e0 -= e0_w * (1.0f - t) / w_sum * tangential_correction * tangent;
    e1 -= e1_w * t / w_sum * tangential_correction * tangent;
}
