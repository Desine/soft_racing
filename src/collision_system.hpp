#pragma once
#include "soft_body.hpp"
#include "glm/glm.hpp"

struct SoftSoftCollisionConstraint
{
    SoftBody *bodyA;
    SoftBody *bodyB;
    uint32_t pointIndexA;
    uint32_t edgeIndexB;
    glm::vec2 normal;
    float distance;
    float compliance;
    float lambda;
};

void DetectSoftSoftCollisions(
    const SoftBody &bodyA,
    const SoftBody &bodyB,
    float compliance,
    float minDistanceThreshold,
    std::vector<SoftSoftCollisionConstraint> &outConstraints);