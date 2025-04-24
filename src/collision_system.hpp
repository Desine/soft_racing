#pragma once
#include "soft_body.hpp"
#include "glm/glm.hpp"

struct SoftSoftCollisionConstraint
{
    SoftBody *bodyA;
    SoftBody *bodyB;
    uint32_t pointIndex;
    uint32_t edgePointIndex0;
    uint32_t edgePointIndex1;
    float compliance;
    float lambda = 0.0f;
};

void DetectSoftSoftCollisions(
    const SoftBody &bodyA,
    const SoftBody &bodyB,
    float compliance,
    std::vector<SoftSoftCollisionConstraint> &outConstraints);
void SolveSoftSoftCollisionConstraint(SoftSoftCollisionConstraint &constraint, float dt);
