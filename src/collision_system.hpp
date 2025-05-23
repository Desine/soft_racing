#pragma once
#include "soft_body.hpp"
#include "glm/glm.hpp"

struct SoftSoftCollisionConstraint
{
    SoftBody *softBodyA;
    SoftBody *softBodyB;
    uint32_t pointIndex;
    uint32_t edgePointIndex0;
    uint32_t edgePointIndex1;
    float compliance = 0.0f;
    float lambda = 0.0f;

    float frictionStatic = 0.5f;
    float frictionKinetic = 0.3f;
};

void DetectSoftSoftCollisions(
    SoftBody &softBodyA,
    SoftBody &softBodyB,
    float compliance,
    float frictionStatic,
    float frictionKinetic,
    std::vector<SoftSoftCollisionConstraint> &outConstraints);
void SolveSoftSoftCollisionConstraint(SoftSoftCollisionConstraint &constraint, float dt);
