#pragma once
#include "soft_body.hpp"
#include <memory>

struct DistanceJoint
{
    std::weak_ptr<SoftBody> softBody1, softBody2;
    uint32_t index1, index2;
    float restDistance;
    float compliance = 0.0f;
    float lambda = 0.0f;

    DistanceJoint() = default;

    DistanceJoint(std::weak_ptr<SoftBody> softBody1, std::weak_ptr<SoftBody> softBody2)
        : softBody1(softBody1), softBody2(softBody2) {}
};

void ResetJointsLambdas(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints);
void SolveDistanceJoints(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints, float dt);
