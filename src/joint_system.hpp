#pragma once
#include "soft_body.hpp"
#include <memory>

struct DistanceJoint
{
    std::weak_ptr<SoftBody> softBody1, softBody2;
    std::vector<uint32_t> indices1, indices2;
    std::vector<float> restDistances;
    std::vector<float> compliances;
    std::vector<float> lambdas;

    DistanceJoint() = default;

    DistanceJoint(std::weak_ptr<SoftBody> softBody1, std::weak_ptr<SoftBody> softBody2)
        : softBody1(softBody1), softBody2(softBody2) {}
};

void ResetJointsLambdas(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints);
void SolveDistanceJoints(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints, float dt);
