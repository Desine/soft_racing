#pragma once
#include "soft_body.hpp"

struct DistanceJoint
{
    SoftBody *softBody1 = nullptr;
    SoftBody *softBody2 = nullptr;
    std::vector<uint32_t> indices1, indices2;
    std::vector<float> restDistances;
    std::vector<float> compliances;
    std::vector<float> lambdas;

    DistanceJoint() = default;

    DistanceJoint(SoftBody *softBody1, SoftBody *softBody2)
        : softBody1(softBody1), softBody2(softBody2) {}
};

void ResetJointsLambdas(std::vector<DistanceJoint> &distanceJoints);
void SolveDistanceJoints(std::vector<DistanceJoint> &distanceJoints, float dt);
