#include "joint_system.hpp"
#include <iostream>

void ResetJointsLambdas(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints)
{
    for (auto j : distanceJoints)
        j->lambda = 0.0f;
}

void SolveDistanceJoints(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints, float dt)
{
    for (auto &j : distanceJoints)
    {
        auto softBody1 = j->softBody1.lock();
        auto softBody2 = j->softBody2.lock();

        if (!softBody1 || !softBody2)
            continue;

        glm::vec2 &p1 = softBody1->pointMasses.positions[j->index1];
        glm::vec2 &p2 = softBody2->pointMasses.positions[j->index2];

        float w1 = softBody1->pointMasses.inverseMasses[j->index1];
        float w2 = softBody2->pointMasses.inverseMasses[j->index2];

        glm::vec2 delta = p1 - p2;
        float len = glm::length(delta);
        if (len < 1e-6f)
            continue;

        float C = len - j->restDistance;
        glm::vec2 grad = delta / len;

        float alphaTilde = j->compliance / (dt * dt);
        float denom = w1 + w2 + alphaTilde;
        float deltaLambda = (-C - alphaTilde * j->lambda) / denom;
        j->lambda += deltaLambda;

        p1 += w1 * deltaLambda * grad;
        p2 -= w2 * deltaLambda * grad;
    }
}
