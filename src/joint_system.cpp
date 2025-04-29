#include "joint_system.hpp"
#include <iostream>


void ResetJointsLambdas(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints)
{
    for (auto j : distanceJoints)
        for (auto l : j->lambdas)
            l = 0.0f;
}

void SolveDistanceJoints(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints, float dt)
{
    for (auto j : distanceJoints)
    {
        auto sb1 = j->softBody1.lock();
        auto sb2 = j->softBody2.lock();
        if (!sb1 || !sb2)
        {
            std::cerr << "SoftBody expired in DistanceJoint!\n";
            continue;
        }

        auto &pm1 = sb1->pointMasses;
        auto &pm2 = sb2->pointMasses;

        for (size_t idx = 0; idx < j->indices1.size(); ++idx)
        {
            uint32_t i1 = j->indices1[idx];
            uint32_t i2 = j->indices2[idx];
            
            auto &p1 = pm1.positions[i1];
            auto &p2 = pm2.positions[i2];

            float w1 = pm1.inverseMasses[i1];
            float w2 = pm2.inverseMasses[i2];

            glm::vec2 delta = p1 - p2;
            float len = glm::length(delta);
            if (len < 1e-6f)
                continue;

            float C = len - j->restDistances[idx];
            glm::vec2 grad = delta / len;

            float alphaTilde = j->compliances[idx] / (dt * dt);
            float denom = w1 + w2 + alphaTilde;
            float deltaLambda = (-C - alphaTilde * j->lambdas[idx]) / denom;
            j->lambdas[idx] += deltaLambda;

            glm::vec2 corr = deltaLambda * grad;

            p1 += w1 * corr;
            p2 -= w2 * corr;
        }
    }
}
