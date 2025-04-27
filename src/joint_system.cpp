#include "joint_system.hpp"
#include <iostream>

void ResetJointsLambdas(std::vector<DistanceJoint> &distanceJoints)
{
    for (auto &j : distanceJoints)
        for (auto &l : j.lambdas)
            l = 0.0f;
}

void SolveDistanceJoints(std::vector<DistanceJoint> &distanceJoints, float dt)
{
    for (auto &j : distanceJoints)
    {
        if (!j.softBody1 || !j.softBody2)
        {
            std::cerr << "Null softBody pointer in DistanceJoint!\n";
            continue;
        }
        auto &pm1 = j.softBody1->pointMasses;
        auto &pm2 = j.softBody2->pointMasses;

        for (size_t idx = 0; idx < j.indices1.size(); ++idx)
        {
            uint32_t i1 = j.indices1[idx];
            uint32_t i2 = j.indices2[idx];
            if (i1 >= pm1.inverseMasses.size() || i2 >= pm2.inverseMasses.size())
            {
                std::cerr << "Index out of range in DistanceJoint! i1=" << i1 << " i2=" << i2 << "\n";
                continue;
            }
            std::cout << "idx: " << idx << std::endl;
            std::cout << "j.indices1[idx]: " << j.indices1[idx] << " j.indices2[idx]: " << j.indices2[idx] << std::endl;
            std::cout << "j.indices1.size(): " << j.indices1.size() << " j.indices2.size(): " << j.indices2.size() << std::endl;
            std::cout << "pm1.positions.size(): " << pm1.positions.size() << " pm2.positions.size(): " << pm2.positions.size() << std::endl;
            std::cout << "pm1.inverseMasses.size(): " << pm1.inverseMasses.size() << " pm2.inverseMasses.size(): " << pm2.inverseMasses.size() << std::endl;

            auto &p1 = pm1.positions[i1];
            auto &p2 = pm2.positions[i2];

            float w1 = pm1.inverseMasses[i1];
            float w2 = pm2.inverseMasses[i2];

            glm::vec2 delta = p1 - p2;
            float len = glm::length(delta);
            if (len < 1e-6f)
                continue;

            float C = len - j.restDistances[idx];
            glm::vec2 grad = delta / len;

            float alphaTilde = j.compliances[idx] / (dt * dt);
            float denom = w1 + w2 + alphaTilde;
            float deltaLambda = (-C - alphaTilde * j.lambdas[idx]) / denom;
            j.lambdas[idx] += deltaLambda;

            glm::vec2 corr = deltaLambda * grad;

            p1 += w1 * corr;
            p2 -= w2 * corr;
        }
    }
}
