#include "joint_system.hpp"
#include "utils.hpp"
#include "renderer.hpp"
#include <glm/gtx/rotate_vector.hpp>
#include <SFML/Graphics/Color.hpp>
#include <iostream>
#include <algorithm>

void ResetJointsLambdas(PhysicsScene &physicsScene)
{
    for (auto &j : physicsScene.distanceJoints)
        j->lambda = 0.0f;
    for (auto &j : physicsScene.motorJoints)
        j->lambda = 0.0f;
}

void SolveDistanceJoints(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints, float dt)
{
    for (auto &j : distanceJoints)
    {
        auto sb1 = j->softBody1.lock();
        auto sb2 = j->softBody2.lock();

        if (!sb1 || !sb2)
            continue;

        glm::vec2 &p1 = sb1->pointMasses.positions[j->index1];
        glm::vec2 &p2 = sb2->pointMasses.positions[j->index2];

        float w1 = sb1->pointMasses.inverseMasses[j->index1];
        float w2 = sb2->pointMasses.inverseMasses[j->index2];

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

/*
найти anchor = центр anchorSoftBody[anchorIndices] + (anchorLocalOffset * матрица смещения от anchorStartPositions).
найти targetAngularVel = joint->targetRPM * 2.0f * M_PI / 60.0f.
пременить torque к точкам тел с учётом deltaLabmda и compliance.
обновить lambda.
*/

void SolveMotorJoints(std::vector<std::shared_ptr<MotorJoint>> motorJoints, float dt) {
    for (auto& joint : motorJoints) {
        auto anchorBody = joint->anchorSoftBody.lock();
        auto body1 = joint->softBody1.lock();
        auto body2 = joint->softBody2.lock();
        if (!anchorBody || !body1 || !body2) continue;

        // 1. Найти anchor
        glm::vec2 anchor(0.0f);
        for (auto idx : joint->anchorIndices) {
            anchor += anchorBody->pointMasses.positions[idx];
        }
        anchor /= static_cast<float>(joint->anchorIndices.size());

        glm::vec2 anchorOffset(0.0f);
        for (size_t i = 0; i < joint->anchorIndices.size(); ++i) {
            anchorOffset += joint->anchorStartPositions[i];
        }
        anchorOffset /= static_cast<float>(joint->anchorIndices.size());
        anchor += joint->anchorLocalOffset; // Предположим, anchorLocalOffset уже в мировой системе

        // Отобразить anchor
        Renderer::DrawCircle(anchor, 5.0f, sf::Color::Red);

        // 2. Применить torque
        float targetAnglePerStep = joint->targetRPM * 2.0f * 3.14159265f / 60.0f * dt;
        float compliance = joint->compliance;
        float alpha_tilde = compliance / (dt * dt);

        for (auto& softBody : { body1, body2 }) {
            if (!softBody) continue;
            for (auto idx : (softBody == body1 ? joint->indices1 : joint->indices2)) {
                auto& pos = softBody->pointMasses.positions[idx];
                auto& invMass = softBody->pointMasses.inverseMasses[idx];

                glm::vec2 r = pos - anchor;
                float r_len = glm::length(r);
                if (r_len < 1e-6f) continue;

                glm::vec2 tangent(-r.y, r.x); // ортогональ для вращения
                tangent = glm::normalize(tangent);

                float torquePerMass = joint->torque * r_len;
                float C = 0.0f; // целевая функция, можно доработать для угла
                float gradC = invMass * r_len * r_len;

                float deltaLambda = (-C - alpha_tilde * joint->lambda) / (gradC + alpha_tilde);
                joint->lambda += deltaLambda;

                pos += deltaLambda * invMass * tangent;

                // Отобразить линию смещения
                Renderer::DrawLine(anchor, pos, sf::Color::Green);
            }
        }
    }
}
