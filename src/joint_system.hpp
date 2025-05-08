#pragma once
#include "soft_body.hpp"
#include "physics_scene.hpp"
#include "glm/glm.hpp"
#include <memory>
#include <vector>

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

struct MotorJoint
{
    std::weak_ptr<SoftBody> softBody1, softBody2, anchorSoftBody;
    std::vector<uint32_t> indices1, indices2, anchorIndices;
    glm::vec2 anchorLocalOffset = {0.0f, 0.0f};
    std::vector<glm::vec2> anchorStartPositions;
    float targetRPM;
    float torque;
    float compliance = 0.0f;
    float lambda = 0.0f;

    MotorJoint() = default;
    MotorJoint(std::weak_ptr<SoftBody> softBody1, std::weak_ptr<SoftBody> softBody2)
        : softBody1(softBody1), softBody2(softBody2) {}
};

void ResetJointsLambdas(PhysicsScene &physicsScene);
void SolveDistanceJoints(std::vector<std::shared_ptr<DistanceJoint>> distanceJoints, float dt);
void SolveMotorJoints(std::vector<std::shared_ptr<MotorJoint>> motorJoints, float dt);
