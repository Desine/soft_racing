#pragma once
#include <vector>
#include "soft_body.hpp"
#include "joint_system.hpp"

class PhysicsScene
{
public:
    PhysicsScene(){};

    void Clear();

    glm::vec2 gravity = glm::vec2(0.0f, 0.0f);
    std::vector<SoftBody> softBodies;
    std::vector<DistanceJoint> distanceJoints;
};
