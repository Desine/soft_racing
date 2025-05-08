#pragma once
#include "soft_body.hpp"
#include <vector>
#include <memory>

struct DistanceJoint;
struct MotorJoint;

class PhysicsScene
{
    public:
    PhysicsScene() {};
    
    void Clear();
    
    glm::vec2 gravity = glm::vec2(0.0f, 0.0f);
    std::vector<std::shared_ptr<SoftBody>> softBodies;
    
    std::vector<std::shared_ptr<DistanceJoint>> distanceJoints;
    std::vector<std::shared_ptr<MotorJoint>> motorJoints;
};
