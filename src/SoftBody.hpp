#pragma once
#include "Level.hpp"
#include <vector>
#include <SFML/Graphics.hpp>
#include "glm/glm.hpp"

struct PointMass {
    glm::vec2 position;
    glm::vec2 previousPosition;
    glm::vec2 velocity;
    float mass;
    float inverseMass;
    bool isFixed = false;
};

struct DistanceConstraint {
    int a, b;
    float restDistance;
    float compliance;
};

struct VolumeConstraint {
    std::vector<int> ids;
    float restVolume;
    float compliance;
};

class SoftBody {
    public:
    std::vector<PointMass> pointMasses;
    std::vector<DistanceConstraint> distanceConstraints;
    std::vector<int> collisionPointMasses;
    std::vector<VolumeConstraint> volumeConstraints;
    
    void AddPoint(const glm::vec2& position, float mass = 1.f, const glm::vec2& velocity = glm::vec2(0,0), bool fixed = false);
    void AddDistanceConstraint(int a, int b, float compliance = 0.0f);
    void AddVolumeConstraint(const std::vector<int>& pointIndices, float compliance = 0.0f);
    void AddVolumeConstraint(const std::vector<int>& pointIndices, float compliance, float volume);
    void SolveDistanceConstraints(float deltaTime);
    void SolveVolumeConstraints(float deltaTime);
    void Simulate(float deltaTime, glm::vec2 gravity = glm::vec2(0.0f, -9.8f));
    void ResolveGroundCollision(const Level& level, float carPositionX, float fov, float precision);
};
