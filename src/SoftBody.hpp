#pragma once
#include "Level.hpp"
#include <vector>
#include <SFML/Graphics.hpp>
#include "glm/glm.hpp"

struct PointMass {
    glm::vec2 position;
    glm::vec2 previousPosition;
    float mass;
    glm::vec2 velocity;
    bool isFixed = false;
};

struct DistanceConstraint {
    int a, b;
    float restDistance;
    float compliance = 0.5f;
};

class SoftBody {
    public:
    std::vector<PointMass> pointMasses;
    std::vector<DistanceConstraint> distanceConstraints;
    std::vector<int> collisionPointMasses;
    
    void AddPoint(const glm::vec2& position, float mass = 1.f, const glm::vec2& velocity = glm::vec2(0,0), bool fixed = false);
    void AddDistanceConstraint(int a, int b, float compliance = 0.0f);
    void SolveDistanceConstraints(float deltaTime);
    void Simulate(float deltaTime, glm::vec2 gravity);
    void ResolveGroundCollision(const Level& level, float carPositionX, float fov, float precision);
    void Draw(sf::RenderWindow& window);
};
