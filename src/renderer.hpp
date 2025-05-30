#pragma once
#include "level.hpp"
#include "soft_body.hpp"
#include "collision_system.hpp"
#include "joint_system.hpp"

#include <SFML/Graphics.hpp>
#include <vector>
#include <memory>

class Renderer
{
public:
    Renderer() = delete;

    static sf::RenderWindow *window;

    static void SetWindow(sf::RenderWindow *window);
    static void DrawLevel(const Level &level, float carPositionX, float fov, float precision);
    static void DrawDistanceConstraint(PointMasses &pointMasses, DistanceConstraint &distanceConstraint);
    static void DrawDistanceConstraints(PointMasses &pointMasses, std::vector<DistanceConstraint> &distanceConstraints);
    static void DrawDistanceJoints(std::vector<std::shared_ptr<DistanceJoint>> &distanceJoints);
    static void DrawDistanceJoint(DistanceJoint &distanceJoint);
    static void DrawSoftBodies(const std::vector<SoftBody> &softBodies);
    static void DrawSoftBody(const SoftBody &softBoby);

    static void DrawCircle(const glm::vec2 &pos, float radius, const sf::Color &color);
    static void DrawLine(const glm::vec2 &from, const glm::vec2 &to, const sf::Color &color);
    static void DrawSoftSoftPointEdgeCollision(const SoftSoftCollisionConstraint &constraint);
};