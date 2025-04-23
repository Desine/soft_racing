#pragma once
#include <SFML/Graphics.hpp>
#include "level.hpp"
#include "soft_body.hpp"
#include "collision_system.hpp"

class Renderer
{
public:
    Renderer() = delete;

    static sf::RenderWindow *window;

    static void SetWindow(sf::RenderWindow *window);
    static void DrawLevel(const Level &level, float carPositionX, float fov, float precision);
    static void DrawSoftBody(const SoftBody &softBody);
    static void DrawDistanceConstraints(PointMasses &pointMasses, std::vector<DistanceConstraint> &distanceConstraints);
    static void DrawSoftBodies(std::vector<SoftBody> &softBodies);

    static void DrawCircle(const glm::vec2 &pos, float radius, const sf::Color &color);
    static void DrawLine(const glm::vec2 &from, const glm::vec2 &to, const sf::Color &color);
    static void DrawDebugCollision(const SoftBody &bodyA, const SoftBody &bodyB, const SoftSoftCollisionConstraint &constraint);
};