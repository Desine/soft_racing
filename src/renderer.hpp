#pragma once
#include <SFML/Graphics.hpp>
#include "level.hpp"
#include "soft_body.hpp"

class Renderer
{
public:
    sf::RenderWindow *window;

    void SetWindow(sf::RenderWindow *window);
    void DrawLevel(const Level &level, float carPositionX, float fov, float precision);
    void DrawSoftBody(const SoftBody &softBody);
    void DrawDistanceConstraints(PointMasses &pointMasses, std::vector<DistanceConstraint> &distanceConstraints);
    void DrawSoftBodies(std::vector<SoftBody> &softBodies);
};