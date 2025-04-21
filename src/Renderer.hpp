#pragma once
#include <SFML/Graphics.hpp>
#include "Level.hpp"
#include "SoftBody.hpp"

class Renderer
{
public:
    sf::RenderWindow *window;

    void SetWindow(sf::RenderWindow *window);
    void DrawLevel(const Level &level, float carPositionX, float fov, float precision);
    void DrawSoftBody(const SoftBody &softBody);
    // void DrawDistanceConstraints(PointMass pointMasses, DistanceConstraint distanceConstraints);
};