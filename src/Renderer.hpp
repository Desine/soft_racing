#pragma once
#include <SFML/Graphics.hpp>
#include "Level.hpp"
#include "SoftBody.hpp"

class Renderer
{
public:
    Renderer();

    void DrawLevel(sf::RenderWindow &window, const Level &level, float carPositionX, float fov, float precision);
    void DrawSoftBody(sf::RenderWindow& window, const SoftBody& softBody);
};