#include "Renderer.hpp"
#include <iostream>

Renderer::Renderer()
{
    // возможно, инициализация шрифтов, текстур и т.п.
}

void Renderer::DrawLevel(sf::RenderWindow &window, const Level &level, float carPositionX, float fov, float precision)
{
    std::vector<glm::vec2> points = level.GetPoints(carPositionX, fov, precision);

    if (points.size() < 2)
    {
        std::cout << "ERROR at Renderer::DrawLevel: points.size() < 2" << std::endl;
        return;
    }

    sf::VertexArray terrain(sf::TriangleStrip, points.size() * 2);

    float groundHeight = -500.0f;

    for (size_t i = 0; i < points.size(); ++i)
    {
        float x = points[i].x;
        float y = points[i].y;

        // relief
        terrain[i * 2].position = sf::Vector2f(x, y);
        terrain[i * 2].color = sf::Color(100, 180, 100);

        // ground color
        terrain[i * 2 + 1].position = sf::Vector2f(x, y + groundHeight);
        terrain[i * 2 + 1].color = sf::Color(80, 120, 80);
    }

    window.draw(terrain);
}

void Renderer::DrawSoftBody(sf::RenderWindow &window, const SoftBody &softBody)
{
    // Draw points
    for (const auto &p : softBody.pointMasses)
    {
        sf::CircleShape circle(3.0f);
        circle.setOrigin(3.0f, 3.0f);
        circle.setPosition(p.position.x, p.position.y);
        circle.setFillColor(sf::Color::Red);
        window.draw(circle);
    }

    // Draw distance constraints
    for (const auto &c : softBody.distanceConstraints)
    {
        const auto &a = softBody.pointMasses[c.a];
        const auto &b = softBody.pointMasses[c.b];

        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(a.position.x, a.position.y), sf::Color::White),
            sf::Vertex(sf::Vector2f(b.position.x, b.position.y), sf::Color::White)};
        window.draw(line, 2, sf::Lines);
    }

    // Draw ground points and normals
    for (const auto &p : softBody.pointMasses)
    {
        Level level(1234);

        // point
        float groundY = level.GetHeight(p.position.x);
        sf::CircleShape circle(3.0f);
        circle.setOrigin(3.0f, 3.0f);
        circle.setPosition(p.position.x, groundY);
        circle.setFillColor(sf::Color::Red);
        window.draw(circle);

        // normal
        glm::vec2 normal = level.GetNormal(p.position.x);
        float lineLength = 30.0f;

        sf::VertexArray normalLine(sf::Lines, 2);
        normalLine[0].position = sf::Vector2f(p.position.x, groundY);
        normalLine[0].color = sf::Color::Green;
        normalLine[1].position = sf::Vector2f(
            p.position.x + normal.x * lineLength,
            groundY + normal.y * lineLength);
        normalLine[1].color = sf::Color::Green;

        window.draw(normalLine);
    }
}