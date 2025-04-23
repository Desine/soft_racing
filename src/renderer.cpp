#include "renderer.hpp"
#include <iostream>
#include "glm/glm.hpp"

sf::RenderWindow *Renderer::window = nullptr;

void Renderer::SetWindow(sf::RenderWindow *window)
{
    Renderer::window = window;
}

void Renderer::DrawLevel(const Level &level, float carPositionX, float fov, float precision)
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

    window->draw(terrain);
}

void Renderer::DrawSoftBody(const SoftBody &softBody)
{
    /*
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
            */
}

void Renderer::DrawDistanceConstraints(PointMasses &pointMasses, std::vector<DistanceConstraint> &distanceConstraints)
{
    for (const DistanceConstraint &c : distanceConstraints)
    {
        glm::vec2 &positionA = pointMasses.positions[c.i1];
        glm::vec2 &positionB = pointMasses.positions[c.i2];

        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(positionA.x, positionA.y), sf::Color::White),
            sf::Vertex(sf::Vector2f(positionB.x, positionB.y), sf::Color::White)};
        window->draw(line, 2, sf::Lines);
    }
}

void Renderer::DrawSoftBodies(std::vector<SoftBody> &softBodies)
{
    for (SoftBody &s : softBodies)
        DrawSoftBody(s);
}
void Renderer::DrawSoftBody(SoftBody &softBoby)
{
    auto &positions = softBoby.pointMasses.positions;
    auto &shape = softBoby.collisionPoints;
    int shape_n = softBoby.collisionShape.size();

    for (const DistanceConstraint &c : softBoby.distanceConstraints)
    {
        glm::vec2 &positionA = softBoby.pointMasses.positions[c.i1];
        glm::vec2 &positionB = softBoby.pointMasses.positions[c.i2];

        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(positionA.x, positionA.y), sf::Color::Cyan),
            sf::Vertex(sf::Vector2f(positionB.x, positionB.y), sf::Color::Cyan)};
        window->draw(line, 2, sf::Lines);
    }

    for (int i = 0; i < shape_n; ++i)
    {
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(positions[shape[i]].x, positions[shape[i]].y), sf::Color::White),
            sf::Vertex(sf::Vector2f(positions[shape[(i + 1) % shape_n]].x, positions[shape[(i + 1) % shape_n]].y), sf::Color::White)};
        window->draw(line, 2, sf::Lines);
    }
}

void Renderer::DrawCircle(const glm::vec2 &pos, float radius, const sf::Color &color)
{
    sf::CircleShape shape(radius);
    shape.setFillColor(color);
    shape.setOrigin(radius, radius);
    shape.setPosition(pos.x, pos.y);
    window->draw(shape);
}

void Renderer::DrawLine(const glm::vec2 &from, const glm::vec2 &to, const sf::Color &color)
{
    sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f(from.x, from.y), color),
            sf::Vertex(sf::Vector2f(to.x, to.y), color)};
    window->draw(line, 2, sf::Lines);
}

void Renderer::DrawDebugCollision(const SoftBody &bodyA, const SoftBody &bodyB, const SoftSoftCollisionConstraint &constraint)
{
    const glm::vec2 &pA = bodyA.pointMasses.positions[constraint.pointIndexA];

    DrawCircle(pA, 3.0f, sf::Color::Red);

    const auto &shapeB = bodyB.collisionShape;
    const auto &posB = bodyB.pointMasses.positions;
    glm::vec2 e1 = posB[shapeB[constraint.edgeIndexB]];
    glm::vec2 e2 = posB[shapeB[(constraint.edgeIndexB + 1) % shapeB.size()]];
    DrawLine(e1, e2, sf::Color::Yellow);

    glm::vec2 normalEnd = pA + constraint.normal * 20.0f;
    DrawLine(pA, normalEnd, sf::Color::White);
}