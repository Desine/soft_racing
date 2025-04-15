#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include "Level.hpp"


void DrawLevel(sf::RenderWindow &window, const Level &level, float carX)
{
    float width = 800; // ширина экрана
    float precision = 5.0f;

    std::vector<Vec2> points = level.GetPoints(carX, width, precision);

    sf::VertexArray lines(sf::LineStrip, points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        lines[i].position = sf::Vector2f(points[i].x - carX + width / 2, 600 - points[i].y);
        lines[i].color = sf::Color::White;
    }

    window.draw(lines);
}


int main()
{
    // временно, нужно убрать позже
    Level level(1234);
    float carX = 0.0f;


    sf::RenderWindow window(sf::VideoMode({1400, 1400}), "SFML works!");
    bool success = ImGui::SFML::Init(window);

    sf::CircleShape shape(100.f);
    shape.setFillColor(sf::Color::Green);

    sf::Clock deltaClock;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);

            if (event.type == sf::Event::Closed)
                window.close();
        }

        ImGui::SFML::Update(window, deltaClock.restart());

        ImGui::Begin("Hello, ImGui!");
        ImGui::Text("This is some text");
        ImGui::End();

        window.clear();

        DrawLevel(window, level, carX);
        window.draw(shape);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}

