#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include "Level.hpp"
#include "SoftBody.hpp"
#include "Renderer.hpp"



int main()
{
    // временно, нужно убрать позже
    Level level(1234);
    float carPositionX = 0.0f;
    float fov = 2000.0f;
    float precision = 10.0f;
    
    Renderer renderer;

    SoftBody triangle;
    triangle.AddPoint(glm::vec2(300, 300)); // точка 0
    triangle.AddPoint(glm::vec2(350, 300)); // точка 1
    triangle.AddPoint(glm::vec2(325, 250), 10.f, glm::vec2(100, 0)); // точка 2
    
    triangle.AddDistanceConstraint(0, 1);
    triangle.AddDistanceConstraint(1, 2);
    triangle.AddDistanceConstraint(2, 0, .5f);
    
    triangle.collisionPointMasses = {0, 1, 2};
       
    






    sf::RenderWindow window(sf::VideoMode({1400, 1400}), "Soft Racing");
    window.setFramerateLimit(60);
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

        renderer.DrawLevel(window, level, carPositionX, fov, precision);
        triangle.Simulate(1.f / 60.f, glm::vec2(0.0f, -9.8f));
        // triangle.ResolveGroundCollision(level);
        renderer.DrawSoftBody(window, triangle);


        window.draw(shape);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}

