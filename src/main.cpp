#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include <iostream>

#include "Level.hpp"
#include "SoftBody.hpp"
#include "Renderer.hpp"
#include "CarTools.hpp"

void ResetPoligon(SoftBody &softBody)
{
    softBody = *new SoftBody();

    softBody.AddPoint(glm::vec2(300, 300), 3.f, glm::vec2(30, 0));
    softBody.AddPoint(glm::vec2(400, 300));
    softBody.AddPoint(glm::vec2(400, 400));
    softBody.AddPoint(glm::vec2(300, 400));

    softBody.AddDistanceConstraint(0, 1, 0.3f);
    softBody.AddDistanceConstraint(1, 2, 0.3f);
    softBody.AddDistanceConstraint(2, 3, 0.3f);
    softBody.AddDistanceConstraint(3, 0, 0.3f);

    softBody.AddVolumeConstraint({0, 1, 2, 3}, 0.2f);

    softBody.collisionPointMasses = {0, 1, 2};

    CreateWheel(softBody, 
        glm::vec2(300, 300),
        200,
        10,
        3,
        .3f,
        .2f,
        .1f,
        1.f,
        10);
}

int main()
{
    Level level(1234);
    float carPositionX = 0.0f;
    float fov = 2000.0f;
    float precision = 10.0f;

    Renderer renderer;

    SoftBody softBody;
    ResetPoligon(softBody);
    std::cout << softBody.volumeConstraints.back().restVolume << std::endl;

    sf::RenderWindow window(sf::VideoMode({1400, 1400}), "Soft Racing");
    window.setFramerateLimit(600);
    bool success = ImGui::SFML::Init(window);
    if (success)
    {
        ImGui::GetStyle().ScaleAllSizes(4.f);
        ImGui::GetIO().FontGlobalScale = 2.5f;
    }

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

        sf::Time dt = deltaClock.restart();
        ImGui::SFML::Update(window, dt);

        ImGui::Begin("ImGui Begin Title");
        ImGui::Text("some text..");

        ImGui::SliderFloat("Distance Compliance", &softBody.distanceConstraints[0].compliance, 0.f, 1.f);
        ImGui::SliderFloat("Distance Rest Length", &softBody.distanceConstraints[0].restDistance, 0.1f, 1000.0f);
        ImGui::SliderFloat("Volume Compliance", &softBody.volumeConstraints[0].compliance, 0.f, 1.f);
        ImGui::SliderFloat("Rest Volume", &softBody.volumeConstraints[0].restVolume, .1f, 500000.f);
        ImGui::SliderInt("Substeps", &softBody.simulationSubsteps, 1, 10);

        if (ImGui::Button("Reset Triangle"))
            ResetPoligon(softBody);

        ImGui::End();

        window.clear();

        renderer.DrawLevel(window, level, carPositionX, fov, precision);
        softBody.Simulate(dt.asSeconds(), glm::vec2(0.0f, -0.0f));
        // softBody.ResolveGroundCollision(level);
        renderer.DrawSoftBody(window, softBody);

        window.draw(shape);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}
