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
    softBody.Clear();

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
}

int main()
{
    Level level(1234);
    float carPositionX = 0.0f;
    float fov = 2000.0f;
    float precision = 10.0f;

    const glm::vec2 &center = glm::vec2(300, 300);
    float wheelRadius = 200;
    float diskMass = 20;
    float tireMass = 5;
    float tireRatio = .4f;
    float diskHubCompliance = .0001f;
    float diskRimCompliance = .0001f;
    float tireBodyCompliance = .01f;
    float tireTreadCompliance = .001f;
    float tirePressureCompliance = .0f;
    float tirePressure = 1.f;
    int radialSegments = 12;
    glm::vec2 gravity = glm::vec2(0.0f, 0.0f);

    Renderer renderer;

    SoftBody softBody;
    ResetPoligon(softBody);
    std::cout << softBody.volumeConstraints.back().restVolume << std::endl;

    sf::RenderWindow window(sf::VideoMode({2000, 2000}), "Soft Racing");
    window.setFramerateLimit(600);
    bool success = ImGui::SFML::Init(window);
    if (success)
    {
        ImGui::GetStyle().ScaleAllSizes(3.f);
        ImGui::GetIO().FontGlobalScale = 2.f;
    }

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
        // ImGui::Text("some text..");

        if (ImGui::Button("EXIT"))
            window.close();

        if (ImGui::Button("Reset"))
        {
            CreateWheel(softBody,
                        center,
                        wheelRadius,
                        diskMass,
                        tireMass,
                        tireRatio,
                        diskHubCompliance,
                        diskRimCompliance,
                        tireBodyCompliance,
                        tireTreadCompliance,
                        tirePressureCompliance,
                        tirePressure,
                        radialSegments);
            // ResetPoligon(softBody);
        }

        ImGui::SliderFloat("Gravity", &gravity.y, -1000.f, 1000.f);
        ImGui::SliderInt("Substeps", &softBody.simulationSubsteps, 1, 10);

        ImGui::SliderFloat("Wheel Radius", &wheelRadius, 50.f, 500.f);
        ImGui::SliderFloat("Disk Mass", &diskMass, 1.f, 100.f);
        ImGui::SliderFloat("Tire Mass", &tireMass, 1.f, 100.f);
        ImGui::SliderFloat("Tire Ratio", &tireRatio, 0.1f, 0.9f);

        ImGui::SliderFloat("Disk Hub Compliance", &diskHubCompliance, 0.0001f, 1.f);
        ImGui::SliderFloat("Disk Compliance", &diskRimCompliance, 0.0001f, 1.f);
        ImGui::SliderFloat("Tire Body Compliance", &tireBodyCompliance, 0.0001f, 1.f);
        ImGui::SliderFloat("Tire Tread Compliance", &tireTreadCompliance, 0.0001f, 1.f);
        ImGui::SliderFloat("Tire Pressure Compliance", &tirePressureCompliance, 0.0001f, 1.f);
        ImGui::SliderFloat("Tire Pressure", &tirePressure, 0.f, 10.f);

        ImGui::SliderInt("Radial Segments", &radialSegments, 3, 32);

        ImGui::End();

        window.clear();

        renderer.DrawLevel(window, level, carPositionX, fov, precision);
        softBody.Simulate(dt.asSeconds(), gravity);
        softBody.SolveGroundCollision(level);
        renderer.DrawSoftBody(window, softBody);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}
