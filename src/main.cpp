#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include <iostream>

#include "Level.hpp"
#include "SoftBody.hpp"
#include "Renderer.hpp"
#include "CarTools.hpp"

const int WINDOW_WIDTH = 2000;
const int WINDOW_HEIGHT = 2000;


int main()
{
    Level level(1234);
    float carPositionX = 0.0f;
    float fov = 2000.0f;
    float precision = 10.0f;
    
    float simulationSpeed = 10.f;
    glm::vec2 gravity = glm::vec2(0.0f, 0.0f);

    glm::vec2 center = glm::vec2(500, 1000);
    float wheelRadius = 200;
    float diskMass = 20;
    float tireMass = 5;
    float tireRatio = .4f;
    float diskHubCompliance = .0f;
    float diskRimCompliance = .0f;
    float tireBodyCompliance = .02f;
    float tireTreadCompliance = .05f;
    float tirePressureCompliance = .01f;
    float tirePressure = 1.f;
    int radialSegments = 12;

    Renderer renderer;

    SoftBody softBody;
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
    // std::cout << "Rest Volume: " <<softBody.volumeConstraints.back().restVolume << std::endl;

    sf::RenderWindow window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}), "Soft Racing");
    window.setFramerateLimit(140);
    sf::View view = window.getDefaultView();
    view.setSize(WINDOW_WIDTH, -WINDOW_HEIGHT);
    window.setView(view);
    
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

        ImGui::SliderFloat("Simulation Speed", &simulationSpeed, 0.001f, 100.f);
        // ImGui::SliderInt("Substeps", &softBody.simulationSubsteps, 1, 10);
        ImGui::SliderFloat("Gravity Y", &gravity.y, -100.f, 100.f);
        ImGui::SliderFloat("Gravity X", &gravity.x, -100.f, 100.f);

        ImGui::SliderFloat("Wheel Radius", &wheelRadius, 50.f, 500.f);
        ImGui::SliderFloat("Disk Mass", &diskMass, 1.f, 100.f);
        ImGui::SliderFloat("Tire Mass", &tireMass, 1.f, 100.f);
        ImGui::SliderFloat("Tire Ratio", &tireRatio, 0.1f, 0.9f);

        ImGui::SliderFloat("Disk Hub Compliance", &diskHubCompliance, .0f, 1.f);
        ImGui::SliderFloat("Disk Compliance", &diskRimCompliance, .0f, 1.f);
        ImGui::SliderFloat("Tire Body Compliance", &tireBodyCompliance, .0f, 1.f);
        ImGui::SliderFloat("Tire Tread Compliance", &tireTreadCompliance, .0f, 1.f);
        ImGui::SliderFloat("Tire Pressure Compliance", &tirePressureCompliance, .0f, 1.f);
        ImGui::SliderFloat("Tire Pressure", &tirePressure, 0.f, 10.f);

        ImGui::SliderInt("Radial Segments", &radialSegments, 3, 32);

        ImGui::End();

        sf::Vector2f cameraCenter;
        // cameraCenter.x = softBody.pointMasses[0].position.x;
        // cameraCenter.y = softBody.pointMasses[0].position.y;
        view.setCenter(cameraCenter);

        window.clear();
        window.setView(view);
        // carPositionX = softBody.pointMasses[0].position.x;
        renderer.DrawLevel(window, level, carPositionX, fov, precision);
        // softBody.Simulate(dt.asSeconds() * simulationSpeed, gravity);
        softBody.SolveGroundCollision(level);
        renderer.DrawSoftBody(window, softBody);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}
