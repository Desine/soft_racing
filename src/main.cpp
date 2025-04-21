#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include <iostream>

#include "CollisionSolver.hpp"
#include "Simulation.hpp"
#include "SoftBody.hpp"
#include "Renderer.hpp"
#include "CarTools.hpp"
#include "Level.hpp"

const int WINDOW_WIDTH = 2000;
const int WINDOW_HEIGHT = 2000;

SoftBody CreateSoftSquare()
{
    SoftBody body;

    float spacing = 100.0f;
    glm::vec2 origin(1000.0f, 1000.0f);

    // Позиции (4 точки квадрата)
    body.pointMasses.positions = {
        origin,
        origin + glm::vec2(spacing, 0),
        origin + glm::vec2(spacing, spacing),
        origin + glm::vec2(0, spacing)};

    body.pointMasses.prevPositions = body.pointMasses.positions;
    body.pointMasses.velocities.resize(4, glm::vec2(0.0f));
    body.pointMasses.inverseMasses = {1.0f, 1.0f, 1.0f, 1.0f};

    body.pointMasses.velocities[0].y = -10.f;

    // Distance constraints (ребра квадрата + диагонали)
    auto &d = body.distanceConstraints;
    d.push_back({0, 1, spacing, 1e-5f, 0});
    d.push_back({1, 2, spacing, 1e-5f, 0});
    d.push_back({2, 3, spacing, 1e-5f, 0});
    d.push_back({3, 0, spacing, 1e-5f, 0});
    d.push_back({0, 2, spacing * std::sqrt(2.0f), 1e-5f, 0});
    d.push_back({1, 3, spacing * std::sqrt(2.0f), 1e-5f, 0});

    // Volume constraint (все 4 точки)
    VolumeConstraint vc;
    vc.indices = {0, 1, 2, 3};
    vc.restVolume = ComputePolygonArea(body.pointMasses.positions, vc.indices);
    vc.compliance = 1e-5f;
    vc.lambda = 0;
    body.volumeConstraints.push_back(vc);

    return body;
}

int main()
{
    Level level(1234);
    float carPositionX = 0.0f;
    float fov = 2000.0f;
    float precision = 10.0f;

    glm::vec2 gravity = glm::vec2(0.0f, -9.8f);
    float simulationSpeed = 10.f;
    const float dt = 1.0f / 60.0f;
    int solverSubsteps = 1;
    int solverIterations = 2;
    
    SoftBody softBody = CreateSoftSquare();
    sf::VertexArray lines(sf::Lines, 12);

    // glm::vec2 center = glm::vec2(500, 1000);
    // float wheelRadius = 200;
    // float diskMass = 20;
    // float tireMass = 5;
    // float tireRatio = .4f;
    // float diskHubCompliance = .0f;
    // float diskRimCompliance = .0f;
    // float tireBodyCompliance = .02f;
    // float tireTreadCompliance = .05f;
    // float tirePressureCompliance = .01f;
    // float tirePressure = 1.f;
    // int radialSegments = 12;

    // SoftBody softBody = CreateWheel(center,
    //             wheelRadius,
    //             diskMass,
    //             tireMass,
    //             tireRatio,
    //             diskHubCompliance,
    //             diskRimCompliance,
    //             tireBodyCompliance,
    //             tireTreadCompliance,
    //             tirePressureCompliance,
    //             tirePressure,
    //             radialSegments);


    sf::RenderWindow window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}), "Soft Racing");
    window.setFramerateLimit(60);
    sf::View view = window.getDefaultView();
    view.setSize(WINDOW_WIDTH, -WINDOW_HEIGHT);
    window.setView(view);

    bool success = ImGui::SFML::Init(window);
    if (success)
    {
        ImGui::GetStyle().ScaleAllSizes(3.f);
        ImGui::GetIO().FontGlobalScale = 2.f;
    }
    Renderer renderer;
    renderer.SetWindow(&window);

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

        // sf::Time dt = deltaClock.restart();
        // ImGui::SFML::Update(window, dt);
        ImGui::NewFrame();

        ImGui::Begin("ImGui Begin Title");
        // ImGui::Text("some text..");

        if (ImGui::Button("EXIT"))
            window.close();

        if (ImGui::Button("Reset"))
        {
            softBody = CreateSoftSquare();
            // CreateWheel(softBody,
            //             center,
            //             wheelRadius,
            //             diskMass,
            //             tireMass,
            //             tireRatio,
            //             diskHubCompliance,
            //             diskRimCompliance,
            //             tireBodyCompliance,
            //             tireTreadCompliance,
            //             tirePressureCompliance,
            //             tirePressure,
            //             radialSegments);
        }

        ImGui::SliderFloat("Simulation Speed", &simulationSpeed, 0.001f, 100.f);
        ImGui::SliderInt("Substeps", &solverSubsteps, 1, 10);
        ImGui::SliderInt("Iterations", &solverIterations, 1, 10);
        ImGui::SliderFloat("Gravity Y", &gravity.y, -100.f, 100.f);
        ImGui::SliderFloat("Gravity X", &gravity.x, -100.f, 100.f);

        // ImGui::SliderFloat("Wheel Radius", &wheelRadius, 50.f, 500.f);
        // ImGui::SliderFloat("Disk Mass", &diskMass, 1.f, 100.f);
        // ImGui::SliderFloat("Tire Mass", &tireMass, 1.f, 100.f);
        // ImGui::SliderFloat("Tire Ratio", &tireRatio, 0.1f, 0.9f);

        // ImGui::SliderFloat("Disk Hub Compliance", &diskHubCompliance, .0f, 1.f);
        // ImGui::SliderFloat("Disk Compliance", &diskRimCompliance, .0f, 1.f);
        // ImGui::SliderFloat("Tire Body Compliance", &tireBodyCompliance, .0f, 1.f);
        // ImGui::SliderFloat("Tire Tread Compliance", &tireTreadCompliance, .0f, 1.f);
        // ImGui::SliderFloat("Tire Pressure Compliance", &tirePressureCompliance, .0f, 1.f);
        // ImGui::SliderFloat("Tire Pressure", &tirePressure, 0.f, 10.f);

        // ImGui::SliderInt("Radial Segments", &radialSegments, 3, 32);

        ImGui::End();

        const float groundY = 500.0f;
        GenerateCollisionConstraints(softBody, groundY);
        Simulate(softBody, dt * simulationSpeed, solverSubsteps, solverIterations, gravity);
        int vi = 0;
        for (auto &c : softBody.distanceConstraints)
        {
            const auto &p1 = softBody.pointMasses.positions[c.i1];
            const auto &p2 = softBody.pointMasses.positions[c.i2];

            lines[vi].position = sf::Vector2f(p1.x, p1.y);
            lines[vi + 1].position = sf::Vector2f(p2.x, p2.y);
            lines[vi].color = lines[vi + 1].color = sf::Color::White;
            vi += 2;
        }

        // sf::Vector2f cameraCenter;
        // cameraCenter.x = softBody.pointMasses.position[0].x;
        // cameraCenter.y = softBody.pointMasses.position[0].y;
        // view.setCenter(cameraCenter);

        window.clear();
        window.draw(lines);
        sf::RectangleShape ground(sf::Vector2f(800, 5));
        ground.setPosition(0, groundY);
        ground.setFillColor(sf::Color::Green);
        window.draw(ground);

        // window.setView(view);
        // renderer.DrawDistanceConstraints(softBody.pointMasses, softBody.distanceConstraints);
        // carPositionX = softBody.pointMasses.position[0].x;
        // renderer.DrawLevel(level, carPositionX, fov, precision);
        // softBody.Simulate(dt.asSeconds() * simulationSpeed, gravity);
        // softBody.SolveGroundCollision(level);
        // renderer.DrawSoftBody(window, softBody);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}
