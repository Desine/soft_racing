#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include <iostream>

#include "tick_system.hpp"
#include "collision_solver.hpp"
#include "simulation.hpp"
#include "soft_body.hpp"
#include "renderer.hpp"
#include "car_tools.hpp"
#include "level.hpp"
#include "imgui_tick_system.hpp"

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
    int solverSubsteps = 1;
    int solverIterations = 2;

    SoftBody softBody = CreateSoftSquare();
    sf::VertexArray lines(sf::Lines, 12);

    sf::RenderWindow window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}), "Soft Racing");
    window.setFramerateLimit(60);
    sf::View view = window.getDefaultView();
    view.setSize(WINDOW_WIDTH, -WINDOW_HEIGHT);
    window.setView(view);
    bool cameraFollow = false;

    bool success = ImGui::SFML::Init(window);
    if (success)
    {
        ImGui::GetStyle().ScaleAllSizes(3.f);
        ImGui::GetIO().FontGlobalScale = 2.f;
    }
    Renderer renderer;
    renderer.SetWindow(&window);

    TickSystem tickSystem(60.0f);
    tickSystem.SetTimeScale(10.f);

    sf::Clock clock;
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);

            if (event.type == sf::Event::Closed)
                window.close();
        }

        // GUI
        ImGui::NewFrame();

        ImGuiTickSystem(tickSystem);

        ImGui::Begin("Main");

        if (ImGui::Button("EXIT"))
            window.close();

        if (ImGui::Button("Reset"))
            softBody = CreateSoftSquare();

        if (ImGui::Button("Camera follow"))
            cameraFollow = !cameraFollow;

        ImGui::SliderInt("Substeps", &solverSubsteps, 1, 10);
        ImGui::SliderInt("Iterations", &solverIterations, 1, 10);
        ImGui::SliderFloat("Gravity Y", &gravity.y, -100.f, 100.f);
        ImGui::SliderFloat("Gravity X", &gravity.x, -100.f, 100.f);

        ImGui::End();

        // Simulate
        const float groundY = 500.0f;
        float dtReal = clock.restart().asSeconds();
        tickSystem.Update(dtReal);

        while (tickSystem.ShouldStep())
        {
            tickSystem.Step();
            float dtSim = tickSystem.GetFixedDt();

            GenerateCollisionConstraints(softBody, groundY);
            Simulate(softBody, dtSim, solverSubsteps, solverIterations, gravity);
        }

        // Draw
        window.clear();
        if (cameraFollow)
        {
            sf::Vector2f cameraCenter;
            glm::vec2 geometryCenter = GetGeometryCenter(softBody.pointMasses);
            cameraCenter.x = geometryCenter.x;
            cameraCenter.y = geometryCenter.y;
            view.setCenter(cameraCenter);
            window.setView(view);
        }

        renderer.DrawDistanceConstraints(softBody.pointMasses, softBody.distanceConstraints);

        sf::RectangleShape ground(sf::Vector2f(800, 5));
        ground.setPosition(0, groundY);
        ground.setFillColor(sf::Color::Green);
        window.draw(ground);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}
