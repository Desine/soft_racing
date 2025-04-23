#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include <iostream>
#include <vector>

#include "tick_system.hpp"
#include "collision_system.hpp"
#include "simulation.hpp"
#include "soft_body.hpp"
#include "renderer.hpp"
#include "car_tools.hpp"
#include "level.hpp"
#include "tick_system_imgui.hpp"
#include "bodies_manager.hpp"

const int WINDOW_WIDTH = 2000;
const int WINDOW_HEIGHT = 2000;

SoftBody CreateSoftSquare()
{
    SoftBody body;

    float spacing = 100.0f;
    glm::vec2 origin(1000.0f, 1000.0f);

    body.pointMasses.positions = {
        origin,
        origin + glm::vec2(0, spacing),
        origin + glm::vec2(spacing, spacing),
        origin + glm::vec2(spacing, 0)};

    body.pointMasses.prevPositions = body.pointMasses.positions;
    body.pointMasses.velocities.resize(4, glm::vec2(0.0f));
    body.pointMasses.inverseMasses = {1.0f, 1.0f, 1.0f, 1.0f};
    body.pointMasses.velocities[0].x = -30.f; // one point

    auto &d = body.distanceConstraints;
    d.push_back({0, 1, spacing, 1e-5f, 0});

    d.push_back({1, 2, spacing, 1e-5f, 0});
    d.push_back({2, 3, spacing, 1e-5f, 0});
    d.push_back({3, 0, spacing, 1e-5f, 0});
    // d.push_back({0, 2, spacing * std::sqrt(2.0f), 1e-5f, 0});
    // d.push_back({1, 3, spacing * std::sqrt(2.0f), 1e-5f, 0});

    VolumeConstraint vc;
    vc.indices = {0, 1, 2, 3};
    vc.restVolume = ComputePolygonArea(body.pointMasses.positions, vc.indices);
    vc.compliance = 1e-5f;
    vc.lambda = 0;
    body.volumeConstraints.push_back(vc);

    body.collisionPoints = {0, 1, 2, 3};
    body.collisionShape = {0, 1, 2, 3};

    return body;
}

void SetupWindow(sf::RenderWindow &window)
{
    window.create(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}), "Soft Racing");
    window.setFramerateLimit(60);
}

sf::View SetupView(sf::RenderWindow &window)
{
    sf::View view = window.getDefaultView();
    view.setSize(WINDOW_WIDTH, -WINDOW_HEIGHT);
    view.zoom(0.5f);
    window.setView(view);
    return view;
}

bool SetupImGui(sf::RenderWindow &window)
{
    if (!ImGui::SFML::Init(window))
        return false;

    ImGui::GetStyle().ScaleAllSizes(3.f);
    ImGui::GetIO().FontGlobalScale = 2.f;
    return true;
}

int main()
{
    sf::RenderWindow window;
    SetupWindow(window);
    sf::View view = SetupView(window);
    SetupImGui(window);

    bool cameraFollow = false;

    glm::vec2 gravity = glm::vec2(0.0f, -0.8f);
    float simulationSpeed = 10.f;
    int solverSubsteps = 1;
    int solverIterations = 1;

    float groundY = 900.0f;

    BodiesManager bodiesManager;
    bodiesManager.AddSoftBody(CreateSoftSquare());

    TickSystem tickSystem(30.0f);
    tickSystem.SetTimeScale(10.f);
    tickSystem.SetIsPause(true);

    Renderer::SetWindow(&window);

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

        TickSystemImGui(tickSystem);

        ImGui::Begin("Main");
        if (ImGui::Button("EXIT"))
            window.close();
        if (ImGui::Button("Reset"))
        {
            bodiesManager.Clear();
            bodiesManager.AddSoftBody(CreateSoftSquare());
        }
        if (ImGui::Button("Add body"))
            bodiesManager.AddSoftBody(CreateSoftSquare());
        if (ImGui::Button(cameraFollow ? "Camera !follow" : "Camera follow"))
            cameraFollow = !cameraFollow;
        ImGui::SliderInt("Substeps", &solverSubsteps, 1, 10);
        ImGui::SliderInt("Iterations", &solverIterations, 1, 10);
        ImGui::SliderFloat("Gravity Y", &gravity.y, -100.f, 100.f);
        ImGui::SliderFloat("Gravity X", &gravity.x, -100.f, 100.f);
        ImGui::End();

        // Simulate
        float dtReal = clock.restart().asSeconds();
        tickSystem.Update(dtReal);

        while (tickSystem.Step())
        {
            window.clear();
            Simulate(bodiesManager.GetSoftBodies(), tickSystem.GetFixedDt(), solverSubsteps, solverIterations, gravity);

            Renderer::DrawSoftBodies(bodiesManager.GetSoftBodies());

            // std::cout << "Points: " << bodiesManager.GetSoftBody(0).collisionPoints[0] << std::endl;
            // std::cout << "Points: " << bodiesManager.GetSoftBody(0).collisionPoints[1] << std::endl;
            // std::cout << "Points: " << bodiesManager.GetSoftBody(0).collisionPoints[2] << std::endl;
            // std::cout << "Points: " << bodiesManager.GetSoftBody(0).collisionPoints[3] << std::endl;
            // std::cout << "Shape: " << bodiesManager.GetSoftBody(0).collisionShape[0] << std::endl;
            // std::cout << "Shape: " << bodiesManager.GetSoftBody(0).collisionShape[1] << std::endl;
            // std::cout << "Shape: " << bodiesManager.GetSoftBody(0).collisionShape[2] << std::endl;
            // std::cout << "Shape: " << bodiesManager.GetSoftBody(0).collisionShape[3] << std::endl;
        }

        // Draw
        if (cameraFollow)
        {
            sf::Vector2f cameraCenter;
            glm::vec2 geometryCenter = GetGeometryCenter(bodiesManager.GetSoftBody(0).pointMasses);
            cameraCenter.x = geometryCenter.x;
            cameraCenter.y = geometryCenter.y;
            view.setCenter(cameraCenter);
            window.setView(view);
        }

        sf::RectangleShape ground(sf::Vector2f(800, 5));
        ground.setPosition(0, groundY);
        ground.setFillColor(sf::Color::Green);
        window.draw(ground);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}
