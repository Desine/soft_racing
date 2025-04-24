#include "imgui.h"
#include "imgui-SFML.h"
#include <SFML/Graphics.hpp>

#include <iostream>
#include <vector>
#include <random>

#include "tick_system.hpp"
#include "collision_system.hpp"
#include "simulation.hpp"
#include "soft_body.hpp"
#include "renderer.hpp"
#include "shape_tools.hpp"
#include "level.hpp"
#include "tick_system_imgui.hpp"
#include "bodies_manager.hpp"

const int WINDOW_WIDTH = 2000;
const int WINDOW_HEIGHT = 2000;

SoftBody CreateSoftPolygon(int segments)
{
    if (segments < 5)
        segments = 5;

    SoftBody softBody;

    glm::vec2 origin(0.0f, 300.0f);
    float radius = 50.0f;

    softBody.pointMasses.positions = CreatePoigonPositions(segments, radius, origin);
    softBody.pointMasses.prevPositions = softBody.pointMasses.positions;
    softBody.pointMasses.velocities.resize(segments, glm::vec2(0.0f));
    softBody.pointMasses.inverseMasses.resize(segments, 1.0f);
    softBody.pointMasses.velocities[0].x = 3.f; // one point

    AddDistanceConstraintsToLoop(softBody, 1e-5f);
    AddVolumeConstraintToLoop(softBody, 1e-5f);

    AddCollisionPointsToLoop(softBody);
    AddCollisionShapeToLoop(softBody);

    return softBody;
}

SoftBody CreateGround()
{
    SoftBody body;

    float spacing = 500.0f;
    glm::vec2 origin(0.0f, -300.0f);

    body.pointMasses.positions = {
        origin + glm::vec2(-spacing, -spacing / 2),
        origin + glm::vec2(-spacing, spacing / 4),

        origin + glm::vec2(-spacing * .7f, spacing),
        origin + glm::vec2(-spacing * .7f, spacing / 10),
        origin + glm::vec2(spacing * .7f, spacing / 10),
        origin + glm::vec2(spacing * .7f, spacing),

        origin + glm::vec2(spacing, spacing / 4),
        origin + glm::vec2(spacing, -spacing / 2)};
    int pointCount = body.pointMasses.positions.size();

    body.pointMasses.prevPositions = body.pointMasses.positions;
    body.pointMasses.velocities.resize(pointCount, glm::vec2(0.0f));
    body.pointMasses.inverseMasses.resize(pointCount, 0.0f);

    body.collisionPoints = {0, 1, 2, 3, 4, 5, 6, 7};
    body.collisionShape = {0, 1, 2, 3, 4, 5, 6, 7};

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
    sf::Vector2f cameraCenter;
    cameraCenter.x = 0;
    cameraCenter.y = 0;
    view.setCenter(cameraCenter);
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

    glm::vec2 gravity = glm::vec2(0.0f, -9.8f);
    float simulationSpeed = 10.f;
    int solverSubsteps = 5;
    int solverIterations = 3;

    std::random_device dev;
    std::mt19937 rng(dev());

    BodiesManager bodiesManager;
    bodiesManager.AddSoftBody(CreateSoftPolygon(4));
    bodiesManager.AddSoftBody(CreateGround());

    TickSystem tickSystem(30.0f);
    tickSystem.SetTimeScale(10.f);
    tickSystem.SetIsPause(true);
    tickSystem.StepOnce();

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
            bodiesManager.AddSoftBody(CreateSoftPolygon(rng() % 10));
            bodiesManager.AddSoftBody(CreateGround());
        }
        if (ImGui::Button("Add body"))
        {

            bodiesManager.AddSoftBody(CreateSoftPolygon(rng() % 10));
        }
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

            // Renderer::DrawSoftBodies(bodiesManager.GetSoftBodies());

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
            glm::vec2 geometryCenter = ComputeGeometryCenter(bodiesManager.GetSoftBody(0).pointMasses.positions);
            cameraCenter.x = geometryCenter.x;
            cameraCenter.y = geometryCenter.y;
            view.setCenter(cameraCenter);
            window.setView(view);
            std::cout << "Camera center x: " << cameraCenter.x << " y: " << cameraCenter.y << std::endl;
        }

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}
