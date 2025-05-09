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
#include "physics_scene.hpp"
#include "soft_body_loader.hpp"
#include "joint_system.hpp"

const int WINDOW_WIDTH = 2000;
const int WINDOW_HEIGHT = 2000;

SoftBody CreateSoftPolygon(int segments)
{
    if (segments < 5)
        segments = 5;

    SoftBody softBody;

    glm::vec2 origin(0.0f, 300.0f);
    float radius = 50.0f;
    float mass = 10;

    softBody.pointMasses.positions = CreatePoigonPositions(segments, radius, origin);
    softBody.pointMasses.prevPositions = softBody.pointMasses.positions;
    softBody.pointMasses.velocities.resize(segments, glm::vec2(0.0f));
    softBody.pointMasses.inverseMasses.resize(segments, 1.0f / (mass / segments));

    AddDistanceConstraintsToLoop(softBody, 1e-4f);
    AddVolumeConstraintToLoop(softBody, 1e-3f);

    AddCollisionPointsToLoop(softBody);
    AddCollisionShapeToLoop(softBody);

    // glm::vec2 centerPosition = ComputeGeometryCenter(softBody.pointMasses.positions);
    glm::vec2 centerMass = ComputeMassCenter(softBody.pointMasses.positions, softBody.pointMasses.inverseMasses);
    ShapeMatchingConstraint shapeMatchingConstraint;
    shapeMatchingConstraint.startCenterMass = centerMass;
    shapeMatchingConstraint.compliance = .8f;
    shapeMatchingConstraint.lambda = 0.0f;
    for (int i = 0; i < segments; ++i)
    {
        shapeMatchingConstraint.startPositions.push_back(softBody.pointMasses.positions[i]);
        shapeMatchingConstraint.indices.push_back(i);
    }
    // softBody.shapeMatchingConstraints.push_back(shapeMatchingConstraint);

    // PinConstraint pinConstraint;
    // pinConstraint.index = 0;
    // pinConstraint.targetPosition = glm::vec2(0.0f, 350.0f);
    // softBody.pinConstraints.push_back(pinConstraint);

    // testing
    // softBody.pointMasses.velocities[0].x = 3.f; // one point
    // softBody.pointMasses.positions[0] += glm::vec2(300.0f, 0.0f);

    return softBody;
}

SoftBody CreateGround()
{
    SoftBody softBody;

    float spacing = 500.0f;
    glm::vec2 origin(0.0f, -450.0f);

    softBody.pointMasses.positions = {
        origin + glm::vec2(-spacing, -spacing / 2),
        origin + glm::vec2(-spacing, spacing / 4),

        origin + glm::vec2(-spacing * .7f, spacing),
        origin + glm::vec2(-spacing * .7f, spacing / 10),
        origin + glm::vec2(spacing * .7f, spacing / 10),
        origin + glm::vec2(spacing * .7f, spacing),

        origin + glm::vec2(spacing, spacing / 4),
        origin + glm::vec2(spacing, -spacing / 2)};
    int pointCount = softBody.pointMasses.positions.size();

    softBody.pointMasses.prevPositions = softBody.pointMasses.positions;
    softBody.pointMasses.velocities.resize(pointCount, glm::vec2(0.0f));
    softBody.pointMasses.inverseMasses.resize(pointCount, 0.0f);

    AddCollisionPointsToLoop(softBody);
    AddCollisionShapeToLoop(softBody);

    return softBody;
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

    float simulationSpeed = 10.f;
    int solverSubsteps = 20;
    int solverIterations = 1;

    std::random_device dev;
    std::mt19937 rng(dev());

    PhysicsScene physicsScene;
    physicsScene.gravity = glm::vec2(0.0f, -9.8f);
    physicsScene.softBodies.push_back(std::make_shared<SoftBody>(CreateSoftPolygon(rng() % 20)));
    physicsScene.softBodies.push_back(std::make_shared<SoftBody>(CreateGround()));

    TickSystem tickSystem(30.0f);
    tickSystem.SetTimeScale(10.f);
    tickSystem.SetIsPause(true);
    tickSystem.StepOnce();

    Renderer::SetWindow(&window);

    sf::Clock clock;

    Car car;
    AccelerationConstraint *carAccelerationConstraint = nullptr;
    AngularAccelerationConstraint *carAngularAccelerationConstraint = nullptr;
    AngularAccelerationConstraint *wheelAngularAccelerationConstraint = nullptr;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            ImGui::SFML::ProcessEvent(event);

            if (event.type == sf::Event::Closed)
                window.close();
        }

        // control
        if (carAccelerationConstraint)
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::W))
                carAccelerationConstraint->acceleration.y = 20;
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::S))
                carAccelerationConstraint->acceleration.y = -10;
            else
                carAccelerationConstraint->acceleration.y = 0;

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A))
                carAccelerationConstraint->acceleration.x = -10;
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D))
                carAccelerationConstraint->acceleration.x = 10;
            else
                carAccelerationConstraint->acceleration.x = 0;
        }
        if (carAngularAccelerationConstraint)
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
                carAngularAccelerationConstraint->acceleration = 0.3f;
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
                carAngularAccelerationConstraint->acceleration = -0.3f;
            else
                carAngularAccelerationConstraint->acceleration = 0;
        }
        if (wheelAngularAccelerationConstraint)
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))
                wheelAngularAccelerationConstraint->acceleration = 20;
            else if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down))
                wheelAngularAccelerationConstraint->acceleration = -20;
            else
                wheelAngularAccelerationConstraint->acceleration = 0;
        }

        // GUI
        ImGui::NewFrame();

        TickSystemImGui(tickSystem);

        ImGui::Begin("Main");
        if (ImGui::Button("EXIT"))
            window.close();
        if (ImGui::Button("Reset"))
        {
            physicsScene.Clear();
            physicsScene.softBodies.push_back(std::make_shared<SoftBody>(CreateSoftPolygon(rng() % 20)));
            physicsScene.softBodies.push_back(std::make_shared<SoftBody>(CreateGround()));
        }
        if (ImGui::Button("Add car.json"))
        {
            car = LoadCarFromFile("car.json");

            AccelerationConstraint carAC;
            carAC.indices = car.body.get()->collisionPoints;
            car.body.get()->accelerationConstraints.push_back(carAC);
            carAccelerationConstraint = &car.body.get()->accelerationConstraints[0];

            AngularAccelerationConstraint carAAC;
            carAAC.indices = car.body->collisionPoints;
            car.body->angularAccelerationConstraints.push_back(carAAC);
            carAngularAccelerationConstraint = &car.body.get()->angularAccelerationConstraints[0];

            AngularAccelerationConstraint wheelAAC;
            wheelAAC.indices = car.wheels[0]->collisionPoints;
            car.wheels[0]->angularAccelerationConstraints.push_back(wheelAAC);
            wheelAngularAccelerationConstraint = &car.wheels[0].get()->angularAccelerationConstraints[0];

            physicsScene.softBodies.push_back(car.body);
            for (auto &wheel : car.wheels)
                physicsScene.softBodies.push_back(wheel);
            for (auto &joint : car.distanceJoints)
                physicsScene.distanceJoints.push_back(joint);
            for (auto &joint : car.motorJoints)
                physicsScene.motorJoints.push_back(joint);
        }

        if (ImGui::Button("Add wheel"))
        {
            glm::vec2 center = glm::vec2(0, 0);
            float wheelRadius = 100;
            float diskMass = 20;
            float tireMass = 5;
            float tireRatio = .4f;
            float diskHubCompliance = .001f;
            float diskRimCompliance = .001f;
            float tireBodyCompliance = .001f;
            float tireTreadCompliance = .001f;
            float tirePressureCompliance = .001f;
            float tirePressure = 1.f;
            int radialSegments = rng() % 20;

            physicsScene.softBodies.push_back(std::make_shared<SoftBody>(CreateWheel(
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
                radialSegments)));
        }
        if (ImGui::Button("Add body"))
        {
            physicsScene.softBodies.push_back(std::make_shared<SoftBody>(CreateSoftPolygon(rng() % 20)));
            physicsScene.softBodies.push_back(std::make_shared<SoftBody>(CreateSoftPolygon(rng() % 20)));

            auto softBody1 = physicsScene.softBodies[physicsScene.softBodies.size() - 2];
            auto softBody2 = physicsScene.softBodies[physicsScene.softBodies.size() - 1];

            for (auto &p : softBody2->pointMasses.positions)
                p += glm::vec2(100.0f, 0.0f);

            physicsScene.distanceJoints.push_back(std::make_shared<DistanceJoint>());
            auto distanceJoint = physicsScene.distanceJoints[physicsScene.distanceJoints.size() - 1];
            distanceJoint->softBody1 = softBody1;
            distanceJoint->softBody2 = softBody2;
            distanceJoint->index1 = 0;
            distanceJoint->index2 = 0;
            distanceJoint->restDistance = 200.0f;

            physicsScene.motorJoints.push_back(std::make_shared<MotorJoint>());
            auto motorJoint = physicsScene.motorJoints[physicsScene.motorJoints.size() - 1];
            motorJoint->softBody1 = softBody1;
            motorJoint->softBody2 = softBody2;
            motorJoint->anchorSoftBody = softBody1;
            motorJoint->indices1 = softBody1->collisionPoints;
            motorJoint->indices2 = softBody2->collisionPoints;
            motorJoint->anchorIndices = softBody1->collisionPoints;
            motorJoint->anchorStartPositions = softBody1->pointMasses.positions;
            motorJoint->targetRPM = 1.0f;
            motorJoint->torque = 10.0f;
            motorJoint->compliance = 0.2f;
        }
        if (ImGui::Button("Add car_body.json"))
            physicsScene.softBodies.push_back(std::make_shared<SoftBody>(LoadSoftBodyFromFile("car_body.json")));
        if (ImGui::Button(cameraFollow ? "Camera !follow" : "Camera follow"))
            cameraFollow = !cameraFollow;
        ImGui::SliderInt("Substeps", &solverSubsteps, 1, 40);
        ImGui::SliderInt("Iterations", &solverIterations, 1, 10);
        ImGui::SliderFloat("Gravity X", &physicsScene.gravity.x, -20.f, 20.f);
        ImGui::SliderFloat("Gravity Y", &physicsScene.gravity.y, -20.f, 20.f);
        if (ImGui::Button("Gravity zedo"))
            physicsScene.gravity = {0.0f, 0.0f};

        if (carAngularAccelerationConstraint)
            carAngularAccelerationConstraint->position = ComputeMassCenter(car.body->pointMasses.positions, car.body->pointMasses.inverseMasses);
        if (wheelAngularAccelerationConstraint)
            wheelAngularAccelerationConstraint->position = ComputeGeometryCenter(car.wheels[0]->pointMasses.positions);
        ImGui::End();

        // Simulate
        float dtReal = clock.restart().asSeconds();
        tickSystem.Update(dtReal);

        while (tickSystem.Step())
        {
            window.clear();
            Simulate(physicsScene, tickSystem.GetFixedDt(), solverSubsteps, solverIterations);
        }

        // Draw
        if (cameraFollow)
        {
            sf::Vector2f cameraCenter;
            glm::vec2 geometryCenter = ComputeGeometryCenter(physicsScene.softBodies[0]->pointMasses.positions);
            cameraCenter.x = geometryCenter.x;
            cameraCenter.y = geometryCenter.y;
            view.setCenter(cameraCenter);
            window.setView(view);
            std::cout << "Camera center x: " << cameraCenter.x << " y: " << cameraCenter.y << std::endl;
        }

        for (auto &sb : physicsScene.softBodies)
            Renderer::DrawSoftBody(*sb);

        ImGui::SFML::Render(window);
        window.display();
    }

    ImGui::SFML::Shutdown();
}
