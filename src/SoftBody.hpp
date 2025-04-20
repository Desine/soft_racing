#pragma once
#include "Level.hpp"
#include <vector>
#include <SFML/Graphics.hpp>
#include "glm/glm.hpp"

class PointMass
{
public:
    std::vector<glm::vec2> position, prevPosition, velocity, force;
    std::vector<float> mass, inverseMass;

    int count = 0;

    int Add(const glm::vec2 &position, float mass, const glm::vec2 &velocity = glm::vec2(0, 0));
    void ApplyForce(int index, const glm::vec2 &force);
    void Integrate(float deltaTime);
    void UpdateVelocity(float deltaTime);
};

class Constraint
{
public:
    std::vector<float> compliance, lambda;
    int count = 0;

    virtual void Project(PointMass &pointMasses, float deltaTime, bool firstIteration);
};

class DistanceConstraint : public Constraint
{
public:
    std::vector<int> i1, i2;
    std::vector<float> restDistance;

    int Add(int index1, int index2, float restLength, float compliance);
    void Project(PointMass &pointMasses, float deltaTime, bool firstIteration) override;
};

class VolumeConstraint : public Constraint
{
public:
    std::vector<std::vector<int>> indices;
    std::vector<float> restVolume;

    int Add(std::vector<int> indices, float restVolume, float compliance);
    void Project(PointMass &pointMasses, float deltaTime, bool firstIteration) override;
};

class BendingConstraint : public Constraint
{
public:
    std::vector<int> i1, i2, i3;
    std::vector<float> restAngle;

    int Add(int index1, int index2, int index3, float restAngle, float compliance);
    void Project(PointMass &pointMasses, float deltaTime, bool firstIteration) override;
};

class SoftBody
{
public:
    PointMass pointMasses;
    DistanceConstraint distanceConstraints;
    VolumeConstraint volumeConstraints;
    BendingConstraint bendingConstraints;

    void Simulate(float deltaTime, glm::vec2 gravity = glm::vec2(0.0f, -9.8f), const Level &level = NULL);
    void SolveGroundCollision(const Level &level);
    void Clear();
};
