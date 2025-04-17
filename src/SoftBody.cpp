#include "SoftBody.hpp"
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>

void SoftBody::AddPoint(const glm::vec2 &position, float mass, const glm::vec2 &velocity, bool fixed)
{
    pointMasses.push_back({position, position, mass, velocity, fixed});
}

void SoftBody::AddDistanceConstraint(int a, int b, float compliance)
{
    float restLength = glm::length(pointMasses[a].position - pointMasses[b].position);
    distanceConstraints.push_back({a, b, restLength, compliance});
}

void SoftBody::Simulate(float deltaTime, glm::vec2 gravity = glm::vec2(0.0f, -980.0f))
{
    int substeps = 1;
    float deltaStep = deltaTime / substeps;
    for (int i = 0; i < substeps; i++)
    {
        // integration
        for (auto &p : pointMasses)
        {
            if (p.isFixed)
                continue;

            p.velocity += deltaStep * gravity;
            p.previousPosition = p.position;
            p.position += deltaStep * p.velocity;
        }

        // solve
        SolveDistanceConstraints(deltaStep);

        // velocity update
        for (auto &p : pointMasses)
        {
            p.velocity = (p.position - p.previousPosition) / deltaStep;
        }
    }
}

void SoftBody::SolveDistanceConstraints(float deltaTime)
{
    for (auto &d : distanceConstraints)
    {
        PointMass &a = pointMasses[d.a];
        PointMass &b = pointMasses[d.b];

        // w = inverse mass
        float wA = a.isFixed ? 0.0f : 1.0f / a.mass;
        float wB = b.isFixed ? 0.0f : 1.0f / b.mass;
        float wSum = wA + wB;
        if (wSum == 0.0f)
            continue;

        glm::vec2 gradient = a.position - b.position;
        float length = glm::length(gradient); // current distance
        gradient /= length; // normalize gradient
        
        float C = length - d.restDistance; // constraint value
        float alpha = d.compliance / deltaTime / deltaTime;
        float deltaLagrangianMultiplier = -C / (wSum + alpha);

        a.position += wA * deltaLagrangianMultiplier * gradient;
        b.position -= wB * deltaLagrangianMultiplier * gradient;
    }
}

void SoftBody::ResolveGroundCollision(const Level &level, float carPositionX, float fov, float precision)
{
}
