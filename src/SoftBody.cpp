#include "SoftBody.hpp"
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vector_angle.hpp>

void SoftBody::AddPoint(const glm::vec2 &position, float mass, const glm::vec2 &velocity, bool fixed)
{
    pointMasses.push_back({position, position, velocity, mass, 0, fixed});
}

void SoftBody::AddDistanceConstraint(int a, int b, float compliance)
{
    float restLength = glm::length(pointMasses[a].position - pointMasses[b].position);
    distanceConstraints.push_back({a, b, restLength, compliance});
}

void SoftBody::AddVolumeConstraint(const std::vector<int> &pointIndices, float compliance)
{
    if (pointIndices.size() < 3)
        return;

    float restVolume = 0.0f;
    for (size_t i = 0; i < pointIndices.size(); ++i)
    {
        const glm::vec2 &a = pointMasses[pointIndices[i]].position;
        const glm::vec2 &b = pointMasses[pointIndices[(i + 1) % pointIndices.size()]].position;
        restVolume += a.x * b.y - a.y * b.x;
    }
    restVolume = 0.5f * std::abs(restVolume);

    volumeConstraints.push_back({pointIndices, restVolume, compliance});
}
void SoftBody::AddVolumeConstraint(const std::vector<int> &pointIndices, float compliance, float volume)
{
    AddVolumeConstraint(pointIndices, compliance);
    volumeConstraints.back().restVolume = volume;
}

void SoftBody::Simulate(float deltaTime, glm::vec2 gravity)
{
    int substeps = 1;
    float deltaStep = deltaTime / substeps;
    for (int i = 0; i < substeps; i++)
    {
        // integration
        for (auto &p : pointMasses)
        {
            p.inverseMass = p.isFixed ? 0.0f : 1.0f / p.mass; // reduce calculation

            p.velocity += deltaStep * gravity;
            p.previousPosition = p.position;
            p.position += deltaStep * p.velocity;
        }

        // solve
        SolveDistanceConstraints(deltaStep);
        SolveVolumeConstraints(deltaStep);

        // velocity update
        for (auto &p : pointMasses)
        {
            if (p.isFixed)
                continue;
            p.velocity = (p.position - p.previousPosition) / deltaStep;
        }
    }
}

void SoftBody::SolveDistanceConstraints(float deltaTime)
{
    for (auto &d : distanceConstraints)
    {
        PointMass &pA = pointMasses[d.a];
        PointMass &pB = pointMasses[d.b];

        float wSum = pA.inverseMass + pB.inverseMass;
        if (wSum == 0.0f)
            continue;

        glm::vec2 gradient = pA.position - pB.position;
        float length = glm::length(gradient); // current distance
        if (length == 0.0f)
            continue;
        gradient /= length; // normalize gradient

        float c = length - d.restDistance; // constraint value
        float alpha = d.compliance / deltaTime / deltaTime;
        float s = -c / (wSum + alpha); // scalar, delta Lagrangian multiplier

        pA.position += pA.inverseMass * s * gradient;
        pB.position -= pB.inverseMass * s * gradient;
    }
}

void SoftBody::SolveVolumeConstraints(float deltaTime)
{
    for (auto &v : volumeConstraints)
    {
        int pointsCount = v.ids.size();

        float currentVolume = 0.0f;
        glm::vec2 gradients[pointsCount];
        float wSum = 0.0f;
        for (size_t i = 0; i < pointsCount; ++i)
        {
            const glm::vec2 &prev = pointMasses[v.ids[(i - 1 + pointsCount) % pointsCount]].position;
            const glm::vec2 &curr = pointMasses[v.ids[i]].position;
            const glm::vec2 &next = pointMasses[v.ids[(i + 1) % pointsCount]].position;
            currentVolume +=  0.5f * (curr.x * next.y - curr.y * next.x);

            glm::vec2 grad = 0.5f * glm::vec2(next.y - prev.y, prev.x - next.x);
            gradients[i] = grad;

            wSum += pointMasses[v.ids[i]].inverseMass * glm::dot(grad, grad);
        }
        if (wSum == 0)
            continue;

        float c = currentVolume - v.restVolume; // constraint value
        float alpha = v.compliance / deltaTime / deltaTime;
        float s = -c / (wSum + alpha); // scalar, delta Lagrangian multiplier

        for (int i = 0; i < pointsCount; ++i)
        {
            PointMass &p = pointMasses[v.ids[i]];
            p.position += p.inverseMass * s * gradients[i];
        }
    }
}

void SoftBody::ResolveGroundCollision(const Level &level, float carPositionX, float fov, float precision)
{
}
