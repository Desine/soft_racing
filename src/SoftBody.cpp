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

        glm::vec2 deltaPosition = b.position - a.position;
        float currentDistance = glm::length(deltaPosition);
        if (currentDistance == 0.f)
            continue;
        
        glm::vec2 correction = (currentDistance - d.restDistance) * (deltaPosition / currentDistance);
        a.position += wA / wSum * correction;
        b.position -= wB / wSum * correction;

        // // Вычисляем correction, как разницу между текущим расстоянием и заданным
        // glm::vec2 correction = (currentDistance - d.restDistance) * (deltaPosition / currentDistance);

        // // Вычисляем исправление с учётом compliance
        // float C = currentDistance - d.restDistance;
        // float alpha = d.compliance / deltaTime / deltaTime;
        // float s = -C / (wSum + alpha); // alpha добавляется для учёта compliance

        // // Применяем изменения к позициям точек массы
        // a.position += wA / wSum * s * correction;
        // b.position -= wB / wSum * s * correction;

        /*
        glm::vec2 gradient = glm::normalize(a.position - a.position);
        float alpha = d.compliance / deltaTime / deltaTime;
        float constraint = currentDistance - d.restDistance; // C
        float scaler = -constraint / (wSum + alpha);

        a.position -= scaler * wA * gradient;
        b.position += scaler * wB * gradient;

        // delta x1 =  (w1 / w1 + w2) * (l - l0) * ((x2 - x1) / |x2 - x1|)
        // delta x2 = -(w2 / w1 + w2) * (l - l0) * ((x2 - x1) / |x2 - x1|)
        // correction = (l - l0) * ((x2 - x1) / |x2 - x1|)
        // wSum = w1 + w2
        // delta x1 =  (w1 / wSum) * correction
        // delta x2 = -(w2 / wSum) * correction
        //
        // glm::vec2 correction = (currentDistance - d.restDistance) * (deltaPosition / currentDistance);
        // a.position += wA / wSum * correction;
        // b.position -= wB / wSum * correction;
        */
    }
}

void SoftBody::ResolveGroundCollision(const Level &level, float carPositionX, float fov, float precision)
{
}
