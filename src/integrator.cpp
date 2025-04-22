//file Integrator.cpp
#include "integrator.hpp"
#include "soft_body.hpp"


void Integrate(PointMasses &pm, float dt, const glm::vec2 &gravity)
{
    for (size_t i = 0; i < pm.positions.size(); ++i)
    {
        if (pm.inverseMasses[i] == 0.0f) continue;

        pm.velocities[i] += gravity * dt;
        pm.prevPositions[i] = pm.positions[i];
        pm.positions[i] += pm.velocities[i] * dt;
    }
}

void UpdateVelocities(PointMasses &pm, float dt)
{
    for (size_t i = 0; i < pm.positions.size(); ++i)
    {
        pm.velocities[i] = (pm.positions[i] - pm.prevPositions[i]) / dt;
    }
}
