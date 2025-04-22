// file Simulation.cpp
#include "simulation.hpp"
#include "soft_body.hpp"
#include "constraints_solver.hpp"
#include "integrator.hpp"
#include "collision_solver.hpp"

void Simulate(SoftBody &softBody, float dt, int substeps, int iterations, const glm::vec2 &gravity)
{

    float substepDt = dt / substeps;

    for (int step = 0; step < substeps; ++step)
    {

        Integrate(softBody.pointMasses, substepDt, gravity);

        for (auto &c : softBody.distanceConstraints)
            c.lambda = 0.0f;

        for (auto &c : softBody.volumeConstraints)
            c.lambda = 0.0f;

        for (auto &c : softBody.pinConstraints)
            c.lambda = 0.0f;

        for (auto &c : softBody.collisionConstraints)
            c.lambda = 0.0f;

        for (int i = 0; i < iterations; ++i)
        {
            SolveDistanceConstraints(softBody.pointMasses, softBody.distanceConstraints, substepDt);
            SolveVolumeConstraints(softBody.pointMasses, softBody.volumeConstraints, substepDt);
            SolvePinConstraints(softBody.pointMasses, softBody.pinConstraints, substepDt);
            SolveCollisionConstraints(softBody.pointMasses, softBody.collisionConstraints, substepDt);
        }

        UpdateVelocities(softBody.pointMasses, substepDt);
    }
}
