// file simulation.cpp
#include "simulation.hpp"
#include "integrator.hpp"
#include "constraints_solver.hpp"
#include "collision_system.hpp"
#include "renderer.hpp"
#include <iostream>

void Simulate(std::vector<SoftBody> &softBodies, float dt, int substeps, int iterations, const glm::vec2 &gravity)
{
    std::cout << "\n\nSimulate." << std::endl;

    float substepDt = dt / substeps;

    for (SoftBody &softBody : softBodies)
    {
        for (int step = 0; step < substeps; ++step)
        {
            Integrate(softBody.pointMasses, substepDt, gravity);

            for (auto &c : softBody.distanceConstraints)
                c.lambda = 0.0f;

            for (auto &c : softBody.volumeConstraints)
                c.lambda = 0.0f;

            for (auto &c : softBody.pinConstraints)
                c.lambda = 0.0f;

            for (int i = 0; i < iterations; ++i)
            {
                SolveDistanceConstraints(softBody.pointMasses, softBody.distanceConstraints, substepDt);
                SolveVolumeConstraints(softBody.pointMasses, softBody.volumeConstraints, substepDt);
                SolvePinConstraints(softBody.pointMasses, softBody.pinConstraints, substepDt);
            }

            UpdateVelocities(softBody.pointMasses, substepDt);
        }
    }

    // collision detection
    std::vector<SoftSoftCollisionConstraint> collisionConstraints;
    for (size_t i = 0; i < softBodies.size(); ++i)
    {
        for (size_t j = i + 1; j < softBodies.size(); ++j)
        {
            DetectSoftSoftCollisions(
                softBodies[i],
                softBodies[j],
                /*compliance=*/0.0001f,
                /*minDistanceThresholdf=*/100.01f,
                collisionConstraints);
        }
    }

    for (const auto &constraint : collisionConstraints)
        Renderer::DrawDebugCollision(*constraint.bodyA, *constraint.bodyB, constraint);
}
