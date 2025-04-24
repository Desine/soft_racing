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

    for (int step = 0; step < substeps; ++step)
    {
        for (SoftBody &softBody : softBodies)
        {
            Integrate(softBody.pointMasses, substepDt, gravity);

            ResetConstrainsLambdas(softBody);

            for (int i = 0; i < iterations; ++i)
            {
                SolveDistanceConstraints(softBody.pointMasses, softBody.distanceConstraints, substepDt);
                SolveVolumeConstraints(softBody.pointMasses, softBody.volumeConstraints, substepDt);
                SolvePinConstraints(softBody.pointMasses, softBody.pinConstraints, substepDt);
            }

            Renderer::DrawSoftBody(softBody);

            auto center = ComputeGeometryCenter(softBody.pointMasses.positions);
            std::cout << "\nsoftBody center before collision x: " << center.x << " y: " << center.y << std::endl;
            for (int i = 0; i < softBody.pointMasses.positions.size(); i++)
                std::cout << "point: " << i << " x: " << softBody.pointMasses.positions[i].x << " y: " << softBody.pointMasses.positions[i].y << std::endl;
        }

        // detection collisions
        std::vector<SoftSoftCollisionConstraint> collisionConstraints;
        for (size_t i = 0; i < softBodies.size(); ++i)
        {
            for (size_t j = i + 1; j < softBodies.size(); ++j)
            {
                DetectSoftSoftCollisions(
                    softBodies[i],
                    softBodies[j],
                    /*compliance=*/0.001f,
                    collisionConstraints);
                DetectSoftSoftCollisions(
                    softBodies[j],
                    softBodies[i],
                    /*compliance=*/0.001f,
                    collisionConstraints);
            }
        }

        // solve collisions
        for (auto &constraint : collisionConstraints)
        {
            Renderer::DrawDebugCollision(*constraint.bodyA, *constraint.bodyB, constraint);

            for (int i = 0; i < iterations; ++i)
                SolveSoftSoftCollisionConstraint(constraint, substepDt);
        }
    }

    // update velocity
    for (SoftBody &softBody : softBodies)
    {
        Renderer::DrawSoftBody(softBody);
        auto center = ComputeGeometryCenter(softBody.pointMasses.positions);
        std::cout << "\nsoftBody center after collision x: " << center.x << " y: " << center.y << std::endl;
        for (int i = 0; i < softBody.pointMasses.positions.size(); i++)
            std::cout << "point: " << i << " x: " << softBody.pointMasses.positions[i].x << " y: " << softBody.pointMasses.positions[i].y << std::endl;

        UpdateVelocities(softBody.pointMasses, substepDt);
    }
}
