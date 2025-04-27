// file simulation.cpp
#include "simulation.hpp"
#include "integrator.hpp"
#include "constraints_solver.hpp"
#include "collision_system.hpp"
#include "renderer.hpp"
#include <iostream>

void Simulate(std::vector<SoftBody> &softBodies, float dt, int substeps, int iterations, const glm::vec2 &gravity)
{
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
                SolveAngleConstraints(softBody.pointMasses, softBody.angleConstraints, substepDt);
                SolvePinConstraints(softBody.pointMasses, softBody.pinConstraints, substepDt);
                SolveShapeMatchingConstraints(softBody.pointMasses, softBody.shapeMatchingConstraints, substepDt);
            }

            Renderer::DrawSoftBody(softBody);
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
                    /*compliance=*/0.0001f,
                    /*frictionStatic*/ 1.0f,
                    /*frictionKinetic*/ 0.3f,

                    collisionConstraints);
                DetectSoftSoftCollisions(
                    softBodies[j],
                    softBodies[i],
                    /*compliance=*/0.0001f,
                    /*frictionStatic*/ 1.0f,
                    /*frictionKinetic*/ 0.3f,

                    collisionConstraints);
            }
        }

        // solve collisions
        for (auto &constraint : collisionConstraints)
        {
            // Renderer::DrawSoftSoftPointEdgeCollision(*constraint.bodyA, *constraint.bodyB, constraint);

            for (int i = 0; i < iterations; ++i)
                SolveSoftSoftCollisionConstraint(constraint, substepDt);
        }
    }

    // update velocity
    for (SoftBody &softBody : softBodies)
    {
        UpdateVelocities(softBody.pointMasses, substepDt);

        Renderer::DrawSoftBody(softBody);
    }
}
