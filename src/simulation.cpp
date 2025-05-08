#include "simulation.hpp"
#include "joint_system.hpp"
#include "constraints_solver.hpp"
#include "collision_system.hpp"
#include "integrator.hpp"
#include "renderer.hpp"

#include <iostream>

void Simulate(PhysicsScene &physicsScene, float dt, int substeps, int iterations)
{
    std::vector<std::shared_ptr<SoftBody>> &softBodies = physicsScene.softBodies;
    float substep_dt = dt / substeps;

    for (int step = 0; step < substeps; ++step)
    {
        for (auto &sbPtr : softBodies)
        {
            Integrate(sbPtr->pointMasses, substep_dt, physicsScene.gravity);

            ResetConstrainsLambdas(*sbPtr);

            for (int i = 0; i < iterations; ++i)
            {
                SolveAccelerationConstraints(sbPtr->pointMasses, sbPtr->accelerationConstraints, substep_dt / iterations);
                SolveForceConstraints(sbPtr->pointMasses, sbPtr->forceConstraints, substep_dt / iterations);
                SolveVelocityConstraints(sbPtr->pointMasses, sbPtr->VelocityConstraints, substep_dt / iterations);
                SolveAngularAccelerationConstraints(sbPtr->pointMasses, sbPtr->angularAccelerationConstraints, substep_dt / iterations);
                SolveAngularForceConstraints(sbPtr->pointMasses, sbPtr->angularForceConstraints, substep_dt / iterations);
                SolveAngularVelocityConstraints(sbPtr->pointMasses, sbPtr->angularVelocityConstraints, substep_dt / iterations);

                SolveDistanceConstraints(sbPtr->pointMasses, sbPtr->distanceConstraints, substep_dt);
                SolveVolumeConstraints(sbPtr->pointMasses, sbPtr->volumeConstraints, substep_dt);
                SolveAngleConstraints(sbPtr->pointMasses, sbPtr->angleConstraints, substep_dt);
                SolvePinConstraints(sbPtr->pointMasses, sbPtr->pinConstraints, substep_dt);
                SolveShapeMatchingConstraints(sbPtr->pointMasses, sbPtr->shapeMatchingConstraints, substep_dt);
            }

            // Renderer::DrawSoftBody(softBody);
        }

        ResetJointsLambdas(physicsScene);
        for (int i = 0; i < iterations; ++i)
        {
            SolveDistanceJoints(physicsScene.distanceJoints, substep_dt);
            SolveMotorJoints(physicsScene.motorJoints, substep_dt);
        }

        // detection collisions
        std::vector<SoftSoftCollisionConstraint> collisionConstraints;
        for (size_t i = 0; i < softBodies.size(); ++i)
        {
            for (size_t j = i + 1; j < softBodies.size(); ++j)
            {
                DetectSoftSoftCollisions(
                    *softBodies[i],
                    *softBodies[j],
                    /*compliance=*/0.0f,
                    /*frictionStatic*/ 1.0f,
                    /*frictionKinetic*/ 0.3f,
                    collisionConstraints);
                DetectSoftSoftCollisions(
                    *softBodies[j],
                    *softBodies[i],
                    /*compliance=*/0.0f,
                    /*frictionStatic*/ 1.0f,
                    /*frictionKinetic*/ 0.3f,
                    collisionConstraints);
            }
        }

        // solve collisions
        for (auto &cc : collisionConstraints)
        {
            // Renderer::DrawSoftSoftPointEdgeCollision(cc);

            for (int i = 0; i < iterations; ++i)
                SolveSoftSoftCollisionConstraint(cc, substep_dt);
        }

        // update velocity
        for (auto &sbPtr : softBodies)
        {
            UpdateVelocities(sbPtr->pointMasses, substep_dt);
        }
    }
    Renderer::DrawDistanceJoints(physicsScene.distanceJoints);
}
