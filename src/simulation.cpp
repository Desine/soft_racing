#include "simulation.hpp"
#include "integrator.hpp"
#include "constraints_solver.hpp"
#include "collision_system.hpp"
#include "renderer.hpp"
#include "joint_system.hpp"

#include <iostream>

void Simulate(PhysicsScene &physicsScene, float dt, int substeps, int iterations)
{
    std::vector<SoftBody> &softBodies = physicsScene.softBodies;
    float substep_dt = dt / substeps;

    for (int step = 0; step < substeps; ++step)
    {
        for (SoftBody &sb : softBodies)
        {
            Integrate(sb.pointMasses, substep_dt, physicsScene.gravity);

            ResetConstrainsLambdas(sb);

            for (int i = 0; i < iterations; ++i)
            {
                SolveDistanceConstraints(sb.pointMasses, sb.distanceConstraints, substep_dt);
                SolveVolumeConstraints(sb.pointMasses, sb.volumeConstraints, substep_dt);
                SolveAngleConstraints(sb.pointMasses, sb.angleConstraints, substep_dt);
                SolvePinConstraints(sb.pointMasses, sb.pinConstraints, substep_dt);
                SolveShapeMatchingConstraints(sb.pointMasses, sb.shapeMatchingConstraints, substep_dt);
            }

            // Renderer::DrawSoftBody(softBody);
        }

        ResetJointsLambdas(physicsScene.distanceJoints);
        for (int i = 0; i < iterations; ++i)
        {
            SolveDistanceJoints(physicsScene.distanceJoints, substep_dt);
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
        for (auto &cc : collisionConstraints)
        {
            // Renderer::DrawSoftSoftPointEdgeCollision(cc);

            for (int i = 0; i < iterations; ++i)
                SolveSoftSoftCollisionConstraint(cc, substep_dt);
        }

        // update velocity
        for (SoftBody &sb : softBodies)
        {
            UpdateVelocities(sb.pointMasses, substep_dt);
        }
    }
}
