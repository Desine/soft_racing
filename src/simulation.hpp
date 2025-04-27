#pragma once
#include "soft_body.hpp"
#include "physics_scene.hpp"

void Simulate(PhysicsScene &physicsScene, float dt, int substeps, int iterations);