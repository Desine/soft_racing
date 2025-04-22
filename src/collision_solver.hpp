#pragma once
#include "soft_body.hpp"

void GenerateCollisionConstraints(SoftBody& softBody, float groundY);
void SolveCollisionConstraints(PointMasses& pm, std::vector<CollisionConstraint>& constraints, float dt);
