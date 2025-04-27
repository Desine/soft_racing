// file constraint_solver.hpp
#pragma once
#include "soft_body.hpp"

void SolveDistanceConstraints(PointMasses &pm, std::vector<DistanceConstraint> &constraints, float dt);
void SolveVolumeConstraints(PointMasses &pm, std::vector<VolumeConstraint> &constraints, float dt);
void SolveAngleConstraints(PointMasses &pm, std::vector<AngleConstraint> &constraints, float dt);
void SolveShapeMatchingConstraints(PointMasses &pm, std::vector<ShapeMatchingConstraint> &constraints, float dt);
void SolvePinConstraints(PointMasses &pm, std::vector<PinConstraint> &constraints, float dt);
