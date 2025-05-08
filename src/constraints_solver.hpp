#pragma once
#include "soft_body.hpp"

void SolveDistanceConstraints(PointMasses &pm, std::vector<DistanceConstraint> &constraints, float dt);
void SolveVolumeConstraints(PointMasses &pm, std::vector<VolumeConstraint> &constraints, float dt);
void SolveAngleConstraints(PointMasses &pm, std::vector<AngleConstraint> &constraints, float dt);
void SolveShapeMatchingConstraints(PointMasses &pm, std::vector<ShapeMatchingConstraint> &constraints, float dt);
void SolvePinConstraints(PointMasses &pm, std::vector<PinConstraint> &constraints, float dt);

void SolveAccelerationConstraints(PointMasses &pm, std::vector<AccelerationConstraint> &constraints, float dt);
void SolveForceConstraints(PointMasses &pm, std::vector<ForceConstraint> &constraints, float dt);
void SolveVelocityConstraints(PointMasses &pm, std::vector<VelocityConstraint> &constraints, float dt);
void SolveAngularAccelerationConstraints(PointMasses &pm, std::vector<AngularAccelerationConstraint> &constraints, float dt);
void SolveAngularForceConstraints(PointMasses &pm, std::vector<AngularForceConstraint> &constraints, float dt);
void SolveAngularVelocityConstraints(PointMasses &pm, std::vector<AngularVelocityConstraint> &constraints, float dt);