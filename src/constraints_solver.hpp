//file ConstraintSolver.hpp
#include "soft_body.hpp"

void SolveDistanceConstraints(PointMasses &pm, std::vector<DistanceConstraint> &constraints, float dt);
void SolveVolumeConstraints(PointMasses &pm, std::vector<VolumeConstraint> &constraints, float dt);
void SolvePinConstraints(PointMasses& pm, std::vector<PinConstraint>& constraints, float dt);