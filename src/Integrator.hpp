//file Integrator.hpp
#include "SoftBody.hpp"


void Integrate(PointMasses &pm, float dt, const glm::vec2 &gravity);
void UpdateVelocities(PointMasses &pm, float dt);