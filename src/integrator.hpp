//file Integrator.hpp
#pragma once
#include "soft_body.hpp"


void Integrate(PointMasses &pm, float dt, const glm::vec2 &gravity);
void UpdateVelocities(PointMasses &pm, float dt);