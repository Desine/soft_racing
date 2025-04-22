//file Simulation.hpp
#pragma once

#include "soft_body.hpp"

void Simulate(SoftBody& softBody, float dt, int substeps, int iterations, const glm::vec2 &gravity);