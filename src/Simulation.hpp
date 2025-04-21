//file Simulation.hpp
#pragma once

#include "SoftBody.hpp"

void Simulate(SoftBody& softBody, float dt, int substeps, int iterations, const glm::vec2 &gravity);