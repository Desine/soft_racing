//file Simulation.hpp
#pragma once
#include "soft_body.hpp"


void Simulate(std::vector<SoftBody> &softBodies, float dt, int substeps, int iterations, const glm::vec2 &gravity);