
#pragma once
#include "SoftBody.hpp"

void CreateWheel(
    SoftBody &wheel,
    const glm::vec2 &center,
    float wheelRadius,
    float diskMass,
    float tireMass,
    float tireRatio,
    float tireBodyCompliance,
    float tireTreadCompliance,
    float tirePressure,
    int radialSegments);
