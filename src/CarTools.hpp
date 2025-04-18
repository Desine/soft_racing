
#pragma once
#include "SoftBody.hpp"

void CreateWheel(
    SoftBody &wheel,
    const glm::vec2 &center,
    float wheelRadius,
    float diskMass,
    float tireMass,
    float tireRatio,
    float diskHubCompliance,
    float diskRimCompliance,
    float tireBodyCompliance,
    float tireTreadCompliance,
    float tirePressureCompliance,
    float tirePressure,
    int radialSegments);
