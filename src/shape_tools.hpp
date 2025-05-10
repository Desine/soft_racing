#pragma once
#include "soft_body.hpp"
#include "glm/glm.hpp"

SoftBody CreateWheel(
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

std::vector<glm::vec2> CreatePoigonPositions(int segments, float radius, glm::vec2 origin = glm::vec2(0.0f, 0.0f), float rotate_angle = 0.0f);
void AddDistanceConstraintsToLoop(SoftBody &softBody, float compliance = 0.0f);
void AddVolumeConstraintToLoop(SoftBody &softBody, float compliance);
void AddCollisionPointsToLoop(SoftBody &softBody);
void AddCollisionShapeToLoop(SoftBody &softBody);