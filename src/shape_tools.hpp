#pragma once
#include "soft_body.hpp"
#include "glm/glm.hpp"

// SoftBody CreateWheel(
//     const glm::vec2 &center,
//     float wheelRadius,
//     float diskMass,
//     float tireMass,
//     float tireRatio,
//     float diskHubCompliance,
//     float diskRimCompliance,
//     float tireBodyCompliance,
//     float tireTreadCompliance,
//     float tirePressureCompliance,
//     float tirePressure,
//     int radialSegments);

// glm::vec2 center = glm::vec2(500, 1000);
// float wheelRadius = 200;
// float diskMass = 20;
// float tireMass = 5;
// float tireRatio = .4f;
// float diskHubCompliance = .0f;
// float diskRimCompliance = .0f;
// float tireBodyCompliance = .02f;
// float tireTreadCompliance = .05f;
// float tirePressureCompliance = .01f;
// float tirePressure = 1.f;
// int radialSegments = 12;

// SoftBody softBody = CreateWheel(center,
//             wheelRadius,
//             diskMass,
//             tireMass,
//             tireRatio,
//             diskHubCompliance,
//             diskRimCompliance,
//             tireBodyCompliance,
//             tireTreadCompliance,
//             tirePressureCompliance,
//             tirePressure,
//             radialSegments);

// ImGui::SliderFloat("Wheel Radius", &wheelRadius, 50.f, 500.f);
// ImGui::SliderFloat("Disk Mass", &diskMass, 1.f, 100.f);
// ImGui::SliderFloat("Tire Mass", &tireMass, 1.f, 100.f);
// ImGui::SliderFloat("Tire Ratio", &tireRatio, 0.1f, 0.9f);

// ImGui::SliderFloat("Disk Hub Compliance", &diskHubCompliance, .0f, 1.f);
// ImGui::SliderFloat("Disk Compliance", &diskRimCompliance, .0f, 1.f);
// ImGui::SliderFloat("Tire Body Compliance", &tireBodyCompliance, .0f, 1.f);
// ImGui::SliderFloat("Tire Tread Compliance", &tireTreadCompliance, .0f, 1.f);
// ImGui::SliderFloat("Tire Pressure Compliance", &tirePressureCompliance, .0f, 1.f);
// ImGui::SliderFloat("Tire Pressure", &tirePressure, 0.f, 10.f);

// ImGui::SliderInt("Radial Segments", &radialSegments, 3, 32);

std::vector<glm::vec2> CreatePoigonPositions(int segments, float radius, glm::vec2 origin = glm::vec2(0.0f, 0.0f));
void AddDistanceConstraintsToLoop(SoftBody &softBody, float compliance = 0.0f);
void AddVolumeConstraintToLoop(SoftBody &softBody, float compliance);
void AddCollisionPointsToLoop(SoftBody &softBody);
void AddCollisionShapeToLoop(SoftBody &softBody);