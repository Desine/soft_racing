
#include "CarTools.hpp"
#include <cmath>

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
    int radialSegments)
{

    // safety clamp
    radialSegments = std::clamp(radialSegments, 3, 30);
    tireRatio = std::clamp(tireRatio, 0.1f, 0.7f);

    // SoftBody clear
    wheel.Clear();

    // prepare values
    float diskPointMass = diskMass / (radialSegments + 1);
    float tirePointMass = tireMass / radialSegments;

    float innerRadius = wheelRadius * (1.0f - tireRatio);
    float outerRadius = wheelRadius;
    float angleStep = 2 * M_PI / radialSegments;
    float halfAngleStep = angleStep * 0.5f;

    // wheel.AddPoint(center, diskPointMass);

    std::vector<int> innerIds;
    std::vector<int> outerIds;
    for (int i = 0; i < radialSegments; i++)
    {
        float angle = i * angleStep;
        glm::vec2 diskDir = glm::vec2(std::cos(angle), std::sin(angle));
        glm::vec2 diskPos = center + diskDir * innerRadius;

        float tireAngle = angle - halfAngleStep;
        glm::vec2 tireDir = glm::vec2(std::cos(tireAngle), std::sin(tireAngle));
        glm::vec2 tirePos = center + tireDir * outerRadius;

        // innerIds.push_back(wheel.AddPoint(diskPos, diskPointMass));
        // outerIds.push_back(wheel.AddPoint(tirePos, tirePointMass));
    }

    for (int i = 0; i < radialSegments; i++)
    {
        // wheel.AddDistanceConstraint(0, innerIds[i], diskHubCompliance);                                       // center to disk
        // wheel.AddDistanceConstraint(innerIds[i], innerIds[(i + 1) % radialSegments], diskRimCompliance);      // disk to disk
        // wheel.AddDistanceConstraint(innerIds[i], outerIds[i], tireBodyCompliance);                         // disk to tire
        // wheel.AddDistanceConstraint(innerIds[i], outerIds[(i + 1) % radialSegments], tireBodyCompliance);  // disk to tire, shifted
        // wheel.AddDistanceConstraint(outerIds[i], outerIds[(i + 1) % radialSegments], tireTreadCompliance); // tire to tire
    }

    // wheel.AddVolumeConstraint(innerIds);
    // wheel.AddVolumeConstraint(outerIds, tirePressureCompliance);
    // wheel.volumeConstraints.back().restVolume *= tirePressure;
}
